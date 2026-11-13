"""The sync Robot: a blocking facade over ringfour.aio.client.Robot.

Phase 2 scope: connection + handshake, capability surface with singular
sugar, audit API, task handles, and the Phase-3-placeholder navigate_to.
Every method here delegates to the async implementation; the facades are
written by hand (explicit over clever — no generated wrappers).
"""

from __future__ import annotations

import warnings
from types import MappingProxyType
from typing import Any, Iterator, Mapping

from . import aio
from .audit import AuditAPI
from .autonomy import PlacesAPI, SlamAPI
from .data import Event, OccupancyGrid
from .description import RobotDescription
from .models import (
    BatteryState,
    DispatchReceipt,
    GroupInfo,
    DiagnosticsReport,
    FluidPressure,
    Illuminance,
    ImuReading,
    MagReading,
    MobilityState,
    Pose,
    PowerLaneState,
    ProfileInfo,
    RangeReading,
    Temperature,
    Twist,
)
from .errors import CapabilityNotSupported
from .runner import Runner
from .sensing import Arm, Camera, Lidar
from .teleop import TeleopSession
from .tf import TfAPI
from .vendor import ActionGroup
from .watches import WatchHandle
from .watchers import SensorGroups, Watches


class Robot:
    """A connection to one robot via its ringfour agent.

    ``Robot(uri)`` connects, performs the handshake and fetches the
    capability report. With no URI it tries the default local unix socket,
    then localhost TCP. Raises ConnectionFailed if no target answers,
    ProtocolMismatch on protocol disagreement.

    Usable as a context manager; ``close()`` ends the connection. Running
    motions enter agent-side cancellation/cleanup on disconnect. Cleanup
    evidence and limitations depend on the advertised agent contract.
    """

    def __init__(self, uri: str | None = None, timeout: float = 5.0, *, transport=None) -> None:
        self._runner = Runner()
        try:
            self._robot: aio.Robot = self._runner.run(
                aio.Robot.connect(uri, timeout=timeout, transport=transport)
            )
        except BaseException:
            self._runner.stop()
            raise
        from .commands import Commands, Operations
        self.commands = Commands(self._runner, self._robot.commands)
        self.operations = Operations(self._runner, self._robot.operations)
        self.audit = AuditAPI(self._runner, self._robot.audit)
        self.tf = TfAPI(self._runner, self._robot.tf)
        # Wrap the aio handles once; capability checks stay in the aio
        # accessors, which these properties go through.
        self._groups = MappingProxyType(
            {name: Arm(self._runner, aio, owner=self)
             for name, aio in self._robot._groups.items()})
        # The SAME objects, not copies: robot.groups["x"] and
        # robot.arms["x"] must be one handle, or a mission that reaches
        # one way and locks the other would hold two.
        self._arms = MappingProxyType(
            {name: self._groups[name] for name in self._robot._arms
             if name in self._groups})
        self._cameras = MappingProxyType(
            {name: Camera(self._runner, cam)
             for name, cam in self._robot._cameras.items()})
        self._lidar = Lidar(self._runner, self._robot._lidar)
        self._slam = SlamAPI(self._runner, self._robot._slam)
        self._places = PlacesAPI(self._runner, self._robot._places)
        self._power = PowerAPI(self._runner, self._robot._power)
        self._power_lanes = MappingProxyType(
            {name: PowerLane(self._runner, lane)
             for name, lane in self._robot._power_lanes.items()})

    def close(self) -> None:
        """Close the connection. Idempotent."""
        try:
            self._runner.run(self._robot.close())
        except Exception:
            pass  # best effort; we are tearing down anyway
        self._runner.stop()

    def __enter__(self) -> "Robot":
        return self

    def __exit__(self, *exc_info: Any) -> None:
        self.close()

    # -- identity and capabilities ---------------------------------------------

    @property
    def uri(self) -> str:
        """The URI this robot was reached at.

        Useful for anything that needs to open a second connection to the
        same agent, and for telling an operator which robot they are
        looking at."""
        return self._robot.uri

    @property
    def name(self) -> str:
        """Robot name from the profile, e.g. "base101-kitchen"."""
        return self._robot.name

    @property
    def description(self) -> "RobotDescription | None":
        """The robot's own shape, parsed from `/robot_description`.

        Delivered in the handshake, so this is a local attribute read —
        see the aio property for what None means.
        """
        return self._robot.description

    @property
    def description_sha(self) -> str | None:
        """sha256 of the description this client is holding, or None."""
        return self._robot.description_sha

    def refresh_description(
        self, group: str | None = None,
    ) -> "RobotDescription | None":
        """Refetch the description and rebuild the cached chain."""
        return self._runner.run(self._robot.refresh_description(group))

    @property
    def profile(self) -> ProfileInfo:
        """Profile metadata: name, version, robot model, capabilities."""
        return self._robot.profile

    @property
    def capabilities(self) -> frozenset[str]:
        """The capability names this robot provides."""
        return self._robot.capabilities

    def has(self, capability: str) -> bool:
        """True if the robot's profile provides the named capability."""
        return self._robot.has(capability)

    # -- capability accessors (instances + singular sugar) -----------------------

    def group_info(self, name: str) -> "GroupInfo":
        """One group's static shape (kind, tip, dof_mask, joints, tool)."""
        return self._robot.group_info(name)

    @property
    def group_infos(self) -> Mapping[str, "GroupInfo"]:
        """Every group's static shape, by name."""
        return self._robot.group_infos

    @property
    def groups(self) -> Mapping[str, Arm]:
        """Every actuated structure, by name (spec 8): arms, heads, lifts."""
        self._robot.groups  # capability check
        return self._groups

    @property
    def arms(self) -> Mapping[str, Arm]:
        """The groups of kind ARM, e.g. {"left": Arm, "right": Arm}.

        Retained sugar over ``groups``.
        """
        self._robot.arms  # capability check
        return self._arms

    @property
    def arm(self) -> Arm:
        """The arm, when exactly one exists (else AmbiguousCapability)."""
        return self._arms[self._robot.arm.name]

    @property
    def cameras(self) -> Mapping[str, Camera]:
        """Named cameras, e.g. {"base": Camera, "head": Camera}."""
        self._robot.cameras  # capability check
        return self._cameras

    @property
    def camera(self) -> Camera:
        """The camera, when exactly one exists (else AmbiguousCapability)."""
        return self._cameras[self._robot.camera.name]


    @property
    def lidar(self) -> Lidar:
        """The 2D lidar (spec section 10)."""
        self._robot.lidar  # capability check
        return self._lidar

    # -- mobility state (Phase 3) ---------------------------------------------------

    def get_mobility_state(self) -> MobilityState:
        """Localized pose plus timestamped odometry in one request."""
        return self._runner.run(self._robot.get_mobility_state())

    def get_pose_measurement(self,*,frame='@map',max_age=.5,timeout=1.,require_valid=True):
        return self._runner.run(self._robot.get_pose_measurement(frame=frame,max_age=max_age,timeout=timeout,require_valid=require_valid))

    def get_pose(self,*,frame='@map',max_age=.5,timeout=1.):
        return self._runner.run(self._robot.get_pose(frame=frame,max_age=max_age,timeout=timeout))

    def get_velocity(self) -> Twist:
        """Current velocity (base frame), from odometry."""
        return self._runner.run(self._robot.get_velocity())

    def get_is_moving(self) -> bool:
        """True while the base moves faster than the standstill threshold."""
        return self._runner.run(self._robot.get_is_moving())

    def get_distance_traveled(self) -> float:
        """Odometric distance in meters since the agent started."""
        return self._runner.run(self._robot.get_distance_traveled())

    # -- teleop (Phase 3) -----------------------------------------------------------

    def teleop(self, watchdog: float = 0.5) -> TeleopSession:
        """Open a streamed-control session (context manager). Acquires the
        motion lock; SafetyViolation if another client holds it. The agent
        halts the robot if commands stop arriving within the watchdog."""
        return TeleopSession(self._runner, self._robot.teleop(watchdog))

    # -- navigation (Phase 5) ------------------------------------------------------

    @property
    def integration(self):
        from .integration import RobotIntegration
        return RobotIntegration(self)

    @property
    def signals(self):return self._robot.signals

    def guard(self,condition):
        from .guards import Guard
        return Guard(self._runner,self._robot.guard(condition))

    def get_signal(self,signal,*,max_age=.5,timeout=1.,require_valid=True):
        return self._runner.run(self._robot.get_signal(signal,max_age=max_age,timeout=timeout,require_valid=require_valid))

    def start_navigation(self,goal:Pose):
        """Submit now; returns an Operation with explicit result/update/close APIs."""
        from .operations import Operation
        return Operation(self._runner,self._runner.run(self._robot.start_navigation(goal)))

    def navigate_to(self,goal:Pose,timeout=None):
        """Return verified arrival; timeout stops and joins bounded cleanup."""
        return self._runner.run(self._robot.navigate_to(goal,timeout))

    def start_route(self,waypoints):
        from .operations import Operation
        return Operation(self._runner,self._runner.run(self._robot.start_route(waypoints)))

    def run_route(self,waypoints,timeout=None):
        return self._runner.run(self._robot.run_route(waypoints,timeout))




    def stop(self) -> int:
        """Immediate halt: cancels active navigation, zeroes velocity.
        No resume() (Q3) — re-send the goal."""
        return self._runner.run(self._robot.stop())

    def get_map(self) -> OccupancyGrid:
        """The current occupancy grid (requires slam)."""
        return self._runner.run(self._robot.get_map())

    @property
    def slam(self) -> SlamAPI:
        """SLAM orchestration: modes, save/load, localization."""
        self._robot.slam  # capability check
        return self._slam

    @property
    def places(self) -> PlacesAPI:
        """Named places on the active map."""
        self._robot.places  # capability check
        return self._places

    @property
    def power(self) -> "PowerAPI":
        """Power management: battery + switchable lanes."""
        self._robot.power  # capability check
        return self._power

    @property
    def power_lanes(self) -> Mapping[str, "PowerLane"]:
        """Named power lanes, e.g. {"main": PowerLane, "arms": PowerLane}."""
        self._robot.power_lanes  # capability check
        return self._power_lanes

    # -- watches / status / events (Phase 4) -----------------------------------------


    def _watch_path(self, path: str, *, above: float | None = None,
                    below: float | None = None,
                    outside: tuple[float, float] | None = None,
                    stop: bool = False, debounce: float | None = 0.0,
                    lifetime: float | None = None) -> WatchHandle:
        """The one place a path reaches the wire. Typed handles funnel
        here; nothing else should call it with a literal."""
        return WatchHandle(self._runner, self._robot._watch_path(
            path, above=above, below=below, outside=outside, stop=stop,
            debounce=debounce or 0.0, lifetime=lifetime))

    @property
    def watches(self) -> "Watches":
        """Typed watch handles for this robot's fixed quantities.

        `robot.watches.battery_level(below=15)` rather than
        an unvalidated dotted watch string. Per-instance quantities live
        on the thing that owns them: `robot.arm.watch_effort(joint, ...)`,
        `robot.sensors.<group>.watch_<signal>(...)`.
        """
        return Watches(self)

    @property
    def sensors(self) -> "SensorGroups":
        """Vendor signals this robot's profile declares (read-only)."""
        if not self._robot.has("sensors"):
            raise CapabilityNotSupported(
                f"robot {self.name!r} declares no sensors: section",
                robot=self.name)
        return SensorGroups(self, self._robot._welcome.sensors)

    @property
    def actions(self) -> ActionGroup:
        """Vendor verbs this robot's profile declares."""
        if not self._robot.has("actions"):
            raise CapabilityNotSupported(
                f"robot {self.name!r} declares no actions: section",
                robot=self.name)
        return ActionGroup(self, self._robot._welcome.actions)

    def _sensor_value(self,group,signal):
        return self._sensor_reading(group,signal)['value']

    def _sensor_reading(self, group: str, signal: str) -> dict[str, Any]:
        return self._runner.run(self._robot.connection.call(
            "sensor.read", {"group": group, "signal": signal}))

    def _invoke_action(self, name: str,
                       arguments: dict[str, Any]) -> DispatchReceipt:
        return self._runner.run(self._robot._invoke_action(name, arguments))

    def get_battery(self) -> BatteryState | None:
        """Battery state, or None if the profile maps no battery topic
        (or no data has arrived yet)."""
        return self._runner.run(self._robot.get_battery())

    def get_estop_engaged(self) -> bool:
        """True while the e-stop is engaged (all motion rejected)."""
        return self._runner.run(self._robot.get_estop_engaged())

    def diagnostics(self) -> DiagnosticsReport:
        """System health snapshot (profile status.diagnostics topic)."""
        return self._runner.run(self._robot.diagnostics())

    # -- latest-value sensors ------------------------------------------------------

    @property
    def range_sensors(self) -> tuple[str, ...]:
        """Names of the profile's single-beam rangers."""
        return self._robot.range_sensors

    def range(self, name: str) -> RangeReading:
        """Latest reading of one single-beam ranger."""
        return self._runner.run(self._robot.range(name))

    def imu(self) -> ImuReading:
        """Latest IMU reading."""
        return self._runner.run(self._robot.imu())

    def magnetometer(self) -> MagReading:
        """Latest magnetometer reading (profile imu.mag_topic)."""
        return self._runner.run(self._robot.magnetometer())

    def temperature(self) -> Temperature:
        """Environmental temperature (profile environment.temperature)."""
        return self._runner.run(self._robot.temperature())

    def pressure(self) -> FluidPressure:
        """Barometric/fluid pressure (profile environment.pressure)."""
        return self._runner.run(self._robot.pressure())

    def illuminance(self) -> Illuminance:
        """Ambient light level (profile environment.illuminance)."""
        return self._runner.run(self._robot.illuminance())

    def events(self) -> Iterator[Event]:
        """Generator of system events (estop changes, watch fires).
        The subscription is live once this returns (events are buffered
        until consumed). Break or close() to stop."""
        from .sensing import _bridge
        agen = self._runner.run(self._robot.events())
        return _bridge(self._runner, agen)

    # -- diagnostics ---------------------------------------------------------------

    def ping(self) -> float:
        """Round-trip latency to the agent, in seconds."""
        return self._runner.run(self._robot.ping())


class PowerLane:
    """One switchable power lane (``robot.power.lanes["main"]``). Sync
    facade over ringfour.aio.ui.PowerLane."""

    def __init__(self, runner: Runner, aio: aio.ui.PowerLane) -> None:
        self._runner = runner
        self._aio = aio

    @property
    def name(self) -> str:
        return self._aio.name

    def get_state(self) -> PowerLaneState:
        """Latest lane state: on, voltage, current."""
        return self._runner.run(self._aio.get_state())

    def on(self) -> None:
        """Toggle this lane on."""
        self._runner.run(self._aio.on())

    def off(self) -> None:
        """Toggle this lane off."""
        self._runner.run(self._aio.off())


class PowerAPI:
    """``robot.power``: battery + switchable power lanes. Sync facade
    over ringfour.aio.ui.PowerAPI."""

    def __init__(self, runner: Runner, aio: aio.ui.PowerAPI) -> None:
        self._runner = runner
        self._aio = aio

    def get_battery(self) -> BatteryState | None:
        """Battery state (voltage, level, current, charging)."""
        return self._runner.run(self._aio.get_battery())

    def get_is_on_battery(self) -> bool | None:
        """True = running on battery, False = external power."""
        return self._runner.run(self._aio.get_is_on_battery())
