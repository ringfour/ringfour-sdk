"""The async Robot (ringfour.aio). The sync Robot in ringfour.client is a
blocking facade over this class.

Construct with ``await Robot.connect(uri)`` — async classes can't connect
in __init__.
"""

from __future__ import annotations

import time
import warnings
from types import MappingProxyType
from typing import Any, AsyncIterator, Mapping

from ..capabilities import require, single
from ..data import Event, OccupancyGrid
from ..description import RobotDescription
from ..errors import ConnectionFailed, ProtocolMismatch, CapabilityNotSupported
from ..models import (
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
    ProfileInfo,
    RangeReading,
    Temperature,
    Twist,
    Welcome,
)
from ..uri import DEFAULT_CANDIDATES
from ..version import CLIENT_VERSION, PROTOCOL_VERSION
from .audit import AuditAPI
from .autonomy import (
    PlacesAPI,
    SlamAPI,
    fetch_map,
    goal_to_wire,
)
from .sensing import Arm, Camera, Lidar
from .tasks import Task, TaskRouter
from .teleop import TeleopSession
from .tf import TfAPI
from .transport import CLOSED, Connection
from .ui import PowerAPI, PowerLane
from .watches import WatchHandle


class Robot:
    """An async connection to one robot via its ringfour agent.

    Same surface as the sync Robot, with awaitable methods. Capability
    accessors (groups, arms, cameras) raise CapabilityNotSupported when
    the profile lacks the capability and AmbiguousCapability when singular
    sugar is used with several instances.
    """

    def __init__(self, conn: Connection, welcome: Welcome) -> None:
        self._conn = conn
        from .commands import Commands, Operations
        self.commands = Commands(conn)
        self.operations = Operations(conn)
        self._welcome = welcome
        if welcome.features.get("connection_lease") is not None:
            conn.start_lease(welcome.features["connection_lease"])
        # Parsed lazily: a client that never asks about the robot's shape
        # should not pay to walk 62 links.
        self._description: "RobotDescription | None" = None
        self._group_models={}
        self._tasks = TaskRouter(conn, query_supported=bool(welcome.features.get("task_status")))
        self.audit = AuditAPI(conn)
        self.tf = TfAPI(conn)
        instances = welcome.instances
        # One handle class for every group, whatever its kind. A head is
        # not a different TYPE from an arm — it is a group with a
        # different `kind` and fewer joints, and giving non-arm groups a
        # name-only placeholder meant `robot.groups["head"].control(...)`
        # raised AttributeError. Caught by examples/manip/point_head.py,
        # which is what 6.7 exists for.
        self._groups = MappingProxyType(
            {name: Arm(conn, name, owner=self)
             for name in instances.get("groups", ())})
        # `robot.arms` is the ARM-kind subset of exactly those objects, so
        # robot.arm is robot.groups[name] — one handle, one motion lock.
        self._arms = MappingProxyType(
            {name: self._groups[name]
             for name in instances.get("arms", ())
             if name in self._groups})
        self._cameras = MappingProxyType(
            {name: Camera(conn, name)
             for name in instances.get("cameras", ())})
        # Static group shape from the handshake: kind, tip, dof_mask, the
        # resolved joint order, the tool. Kinematics is client-side, so it
        # has to be here rather than behind a call.
        self._group_infos = dict(welcome.groups)
        self._lidar = Lidar(conn)
        self._slam = SlamAPI(conn)
        self._places = PlacesAPI(conn)
        self._power = PowerAPI(conn)
        self._power_lanes = MappingProxyType(
            {name: PowerLane(conn, name)
             for name in instances.get("power_lanes", ())})

    @classmethod
    async def connect(cls, uri: str | None = None,
                      timeout: float = 5.0, *, transport=None) -> "Robot":
        """Dial, handshake, fetch the capability report.

        With no URI, tries the default local unix socket then localhost TCP.
        Raises ConnectionFailed / ProtocolMismatch.
        """
        candidates = [uri] if uri is not None else list(DEFAULT_CANDIDATES)
        failures: list[str] = []
        conn: Connection | None = None
        for candidate in candidates:
            try:
                conn = await Connection.open(candidate, timeout=timeout)
                break
            except ConnectionFailed as exc:
                failures.append(f"{candidate}: {exc}")
        if conn is None:
            raise ConnectionFailed(
                "could not connect to any target:\n  " + "\n  ".join(failures)
            )
        result = await conn.call(
            "hello",
            {"protocol": PROTOCOL_VERSION, "client_version": CLIENT_VERSION},
            timeout=timeout,
        )
        welcome = Welcome.model_validate(result)
        if welcome.protocol != PROTOCOL_VERSION:
            await conn.close()
            raise ProtocolMismatch(
                f"agent speaks protocol {welcome.protocol}, "
                f"client speaks {PROTOCOL_VERSION}"
            )
        from ..enums import Transport
        supported = welcome.features.get("payload_transports", [])
        try:
            selected = Transport(transport) if transport is not None else (Transport.SHM if conn.uri.startswith("unix:") else Transport.NET)
            if selected.value in supported:
                await conn.call("payload.set_transport", {"transport": selected.value})
            elif transport is not None:
                raise CapabilityNotSupported(f"agent does not support payload transport {selected.value!r}")
        except BaseException:
            await conn.close()
            raise
        return cls(conn, welcome)

    async def close(self) -> None:
        """Close the connection; the agent revokes and cleans up owned work."""
        self._tasks.close()
        await self._conn.close()

    async def __aenter__(self) -> "Robot":
        return self

    async def __aexit__(self, *exc_info: Any) -> None:
        await self.close()

    # -- identity and capabilities ---------------------------------------------

    @property
    def uri(self) -> str:
        """The URI this robot was reached at."""
        return self._conn.uri

    @property
    def name(self) -> str:
        """Robot name from the profile, e.g. "base101-kitchen"."""
        return self._welcome.profile.name

    @property
    def description(self) -> "RobotDescription | None":
        """The robot's own shape, parsed from `/robot_description`.

        Arrives in the handshake (spec 26.2), so this costs no round trip
        and no ROS. Parsed once and cached; ``refresh_description()``
        rebuilds it, which is what a ``model.changed`` event calls for.

        ``None`` means nothing had published on the topic when this client
        connected — the topic is latched, so that means the publisher was
        not up. It is not an error and it is not permanent.
        """
        if self._description is None and self._welcome.description:
            from ..description import RobotDescription
            self._description = RobotDescription.parse(
                self._welcome.description)
        return self._description

    @property
    def description_sha(self) -> str | None:
        """sha256 of the description this client is holding, or None."""
        return self._welcome.description_sha

    async def refresh_description(
        self, group: str | None = None,
    ) -> "RobotDescription | None":
        """Refetch the description and rebuild the cached chain.

        Call it on ``model.changed``. With ``group``, fetches that group's
        own description when its profile overrides the topic (namespaced
        launches) and the composed one otherwise — so a caller can ask
        uniformly without first checking which kind of robot it has.
        """
        from ..description import RobotDescription

        params = {} if group is None else {"group": group}
        result = await self._conn.call("model.description", params)
        parsed = RobotDescription.parse(result["description"])
        if group is None:
            self._description = parsed
            # Keep the sha reported by `description_sha` honest: it comes
            # off the handshake, and the handshake is now stale.
            self._welcome = self._welcome.model_copy(update={
                "description": result["description"],
                "description_sha": result["sha"],
            })
        else:
            self._group_models[group]=parsed
            self._welcome=self._welcome.model_copy(update={"group_description_shas":{**self._welcome.group_description_shas,group:parsed.sha}})
        return parsed

    @property
    def profile(self) -> ProfileInfo:
        """Profile metadata: name, version, robot model, capabilities."""
        return self._welcome.profile

    @property
    def capabilities(self) -> frozenset[str]:
        """The capability names this robot provides."""
        return frozenset(self._welcome.capabilities)

    def has(self, capability: str) -> bool:
        """True if the robot's profile provides the named capability."""
        return capability in self.capabilities

    # -- capability accessors (instances + singular sugar) -----------------------

    async def group_positions(self, name: str) -> dict[str, float]:
        """{joint: position} for one group, in command order.

        Used to fill TrajStatus during a follow; a plain read, no lock.
        """
        result = await self._conn.call("arm.joint_states", {"arm": name})
        return {joint: info["position"]
                for joint, info in result["joints"].items()}

    def group_info(self, name: str) -> "GroupInfo":
        """One group's static shape (kind, tip, dof_mask, joints, tool).

        What `ringfour.kinematics` needs to build a chain, and what a
        mission needs to know whether IK is even available.
        """
        self._require("groups")
        if name not in self._group_infos:
            raise KeyError(
                f"unknown group {name!r} (robot has "
                f"{sorted(self._group_infos)})")
        return self._group_infos[name]

    @property
    def group_infos(self) -> Mapping[str, "GroupInfo"]:
        """Every group's static shape, by name."""
        self._require("groups")
        return MappingProxyType(self._group_infos)

    @property
    def groups(self) -> Mapping[str, Arm]:
        """Every actuated structure, by name (spec 8): arms, heads, lifts."""
        self._require("groups")
        return self._groups

    @property
    def arms(self) -> Mapping[str, Arm]:
        """The groups of kind ARM, e.g. {"left": Arm, "right": Arm}.

        Retained sugar over ``groups`` - shipped in Phase 2/4 and used by
        examples and tests.
        """
        self._require("groups")
        return self._arms

    @property
    def arm(self) -> Arm:
        """The arm, when exactly one exists (else AmbiguousCapability)."""
        self._require("groups")
        return single(self._arms, "arms", self.name)

    @property
    def cameras(self) -> Mapping[str, Camera]:
        """Named cameras, e.g. {"base": Camera, "head": Camera}."""
        self._require("cameras")
        return self._cameras

    @property
    def camera(self) -> Camera:
        """The camera, when exactly one exists (else AmbiguousCapability)."""
        self._require("cameras")
        return single(self._cameras, "cameras", self.name)


    @property
    def lidar(self) -> Lidar:
        """The 2D lidar (spec section 10)."""
        self._require("lidar")
        return self._lidar

    @property
    def slam(self) -> SlamAPI:
        """SLAM orchestration (spec section 11): modes, save/load,
        localization."""
        self._require("slam")
        return self._slam

    @property
    def places(self) -> PlacesAPI:
        """Named places on the active map (spec section 17)."""
        self._require("places")
        return self._places

    @property
    def power(self) -> PowerAPI:
        """Power management: battery + switchable lanes."""
        self._require("power")
        return self._power

    @property
    def power_lanes(self) -> Mapping[str, PowerLane]:
        """Named power lanes, e.g. {"main": PowerLane, "arms": PowerLane}."""
        self._require("power")
        return self._power_lanes

    def _require(self, capability: str) -> None:
        require(self.capabilities, capability, self.name, self.profile.model)

    # -- watches / status / events (Phase 4) -------------------------------------

    def _watch_path(self, path: str, *, above: float | None = None,
              below: float | None = None,
              outside: tuple[float, float] | None = None,
              stop: bool = False, debounce: float = 0.0,
              lifetime: float | None = None) -> WatchHandle:
        """A bridge-side condition monitor over a dotted state path
        (async context manager). Exactly one of above/below/outside.
        ``stop=True`` makes the bridge halt all motion when it fires,
        whatever happens to this client; such watches survive disconnect
        for ``lifetime`` seconds (agent default applies when None)."""
        if stop:raise ValueError("stop=True is retired; use robot.guard(discovered_condition)")
        self._require("watches")
        return WatchHandle(self._conn, path, above=above, below=below,
                           outside=outside, stop=stop, debounce=debounce,
                           lifetime=lifetime)

    @property
    def actions(self):
        """Discovered vendor actions; awaiting a call returns a dispatch receipt."""
        from ..vendor import ActionGroup
        if not self.has("actions"):
            raise CapabilityNotSupported(f"robot {self.name!r} declares no actions")
        return ActionGroup(self, self._welcome.actions)

    @property
    def sensors(self):
        from ..watchers import SensorGroups
        self._require('sensors')
        return SensorGroups(self,self._welcome.sensors)

    @property
    def watches(self):
        from ..watchers import Watches
        return Watches(self)

    async def _sensor_reading(self,group,signal):
        return await self._conn.call('sensor.read',{'group':group,'signal':signal})

    async def _sensor_value(self,group,signal):
        return (await self._sensor_reading(group,signal))['value']

    async def _invoke_action(self, name, arguments) -> DispatchReceipt:
        if not self._welcome.features.get("vendor_dispatch_receipts"):
            raise CapabilityNotSupported("agent does not expose truthful vendor dispatch receipts")
        result = await self._conn.call("action.invoke", {"action": name, "arguments": arguments})
        return DispatchReceipt.model_validate(result)

    async def get_battery(self) -> BatteryState | None:
        """Battery state, or None if the profile maps no battery topic
        (or no data has arrived yet)."""
        self._require("status")
        result = await self._conn.call("status.get")
        if result["battery"] is None:
            return None
        return BatteryState.model_validate(result["battery"])

    async def get_estop_engaged(self) -> bool:
        """True while the e-stop is engaged (all motion rejected)."""
        self._require("status")
        result = await self._conn.call("status.get")
        return bool(result["estop"])

    async def diagnostics(self) -> DiagnosticsReport:
        """System health snapshot (profile status.diagnostics topic)."""
        self._require("status")
        result = await self._conn.call("status.diagnostics")
        return DiagnosticsReport.model_validate(result)

    # -- latest-value sensors ------------------------------------------------------

    @property
    def range_sensors(self) -> tuple[str, ...]:
        """Names of the profile's single-beam rangers."""
        self._require("range_sensors")
        return tuple(self._welcome.instances.get("range_sensors", ()))

    async def range(self, name: str) -> RangeReading:
        """Latest reading of one single-beam ranger."""
        self._require("range_sensors")
        result = await self._conn.call("range.read", {"name": name})
        return RangeReading.model_validate(result)

    async def imu(self) -> ImuReading:
        """Latest IMU reading."""
        self._require("imu")
        return ImuReading.model_validate(await self._conn.call("imu.read"))

    async def magnetometer(self) -> MagReading:
        """Latest magnetometer reading (profile imu.mag_topic)."""
        self._require("imu")
        return MagReading.model_validate(await self._conn.call("imu.mag"))

    async def temperature(self) -> Temperature:
        """Environmental temperature (profile environment.temperature)."""
        self._require("environment")
        return Temperature.model_validate(
            await self._conn.call("env.read", {"kind": "temperature"}))

    async def pressure(self) -> FluidPressure:
        """Barometric/fluid pressure (profile environment.pressure)."""
        self._require("environment")
        return FluidPressure.model_validate(
            await self._conn.call("env.read", {"kind": "pressure"}))

    async def illuminance(self) -> Illuminance:
        """Ambient light level (profile environment.illuminance)."""
        self._require("environment")
        return Illuminance.model_validate(
            await self._conn.call("env.read", {"kind": "illuminance"}))

    async def events(self) -> AsyncIterator[Event]:
        """Subscribe to system events (estop changes, watch fires; nav
        aborts join in Phase 5) and return their async iterator.

        Subscription is active from the moment this coroutine returns —
        events fired before the first ``anext`` are buffered, not lost.
        Break or aclose() to stop."""
        queue = self._conn.subscribe("event")
        try:
            await self._conn.call("events.subscribe")
        except BaseException:
            self._conn.unsubscribe("event", queue)
            raise

        async def iterate() -> AsyncIterator[Event]:
            try:
                while True:
                    item = await queue.get()
                    if item is CLOSED:
                        return
                    yield Event(kind=item["kind"], stamp=item["stamp"],
                                data=item.get("data") or {})
            finally:
                self._conn.unsubscribe("event", queue)
                try:
                    await self._conn.call("events.unsubscribe")
                except Exception:
                    pass  # connection may already be gone

        return iterate()

    # -- mobility state (Phase 3) ---------------------------------------------------
    # The sync API exposes these as properties; properties can't await, so
    # the aio mirror uses methods of the same names.

    async def get_mobility_state(self) -> MobilityState:
        """Localized pose plus timestamped odometry in one request."""
        self._require("mobility")
        return MobilityState.model_validate(await self._conn.call("mobility.get_state"))

    async def get_pose_measurement(self,*,frame='@map',max_age=.5,timeout=1.,require_valid=True):
        from ..models.measurements import Measurement
        self._require('mobility')
        return Measurement[Pose].model_validate(await self._conn.call('measurement.get',
            {'kind':'pose','frame':str(frame),'max_age':max_age,'timeout':timeout,'require_valid':require_valid},timeout=timeout+2.))

    async def get_pose(self,*,frame='@map',max_age=.5,timeout=1.):
        """Fresh measured pose in an explicit frame; unavailable/stale data raises."""
        return (await self.get_pose_measurement(frame=frame,max_age=max_age,timeout=timeout)).value

    async def get_velocity(self) -> Twist:
        """Current velocity (base frame), from odometry."""
        self._require("mobility")
        state = await self._conn.call("mobility.get_state")
        return Twist.model_validate(state["velocity"])

    async def get_is_moving(self) -> bool:
        """True while the base moves faster than the standstill threshold."""
        self._require("mobility")
        state = await self._conn.call("mobility.get_state")
        return bool(state["is_moving"])

    async def get_distance_traveled(self) -> float:
        """Odometric distance in meters since the agent started."""
        self._require("mobility")
        state = await self._conn.call("mobility.get_state")
        return float(state["distance_traveled"])

    # -- teleop (Phase 3) -----------------------------------------------------------

    def teleop(self, watchdog: float = 0.5) -> TeleopSession:
        """Open a streamed-control session (async context manager).
        Acquires the motion lock; raises SafetyViolation if another client
        holds it. The agent halts the robot if commands stop arriving for
        longer than the (server-clamped) watchdog period."""
        self._require("teleop")
        return TeleopSession(self._conn, watchdog)

    # -- navigation (Phase 5) -------------------------------------------------------

    @property
    def integration(self):
        from ..integration import RobotIntegration
        return RobotIntegration(self)

    @property
    def signals(self):
        from ..signals import Signals
        return Signals(self.name,self._welcome.features.get('signals',{}))

    def guard(self,condition):
        from .guards import Guard
        if not self._welcome.features.get('persistent_guards'):raise CapabilityNotSupported('agent does not support persistent guards')
        return Guard(self,condition)

    async def get_signal(self,signal,*,max_age=.5,timeout=1.,require_valid=True):
        from ..signals import Signal
        from ..models.measurements import Measurement
        if not isinstance(signal,Signal) or signal.robot_id!=self.name:raise TypeError('expected a signal discovered on this robot')
        return Measurement[float].model_validate(await self._conn.call('signal.get',
            {'path':signal.path,'max_age':max_age,'timeout':timeout,'require_valid':require_valid},timeout=timeout+2.))

    async def start_navigation(self,goal:Pose):
        """Submit now and return an Operation; updates only observe accepted work."""
        self._require('mobility')
        if not isinstance(goal,Pose):raise TypeError('navigation requires an explicit framed Pose')
        if not self._welcome.features.get('verified_operations'):
            raise CapabilityNotSupported('agent does not support verified operations')
        from .execution import navigation_operation
        task=await self._tasks.start_task('mobility.navigate_to',{'goal':goal_to_wire(goal)})
        return navigation_operation(self,task)

    async def navigate_to(self,goal:Pose,timeout=None):
        """Wait for verified arrival; timeout/cancellation performs bounded cleanup."""
        from .operations import complete
        return await complete(await self.start_navigation(goal),timeout)

    async def start_route(self,waypoints):
        """Submit a finite route; every endpoint is verified before advancing."""
        self._require('mobility')
        waypoints=tuple(waypoints)
        if not waypoints or not all(isinstance(p,Pose) for p in waypoints):raise TypeError('route requires explicit framed Poses')
        if not self._welcome.features.get('verified_operations'):raise CapabilityNotSupported('agent does not support verified operations')
        from .execution import route_operation
        task=await self._tasks.start_task('mobility.start_route',{'goals':[goal_to_wire(p) for p in waypoints]})
        return route_operation(self,task,len(waypoints))

    async def run_route(self,waypoints,timeout=None):
        from .operations import complete
        return await complete(await self.start_route(waypoints),timeout)




    async def stop(self) -> int:
        """Immediate halt: cancels active navigation and zeroes the
        velocity. There is no resume() (Q3) — re-send the goal."""
        self._require("mobility")
        result = await self._conn.call("mobility.stop")
        return int(result["cancelled"])

    async def get_map(self) -> OccupancyGrid:
        """The current occupancy grid (requires slam)."""
        self._require("slam")
        return await fetch_map(self._conn)

    # -- diagnostics ---------------------------------------------------------------

    async def ping(self) -> float:
        """Round-trip latency to the agent, in seconds."""
        start = time.monotonic()
        await self._conn.call("ping")
        return time.monotonic() - start

    # -- low-level escape hatch for tests/debugging ---------------------------------

    @property
    def connection(self) -> Connection:
        """The underlying connection. Not part of the stable API."""
        return self._conn
