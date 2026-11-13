"""Sync sensing facades over ringfour.aio.sensing.

Same surface as the async classes; generators bridge the async
generators through the Runner and close them server-side when the
consumer breaks.
"""

from __future__ import annotations

import warnings
from typing import Any, Iterator

from .aio import sensing as aio_sensing
from .data import DepthFrame, ImageFrame, JointInfo, LaserScan, PointCloud
from .models import CameraIntrinsics, Point, KinematicSnapshot
from .runner import Runner


def _bridge(runner: Runner, agen) -> Iterator:
    """Drive an async generator from sync code, closing it on break."""
    try:
        while True:
            try:
                yield runner.run(agen.__anext__())
            except StopAsyncIteration:
                return
    finally:
        runner.run(agen.aclose())


class Camera:
    """One camera (``robot.camera`` / ``robot.cameras[name]``)."""

    def __init__(self, runner: Runner, aio: aio_sensing.Camera) -> None:
        self._runner = runner
        self._aio = aio
        self.name = aio.name

    def get_info(self) -> dict[str, Any]:
        """Camera metadata: intrinsics, has_depth, frame_ttl, frame.

        The aio Camera has always had this; the sync facade did not wrap
        it, so `frame` — the TF frame the camera's data is expressed in —
        was reachable only from async code.
        """
        return self._runner.run(self._aio.get_info())

    def get_intrinsics(self) -> CameraIntrinsics:
        """Pinhole intrinsics from the camera_info topic."""
        return self._runner.run(self._aio.get_intrinsics())

    def get_frame(self) -> ImageFrame:
        """The latest RGB frame. ``frame.id`` stays valid for deprojection
        for the profile's frame_ttl (default 30 s)."""
        return self._runner.run(self._aio.get_frame())

    def get_depth(self) -> DepthFrame:
        """The latest depth frame, meters."""
        return self._runner.run(self._aio.get_depth())

    def get_synced(self) -> tuple[ImageFrame, DepthFrame]:
        """An RGB-D pair captured together; one deprojection handle."""
        return self._runner.run(self._aio.get_synced())

    def get_cloud(self, *, voxel: float) -> PointCloud:
        """A voxel-downsampled point cloud; ``voxel`` (meters) is
        mandatory — the size budget of this call."""
        return self._runner.run(self._aio.get_cloud(voxel=voxel))

    def deproject(self, pixel: tuple[float, float],
                  frame: ImageFrame | DepthFrame | str,
                  in_frame: str | None = None) -> Point:
        """Pixel -> 3D point using the depth and TF synced to THAT frame.
        Raises FrameExpired past frame_ttl."""
        return self._runner.run(self._aio.deproject(pixel, frame, in_frame))

    def stream(self, fps: float = 5.0) -> Iterator[ImageFrame]:
        """Generator of Frames at up to ``fps`` (server-clamped).
        Break to stop."""
        return _bridge(self._runner, self._aio.stream(fps))


class Lidar:
    """``robot.lidar``."""

    def __init__(self, runner: Runner, aio: aio_sensing.Lidar) -> None:
        self._runner = runner
        self._aio = aio

    def scan(self) -> LaserScan:
        """The latest 2D sweep."""
        return self._runner.run(self._aio.scan())

    def stream(self, hz: float = 5.0) -> Iterator[LaserScan]:
        """Generator of LaserScans at up to ``hz``. Break to stop."""
        return _bridge(self._runner, self._aio.stream(hz))


class Tool:
    """The end effector on a group (spec 8.3). Sync facade."""

    def __init__(self, runner: Runner, aio: Any) -> None:
        self._runner = runner
        self._aio = aio
        self.type = aio.type
        self.joint = aio.joint
        self.open_at = aio.open_at
        self.closed_at = aio.closed_at

    def start_position(self,position,*,execution_timeout=30.):
        from .operations import Operation
        return Operation(self._runner,self._runner.run(self._aio.start_position(position,execution_timeout=execution_timeout)))

    def set(self,position,timeout=None,*,execution_timeout=30.):
        return self._runner.run(self._aio.set(position,timeout,execution_timeout=execution_timeout))

    def open(self) -> float:
        return self._runner.run(self._aio.open())

    def close(self) -> float:
        return self._runner.run(self._aio.close())

    def get_width(self) -> float:
        """The tool joint's current value."""
        return self._runner.run(self._aio.get_width())

    def get_is_holding(self) -> bool:
        """Inferred from commanded-vs-actual plus zero velocity — there is
        no force sensor on this gripper (decision 27)."""
        return self._runner.run(self._aio.get_is_holding())

    def get_state(self) -> dict[str, Any]:
        return self._runner.run(self._aio.get_state())


class Arm:
    """One actuated group: an arm, a head, a lift (spec 8).

    Named `Arm` still because `robot.arms` / `robot.arm` are shipped
    surface; `robot.groups[name]` reaches the same objects.
    """

    def __init__(self, runner: Runner, aio: aio_sensing.Arm,
                 owner: Any = None) -> None:
        self._runner = runner
        self._aio = aio
        self.name = aio.name
        self._owner = owner     # the sync Robot; see aio_sensing.Arm

    def configuration(self, positions):
        """Bind complete named joint values to this robot and current model."""
        from ringfour.configuration import configuration
        return configuration(self, positions)

    def get_joints(self) -> dict[str, JointInfo]:
        """State of every arm joint, with URDF limits."""
        return self._runner.run(self._aio.get_joints())

    def get_joint_measurement(self,*,max_age=.5,timeout=1.,require_valid=True):
        return self._runner.run(self._aio.get_joint_measurement(max_age=max_age,timeout=timeout,require_valid=require_valid))

    def get_joint_positions(self) -> dict[str, float]:
        """{joint: position} for this arm (radians/meters)."""
        return self._runner.run(self._aio.get_joint_positions())

    def get_kinematic_positions(self) -> dict[str, float]:
        """Measured robot joint snapshot; pass as state= to FK/IK/collisions."""
        return self._runner.run(self._aio.get_kinematic_positions())

    def get_kinematic_snapshot(self) -> KinematicSnapshot:
        """Measured positions and their common ROS timestamp, if available."""
        return self._runner.run(self._aio.get_kinematic_snapshot())

    def kinematic_snapshot_at(self, stamp: float) -> KinematicSnapshot:
        """Measured joint feedback at a recorded TF timestamp."""
        return self._runner.run(self._aio.get_kinematic_snapshot(stamp=stamp))

    def wrench(self):
        """Latest wrist force/torque reading (ForceTorque). Needs a
        profile force_torque entry for this arm."""
        return self._runner.run(self._aio.wrench())

    # -- static shape (handshake, no round trip) -----------------------------

    @property
    def kind(self) -> Any:
        """GroupKind.ARM | .POINTING | .LINEAR."""
        return self._aio.kind

    @property
    def joint_names(self) -> tuple[str, ...]:
        """Canonical order for joint vectors — the controller's command
        order, resolved at agent startup."""
        return self._aio.joint_names

    @property
    def tip(self) -> str | None:
        """The frame IK solves to; None disables it."""
        return self._aio.tip

    @property
    def tool(self) -> "Tool | None":
        aio_tool = self._aio.tool
        return None if aio_tool is None else Tool(self._runner, aio_tool)


    # -- typed watch handles (row 23) ----------------------------------------

    def watch_effort(self, joint: str, **limits: Any) -> Any:
        """Watch one joint's effort. The joint name is checked against
        this group's own joints, so a typo raises here rather than arming
        a monitor that can never fire."""
        return self._watch_joint("effort", joint, **limits)

    def watch_position(self, joint: str, **limits: Any) -> Any:
        return self._watch_joint("joint_positions", joint, **limits)

    def watch_wrench(self, component: str, axis: str, **limits: Any) -> Any:
        """Wrist force/torque, e.g. ("force", "z"). Needs a force_torque
        entry for this group in the profile."""
        if component not in ("force", "torque"):
            raise ValueError(
                f"component must be force or torque; got {component!r}")
        if axis not in ("x", "y", "z"):
            raise ValueError(f"axis must be x, y or z; got {axis!r}")
        return self._owner._watch_path(
            f"arms.{self.name}.wrench.{component}.{axis}", **limits)

    def _watch_joint(self, quantity: str, joint: str, **limits: Any) -> Any:
        if joint not in self.joint_names:
            raise ValueError(
                f"group {self.name!r} has no joint {joint!r}; it has "
                f"{list(self.joint_names)}")
        return self._owner._watch_path(
            f"arms.{self.name}.{quantity}.{joint}", **limits)

    # -- motion (spec 8.2) ---------------------------------------------------

    @property
    def integration(self):
        from .integration import GroupIntegration
        return GroupIntegration(self)

    def control(self,mode=None):
        """Lazy control scope; mode selects robotics behavior, not a backend name."""
        return self._control(mode)

    def _control(self, role: Any = None, *, raw: str | None = None) -> Any:
        """Open a control session; nothing moves outside one.

            with robot.arm.control(Controller.TRAJECTORY) as c:
                result = c.run(reviewed_trajectory)
                    ...
        """
        from .control import SessionOpener

        return SessionOpener(
            self._runner, self._aio._control(role, raw=raw))
