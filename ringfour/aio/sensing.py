"""Async sensing surfaces (spec section 10): cameras, lidar, and the
controller-level arm joint view (pre-MoveIt; motion lands in Phase 6).

Image/depth/cloud/scan arrays arrive through the payload transport layer
(ringfour.payloads) — descriptors over RPC, bytes via shared memory in
v1. Never raw socket reads (architecture ruling, 2026-06-12).
"""

from __future__ import annotations

import warnings
from typing import Any, AsyncIterator

from ..errors import CapabilityNotSupported
from ..data import DepthFrame, ImageFrame, JointInfo, LaserScan, PointCloud
from ..models import CameraIntrinsics, ForceTorque, Point, KinematicSnapshot
from ..models.sensing import PayloadRef
from ..payloads import fetch_array
from .transport import CLOSED, Connection


def _frame_from(result: dict[str, Any], conn=None) -> ImageFrame:
    return ImageFrame(
        id=result["frame_id"],
        image=fetch_array(PayloadRef.model_validate(result["payload"]), connection=conn),
        stamp=result["stamp"],
        camera=result["camera"],
    )


class Camera:
    """One camera (``robot.camera`` / ``robot.cameras[name]``)."""

    def __init__(self, conn: Connection, name: str) -> None:
        self._conn = conn
        self.name = name
        self._info: dict[str, Any] | None = None

    async def get_info(self) -> dict[str, Any]:
        """Camera metadata: intrinsics, has_depth, frame_ttl, frame.
        Cached after the first call."""
        if self._info is None:
            self._info = await self._conn.call("camera.info",
                                               {"camera": self.name})
        return self._info

    async def get_intrinsics(self) -> CameraIntrinsics:
        """Pinhole intrinsics from the camera_info topic. Raises
        RingfourError if the profile declares no info topic."""
        info = await self.get_info()
        if info["intrinsics"] is None:
            from ..errors import RingfourError
            raise RingfourError(
                f"camera {self.name!r} has no camera_info topic")
        return CameraIntrinsics.model_validate(info["intrinsics"])

    async def get_frame(self) -> ImageFrame:
        """The latest RGB frame. ``frame.id`` stays valid for deprojection
        for the profile's frame_ttl (default 30 s)."""
        result = await self._conn.call("camera.get_frame",
                                       {"camera": self.name})
        return _frame_from(result, self._conn)

    async def get_depth(self) -> DepthFrame:
        """The latest depth frame, meters."""
        result = await self._conn.call("camera.get_depth",
                                       {"camera": self.name})
        return DepthFrame(
            id=result["frame_id"],
            depth=fetch_array(PayloadRef.model_validate(result["payload"]), connection=self._conn),
            stamp=result["stamp"],
            camera=result["camera"],
        )

    async def get_synced(self) -> tuple[ImageFrame, DepthFrame]:
        """An RGB-D pair captured together (stamps within the agent's
        sync tolerance); both share one deprojection handle."""
        result = await self._conn.call("camera.get_synced",
                                       {"camera": self.name})
        frame = _frame_from(result["frame"], self._conn)
        d = result["depth"]
        depth = DepthFrame(
            id=d["frame_id"],
            depth=fetch_array(PayloadRef.model_validate(d["payload"]), connection=self._conn),
            stamp=d["stamp"],
            camera=self.name,
        )
        return frame, depth

    async def get_cloud(self, *, voxel: float) -> PointCloud:
        """A voxel-downsampled point cloud. ``voxel`` (meters) is
        mandatory — it is the size budget of this call; pick the coarsest
        voxel your task tolerates (0.01 = 1 cm is plenty for most)."""
        result = await self._conn.call(
            "camera.get_cloud", {"camera": self.name, "voxel": voxel},
            timeout=30.0,
        )
        colors = None
        if result["colors"] is not None:
            colors = fetch_array(PayloadRef.model_validate(result["colors"]), connection=self._conn)
        return PointCloud(
            points=fetch_array(PayloadRef.model_validate(result["points"]), connection=self._conn),
            colors=colors,
            stamp=result["stamp"],
            frame=result["frame"],
        )

    async def deproject(self, pixel: tuple[float, float],
                        frame: ImageFrame | DepthFrame | str,
                        in_frame: str | None = None) -> Point:
        """Pixel -> 3D point, server-side, using the depth and TF synced
        to THAT frame. Raises FrameExpired past frame_ttl."""
        frame_id = frame if isinstance(frame, str) else frame.id
        params: dict[str, Any] = {
            "frame_id": frame_id,
            "pixel": [float(pixel[0]), float(pixel[1])],
        }
        if in_frame is not None:
            params["in_frame"] = in_frame
        result = await self._conn.call("camera.deproject", params)
        return Point.model_validate(result["point"])

    async def stream(self, fps: float = 5.0) -> AsyncIterator[ImageFrame]:
        """Async generator of Frames at up to ``fps`` (server-clamped to
        the profile's max_stream_fps). Break or aclose() to stop."""
        queue = self._conn.subscribe("stream.data")
        result = await self._conn.call(
            "camera.stream_start", {"camera": self.name, "fps": fps})
        stream_id = result["stream_id"]
        try:
            async for params in _stream_items(queue, stream_id):
                yield _frame_from(params, self._conn)
        finally:
            self._conn.unsubscribe("stream.data", queue)
            await _quiet_stop(self._conn, "camera.stream_stop", stream_id)


class Lidar:
    """``robot.lidar`` (spec section 10)."""

    def __init__(self, conn: Connection) -> None:
        self._conn = conn

    async def scan(self) -> LaserScan:
        """The latest 2D sweep."""
        result = await self._conn.call("lidar.scan")
        return self._scan_from(result)

    async def stream(self, hz: float = 5.0) -> AsyncIterator[LaserScan]:
        """Async generator of LaserScans at up to ``hz``."""
        queue = self._conn.subscribe("stream.data")
        result = await self._conn.call("lidar.stream_start", {"hz": hz})
        stream_id = result["stream_id"]
        try:
            async for params in _stream_items(queue, stream_id):
                yield self._scan_from(params)
        finally:
            self._conn.unsubscribe("stream.data", queue)
            await _quiet_stop(self._conn, "lidar.stream_stop", stream_id)

    def _scan_from(self, result: dict[str, Any]) -> LaserScan:
        intensities = None
        if result.get("intensities") is not None:
            intensities = fetch_array(PayloadRef.model_validate(result["intensities"]), connection=self._conn)
        return LaserScan(
            ranges=fetch_array(PayloadRef.model_validate(result["ranges"]), connection=self._conn),
            angle_min=result["angle_min"],
            angle_max=result["angle_max"],
            angle_increment=result["angle_increment"],
            stamp=result["stamp"],
            frame=result["frame"],
            range_min=result.get("range_min", 0.0),
            range_max=result.get("range_max", float("inf")),
            intensities=intensities,
        )


class Tool:
    """The end effector on a group (spec 8.3).

    ``open``/``close`` drive the tool joint to the profile's declared
    extremes; ``set`` takes an explicit value in the joint's own units.

    ``is_holding`` is INFERRED, not measured. base101's gripper is a
    position controller: no action result, and `arm_6` exposes position
    and velocity but no effort, so there is no force to read. The rule
    (decision 27) is "commanded differs from actual AND the joint has
    stopped moving" — told to close further, and not moving, means
    pressing on something. Position error alone cannot tell a grasp from a
    jaw still travelling.
    """

    def __init__(self, conn: Connection, group: str,
                 config: Any, owner: Any = None) -> None:
        self._owner=owner
        self._conn = conn
        self.group = group
        self.type = config.type
        self.joint = config.joint
        self.open_at = config.open
        self.closed_at = config.closed

    async def start_position(self,position,*,execution_timeout=30.):
        from ..errors import CapabilityNotSupported
        from .execution import tool_operation
        if self._owner is None or not self._owner._welcome.features.get('verified_operations'):
            raise CapabilityNotSupported('agent does not support verified tool operations')
        task=await self._owner._tasks.start_task('group.start_tool',{'group':self.group,'position':position,'execution_timeout':execution_timeout})
        return tool_operation(self._owner,task)

    async def set(self,position,timeout=None,*,execution_timeout=30.):
        from .operations import complete
        return await complete(await self.start_position(position,execution_timeout=execution_timeout),timeout)

    async def open(self) -> float:
        if self.open_at is None:
            raise ValueError(
                f"group {self.group!r} tool declares no `open:` value; "
                f"use set() with an explicit position")
        return await self.set(self.open_at)

    async def close(self) -> float:
        if self.closed_at is None:
            raise ValueError(
                f"group {self.group!r} tool declares no `closed:` value; "
                f"use set() with an explicit position")
        return await self.set(self.closed_at)

    async def get_state(self) -> dict[str, Any]:
        """position, velocity, commanded, is_holding."""
        return await self._conn.call("group.tool_state",
                                     {"group": self.group})

    async def get_width(self) -> float:
        """The tool joint's current value."""
        return float((await self.get_state())["position"])

    async def get_is_holding(self) -> bool:
        """Inferred — see the class docstring."""
        return bool((await self.get_state())["is_holding"])


class Arm:
    """One actuated group: an arm, a head, a lift (spec 8).

    Named `Arm` still because `robot.arms` and `robot.arm` are shipped
    surface; `robot.groups[name]` reaches the same objects. State members
    are flat on the group deliberately (spec 8.1) — the watch-path grammar
    is flat too, and a namespace would make the two disagree.
    """

    def __init__(self, conn: Connection, name: str,
                 owner: Any = None) -> None:
        self._conn = conn
        self.name = name
        # The Robot that owns this handle. `ringfour.kinematics` needs it
        # to reach the group's declared shape and the description, both of
        # which arrive in the handshake - a group handle alone knows only
        # its name.
        self._owner = owner

    async def get_joints(self) -> dict[str, JointInfo]:
        """State of every arm joint, with URDF limits."""
        result = await self._conn.call("arm.joint_states",
                                       {"arm": self.name})
        return {
            name: JointInfo(
                position=j["position"],
                velocity=j["velocity"],
                effort=j["effort"],
                limits=tuple(j["limits"]) if j["limits"] else None,
            )
            for name, j in result["joints"].items()
        }

    async def get_joint_measurement(self,*,max_age=.5,timeout=1.,require_valid=True):
        from ..models.measurements import Measurement
        from ..models.execution import JointValue
        return Measurement[tuple[JointValue,...]].model_validate(await self._conn.call('measurement.get',
            {'kind':'joints','group':self.name,'max_age':max_age,'timeout':timeout,'require_valid':require_valid},timeout=timeout+2.))

    async def get_joint_positions(self) -> dict[str, float]:
        """{joint: position} for this arm (radians/meters)."""
        return {name: info.position
                for name, info in (await self.get_joints()).items()}

    async def get_kinematic_positions(self) -> dict[str, float]:
        """One measured snapshot for group planning, including external joints."""
        return (await self.kinematic_snapshot()).positions

    async def get_kinematic_snapshot(self, *, stamp: float | None = None) -> KinematicSnapshot:
        """Measured group and external joints with their common ROS timestamp."""
        result = await self._conn.call("arm.joint_states",
                                       {"arm": self.name, "include_external": True,
                                        **({"stamp":stamp} if stamp is not None else {})})
        if "positions" not in result:
            raise CapabilityNotSupported("agent lacks kinematic snapshot support")
        snapshot = KinematicSnapshot.model_validate(result)
        if stamp is not None and (snapshot.stamp is None or abs(snapshot.stamp-stamp)>1e-6):
            raise CapabilityNotSupported("Agent cannot provide measured joint feedback at the requested ROS time")
        return snapshot

    async def wrench(self) -> ForceTorque:
        """Latest wrist force/torque reading. Needs a profile
        force_torque entry for this arm."""
        result = await self._conn.call("arm.wrench", {"arm": self.name})
        return ForceTorque.model_validate(result)

    # -- static shape, from the handshake (no round trip) --------------------

    def configuration(self, positions):
        """Bind complete named joint values to this robot and current model."""
        from ringfour.configuration import configuration
        return configuration(self, positions)

    @property
    def _info(self) -> Any:
        if self._owner is None:
            raise TypeError(
                f"group {self.name!r} has no owning Robot; it cannot report "
                f"its shape")
        return self._owner.group_info(self.name)

    @property
    def kind(self) -> Any:
        """GroupKind.ARM | .POINTING | .LINEAR."""
        return self._info.kind

    @property
    def joint_names(self) -> tuple[str, ...]:
        """Canonical order for joint vectors — the controller's command
        order, resolved at agent startup, not the profile's guess."""
        return self._info.joints

    @property
    def tip(self) -> str | None:
        """The frame IK solves to; None disables it."""
        return self._info.tip

    @property
    def tool(self) -> "Tool | None":
        """The fitted end effector, or None when the profile declares
        none."""
        config = self._info.tool
        if config is None:
            return None
        return Tool(self._conn, self.name, config,self._owner)


    # -- motion (spec 8.2) ---------------------------------------------------

    @property
    def integration(self):
        from ..integration import GroupIntegration
        return GroupIntegration(self)

    def control(self,mode=None):
        """Lazy control scope; mode selects robotics behavior, not a backend name."""
        return self._control(mode)

    def _control(self, role: Any = None, *, raw: str | None = None
                ) -> Any:
        """Open a control session. Nothing moves outside one.

        Returns an async context manager; entering resolves the role,
        takes the motion lock and brings the controller up, and leaving
        restores it. ``raw=`` bypasses role resolution for vendor
        controllers outside the vocabulary — audited, and not portable.
        """
        from .control import ControlSession

        async def _open() -> ControlSession:
            params: dict[str, Any] = {"group": self.name}
            if raw is not None:
                params["raw"] = raw
            else:
                from ..enums import Controller
                params["role"] = str(Controller.TRAJECTORY if role is None else role)
            result = await self._conn.call("group.control_start", params)
            return ControlSession(
                self._owner, self.name, result["controller"],
                None if raw is not None else (Controller.TRAJECTORY if role is None else role),
                result.get("loaded", []), result.get("deactivated", []), result.get("session_id"))

        return _SessionOpener(_open)


async def _stream_items(queue: Any, stream_id: str) -> AsyncIterator[dict]:
    """Yield this stream's notifications until the agent ends it or the
    connection drops."""
    from ..errors import ConnectionLost, RingfourError
    while True:
        item = await queue.get()
        if item is CLOSED:
            raise ConnectionLost("connection lost mid-stream")
        if item.get("stream_id") != stream_id:
            continue
        if item.get("end"):
            if item.get("error"):
                raise RingfourError(
                    f"stream ended by agent: {item['error']}")
            return
        yield item


async def _quiet_stop(conn: Connection, method: str, stream_id: str) -> None:
    """Best-effort server-side stop when the consumer goes away."""
    try:
        await conn.call(method, {"stream_id": stream_id})
    except Exception:
        pass  # agent cleans up on disconnect anyway


class _SessionOpener:
    """Awaitable that is also an async context manager.

    So both spellings work, and the ``with`` form is the one that cannot
    leak a session:

        session = await group.control(Controller.TRAJECTORY)
        async with group.control(Controller.TRAJECTORY) as c: ...
    """

    def __init__(self, open_session: Any) -> None:
        self._open_session = open_session
        self._session: Any = None

    def __await__(self) -> Any:
        return self.open().__await__()

    async def open(self):
        if self._session is None:
            self._session = await self._open_session()
        return self._session

    async def __aenter__(self) -> Any:
        return await self.open()

    async def __aexit__(self, *exc_info: Any) -> None:
        if self._session is not None:
            await self._session.__aexit__(*exc_info)
