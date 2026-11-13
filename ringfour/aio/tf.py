"""Async TF API (spec section 6): robot.tf on the aio client.

Frame aliases (``Frame.MAP``/``.ODOM``/``.BASE``, i.e. "@map"/"@odom"/
"@base") resolve on the agent against the profile's frame names; results
always carry the resolved (real) names.
"""

from __future__ import annotations

from ..enums import Frame
from ..models import Point, Pose, Transform
from .transport import Connection


class TfAPI:
    """``robot.tf`` on the async client."""

    def __init__(self, conn: Connection) -> None:
        self._conn = conn

    async def lookup(self, parent: str, child: str,
                     timeout: float = 2.0, *, stamp: float | None = None) -> Transform:
        """Latest transform that re-expresses child-frame coordinates in
        the parent frame. Raises TimeoutError if TF has no data in time."""
        raw = await self._conn.call(
            "tf.lookup",
            {"parent": parent, "child": child, "timeout": timeout,
             **({"stamp": stamp} if stamp is not None else {})},
            timeout=timeout + 2.0,
        )
        return Transform.model_validate(raw)

    async def transform(self, entity: Pose | Point,
                        to_frame: str) -> Pose | Point:
        """Re-express a Pose or Point in another frame (server-side, so
        aliases and TF time handling stay in one place)."""
        kind = "pose" if isinstance(entity, Pose) else "point"
        result = await self._conn.call("tf.transform", {
            "kind": kind,
            "entity": entity.model_dump(mode="json"),
            "to_frame": to_frame,
        })
        model = Pose if kind == "pose" else Point
        return model.model_validate(result["entity"])

    async def pose_of(self, frame: str, in_frame: str = Frame.MAP, *, stamp: float | None = None) -> Pose:
        """The pose of a frame, in @map by default."""
        t = await self.lookup(in_frame, frame, stamp=stamp)
        return Pose(
            x=t.translation.x, y=t.translation.y, z=t.translation.z,
            q=t.rotation, frame=t.parent,
        )

    async def get_frames(self) -> list[str]:
        """All frame names currently in the TF tree."""
        result = await self._conn.call("tf.frames")
        return list(result["frames"])

    async def wait_for(self, frame: str, timeout: float = 5.0) -> None:
        """Block until the frame exists in TF; TimeoutError otherwise."""
        await self._conn.call(
            "tf.wait_for", {"frame": frame, "timeout": timeout},
            timeout=timeout + 2.0,
        )
