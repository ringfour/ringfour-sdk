"""Sync TF API: blocking facade over ringfour.aio.tf.TfAPI."""

from __future__ import annotations

from .aio.tf import TfAPI as AioTfAPI
from .enums import Frame
from .models import Point, Pose, Transform
from .runner import Runner


class TfAPI:
    """``robot.tf`` on the sync client (spec section 6)."""

    def __init__(self, runner: Runner, api: AioTfAPI) -> None:
        self._runner = runner
        self._api = api

    def lookup(self, parent: str, child: str,
               timeout: float = 2.0, *, stamp: float | None = None) -> Transform:
        """Latest transform child->parent. TimeoutError if TF has no data."""
        return self._runner.run(self._api.lookup(parent, child, timeout, **({"stamp":stamp} if stamp is not None else {})))

    def transform(self, entity: Pose | Point,
                  to_frame: str) -> Pose | Point:
        """Re-express a Pose or Point in another frame."""
        return self._runner.run(self._api.transform(entity, to_frame))

    def pose_of(self, frame: str, in_frame: str = Frame.MAP, *, stamp: float | None = None) -> Pose:
        """The pose of a frame, in @map by default."""
        return self._runner.run(self._api.pose_of(frame, in_frame, **({"stamp":stamp} if stamp is not None else {})))

    def get_frames(self) -> list[str]:
        """All frame names currently in the TF tree."""
        return self._runner.run(self._api.get_frames())

    def wait_for(self, frame: str, timeout: float = 5.0) -> None:
        """Block until the frame exists in TF; TimeoutError otherwise."""
        self._runner.run(self._api.wait_for(frame, timeout))
