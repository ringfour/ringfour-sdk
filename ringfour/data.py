"""Array-bearing data types handed to user code (spec sections 5 and 10).

These are frozen dataclasses, not pydantic models: they carry numpy
arrays, which never cross the wire as JSON. The wire carries a
``PayloadRef`` descriptor (ringfour.models.sensing); the transport layer
(ringfour.payloads) maps it to the array, and these types are how the
user receives it.

Arrays are read-only views over shared memory (zero copy, local v1).
They stay valid for the lifetime of the Python object even after the
agent expires the segment — POSIX keeps mapped memory alive until the
mapping is dropped. Call ``np.array(x.image)`` if you need a private
mutable copy.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass(frozen=True)
class ImageFrame:
    """One RGB camera frame. ``id`` is the server-side handle for
    deprojection, valid for the profile's ``frame_ttl`` (default 30 s,
    sized for a cloud VLM round trip)."""

    id: str
    image: np.ndarray   # (H, W, 3) uint8
    stamp: float
    camera: str


@dataclass(frozen=True)
class DepthFrame:
    """One depth frame, meters."""

    id: str
    depth: np.ndarray   # (H, W) float32 meters
    stamp: float
    camera: str


@dataclass(frozen=True)
class PointCloud:
    """A voxel-downsampled point cloud from ``camera.get_cloud(voxel=)``."""

    points: np.ndarray          # (N, 3) float32, in ``frame``
    colors: np.ndarray | None   # (N, 3) uint8, or None without aligned RGB
    stamp: float
    frame: str
    # (N,) float32 lidar intensity; None for camera-derived clouds.
    intensities: np.ndarray | None = None


@dataclass(frozen=True)
class LaserScan:
    """One 2D lidar sweep (spec section 5)."""

    ranges: np.ndarray   # (N,) float32 meters; inf/nan = no return
    angle_min: float
    angle_max: float
    angle_increment: float
    stamp: float
    frame: str
    range_min: float = 0.0
    range_max: float = float("inf")
    # (N,) float32 reflectance; None when the driver provides none.
    intensities: np.ndarray | None = None


@dataclass(frozen=True)
class JointInfo:
    """One joint's state (spec section 5). ``limits`` from the URDF;
    None for continuous joints or before the robot description arrived."""

    position: float
    velocity: float
    effort: float
    limits: tuple[float, float] | None


@dataclass(frozen=True)
class OccupancyGrid:
    """A 2D grid map, from SLAM or loaded from disk (Phase 5 verbs).

    ``data`` follows the ROS convention: -1 unknown, 0 free, 100
    occupied. Coordinate queries take world meters in the map frame.
    """

    width: int            # cells
    height: int           # cells
    resolution: float     # meters per cell
    origin: Any           # Pose of cell (0, 0) in the world
    data: np.ndarray      # (H, W) int8

    def _cell(self, x: float, y: float) -> tuple[int, int]:
        col = int((x - self.origin.x) / self.resolution)
        row = int((y - self.origin.y) / self.resolution)
        if not (0 <= col < self.width and 0 <= row < self.height):
            raise ValueError(f"({x:g}, {y:g}) is outside the map")
        return row, col

    def cost_at(self, x: float, y: float) -> int:
        """Occupancy value at world (x, y): -1/0..100."""
        row, col = self._cell(x, y)
        return int(self.data[row, col])

    def is_free(self, x: float, y: float) -> bool:
        """True when the cell is known and unoccupied."""
        return self.cost_at(x, y) == 0

    def to_numpy(self) -> np.ndarray:
        """The raw (H, W) int8 grid."""
        return self.data


@dataclass(frozen=True)
class Observation:
    """Experimental synchronized-policy bundle; no stable acquisition producer.

    Not exported from the SDK root. Use Measurement for produced provenance.
    """

    stamp: float
    frames: dict[str, "ImageFrame"]
    joint_positions: dict[str, float]
    joint_velocities: dict[str, float]
    joint_efforts: dict[str, float]
    ee_poses: dict[str, Any]
    gripper_widths: dict[str, float]


@dataclass(frozen=True)
class Event:
    """One system event from ``robot.events()`` (estop, watch fires;
    nav aborts join in Phase 5)."""

    kind: str
    stamp: float
    data: dict[str, Any] = field(default_factory=dict)
