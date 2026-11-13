"""Navigation and manipulation status types.

Defined types-first: NavStatus / Frontier / MapInfo are produced by the
Phase 5 autonomy verbs, TrajStatus by the Phase 6 manipulation track.
Having them in the SDK now keeps the wire schema (protocol.json) ahead
of the verbs and lets user code import against the final shapes.
"""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict

from ..enums import Phase
from .spatial import Pose


class NavStatus(BaseModel):
    """Yielded by the ``navigate()`` generator (Phase 5)."""

    model_config = ConfigDict(frozen=True)

    # Navigation never reports Phase.PLANNING (Nav2 plans inside MOVING).
    phase: Phase
    progress: float                 # 0..1, estimated
    pose: Pose
    remaining_distance: float       # meters, estimated
    waypoint_index: int = 0
    stamp: float


class TrajStatus(BaseModel):
    """Yielded by ``arm.execute()`` during trajectory execution (Phase 6)."""

    model_config = ConfigDict(frozen=True)

    # Trajectory execution never reports Phase.RECOVERING.
    phase: Phase
    progress: float
    joint_positions: dict[str, float]
    ee_pose: Pose | None = None
    effort: dict[str, float] = {}
    error: str | None = None
    # DID THE JOINTS ACTUALLY GET THERE. `phase` says what the controller
    # did; this says what the robot did, and they are not the same claim.
    # A trajectory given too little time finishes its schedule and stops
    # part way, which the controller calls success. None while running.
    arrived: bool | None = None
    # Final joint minus commanded, per joint, in radians or metres. Filled
    # on the terminal status; empty while running.
    residual: dict[str, float] = {}

class Frontier(BaseModel):
    """A candidate exploration target (Phase 5)."""

    model_config = ConfigDict(frozen=True)

    pose: Pose
    area: float        # estimated unmapped area beyond, m^2
    distance: float    # from current position, meters


class MapInfo(BaseModel):
    """Metadata about a saved map (Phase 5)."""

    model_config = ConfigDict(frozen=True)

    name: str
    created_at: str    # ISO timestamp
    resolution: float  # meters per cell
    dimensions: tuple[float, float]  # (width, height) meters
