"""Mapping and explicitly resolved places; finite motion uses Operation."""

from __future__ import annotations

from typing import Any, Iterator

from .aio import autonomy as aio_autonomy
from .data import OccupancyGrid
from .enums import SlamMode
from .models import MapInfo, Pose
from .runner import Runner




class SlamAPI:
    """``robot.slam`` (spec section 11)."""

    def __init__(self, runner: Runner, aio: aio_autonomy.SlamAPI) -> None:
        self._runner = runner
        self._aio = aio

    def get_state(self) -> dict[str, Any]:
        """Mapping mode, active map, quality, saved maps, and backend support."""
        return self._runner.run(self._aio.get_state())

    def get_mode(self) -> SlamMode:
        """SlamMode.MAPPING | .LOCALIZATION | .IDLE."""
        return self._runner.run(self._aio.get_mode())


    def start(self, *, preserve_graph: bool = False) -> None:
        """Ensure mapping mode; optionally retain the loaded pose graph."""
        self._runner.run(self._aio.start(**({"preserve_graph":True} if preserve_graph else {})))

    def stop(self) -> None:
        """Pause map building; map + TF stay published (idle)."""
        self._runner.run(self._aio.stop())

    def save(self, name: str) -> MapInfo:
        """Serialize the pose graph + 2D grid + metadata."""
        return self._runner.run(self._aio.save(name))

    def load(self, name: str) -> None:
        """Load a saved map and continue mapping on top of it."""
        self._runner.run(self._aio.load(name))

    def localize(self, pose: Pose | None = None,
                 name: str | None = None) -> None:
        """Switch to localization mode against a saved map."""
        self._runner.run(self._aio.localize(pose, name))

    def relocalize(self) -> None:
        """Re-run localization at the current pose estimate."""
        self._runner.run(self._aio.relocalize())

    def wait_for_localization(self, timeout: float = 10.0) -> float:
        """Block until localized; returns the quality. TimeoutError
        otherwise."""
        return self._runner.run(self._aio.wait_for_localization(timeout))

    def get_localization_quality(self) -> float:
        """0..1 heuristic (map->odom stability)."""
        return self._runner.run(self._aio.get_localization_quality())

    def get_is_lost(self) -> bool:
        return self._runner.run(self._aio.get_is_lost())

    def get_saved_maps(self) -> list[str]:
        return self._runner.run(self._aio.get_saved_maps())

    def map_info(self, name: str) -> MapInfo:
        return self._runner.run(self._aio.map_info(name))

    def delete(self, name: str) -> None:
        self._runner.run(self._aio.delete(name))


class PlacesAPI:
    """``robot.places`` (spec section 17)."""

    def __init__(self, runner: Runner, aio: aio_autonomy.PlacesAPI) -> None:
        self._runner = runner
        self._aio = aio

    def save(self, name: str, pose: Pose | None = None) -> Pose:
        """Save a place on the active map; default = current pose."""
        return self._runner.run(self._aio.save(name, pose))

    def __getitem__(self, name: str) -> Pose:
        """robot.places["counter"] -> Pose. Raises UnknownPlace."""
        return self._runner.run(self._aio.get(name))

    def get(self, name: str) -> Pose:
        return self[name]

    def list(self) -> list[str]:
        """Place names on the active map."""
        return self._runner.run(self._aio.list())

    def delete(self, name: str) -> None:
        self._runner.run(self._aio.delete(name))


def fetch_map(runner: Runner, robot_aio: Any) -> OccupancyGrid:
    return runner.run(robot_aio.map())
