"""Mapping and explicitly resolved places; finite motion uses Operation."""

from __future__ import annotations

import time
from typing import Any

from ..data import OccupancyGrid
from ..enums import Phase, SlamMode
from ..errors import RingfourError
from ..models import MapInfo, Pose
from ..models.sensing import PayloadRef
from ..payloads import fetch_array
from .tasks import Task
from .transport import Connection


def goal_to_wire(goal:Pose):
    """Serialize an explicitly resolved framed pose."""
    if not isinstance(goal,Pose):raise TypeError('navigation requires an explicit framed Pose')
    return goal.model_dump(mode='json')





class SlamAPI:
    """``robot.slam`` (spec section 11), async."""

    def __init__(self, conn: Connection) -> None:
        self._conn = conn

    async def get_state(self) -> dict[str, Any]:
        """{mode, backend, active_map, localization_quality, is_lost,
        saved_maps} in one call."""
        result=await self._conn.call("slam.get_state")
        return {k:v for k,v in result.items() if k not in ("backend",)}

    async def get_mode(self) -> SlamMode:
        return SlamMode((await self.get_state())["mode"])


    async def start(self, *, preserve_graph: bool = False) -> None:
        """Ensure mapping mode; optionally retain the loaded pose graph."""
        if preserve_graph:
            if not (await self.get_state()).get("supports_preserve_graph"):
                from ..errors import CapabilityNotSupported
                raise CapabilityNotSupported("Agent cannot resume mapping without clearing its pose graph")
            await self._conn.call("slam.start", {"preserve_graph":True})
        else:
            await self._conn.call("slam.start")

    async def stop(self) -> None:
        """Pause map building; map + TF stay published (idle)."""
        await self._conn.call("slam.stop")

    async def save(self, name: str) -> MapInfo:
        """Serialize pose graph + 2D grid + metadata under map_dir."""
        result = await self._conn.call("slam.save", {"name": name},
                                       timeout=60.0)
        return MapInfo.model_validate(result["info"])

    async def load(self, name: str) -> None:
        """Load a saved map and continue mapping on top of it."""
        await self._conn.call("slam.load", {"name": name}, timeout=30.0)

    async def localize(self, pose: Pose | None = None,
                       name: str | None = None) -> None:
        """Switch to localization mode against a saved map (default:
        the active one, at the current pose estimate)."""
        params: dict[str, Any] = {}
        if pose is not None:
            params["pose"] = pose.model_dump(mode="json")
        if name is not None:
            params["name"] = name
        await self._conn.call("slam.localize", params, timeout=30.0)

    async def relocalize(self) -> None:
        """Re-run localization at the current pose estimate."""
        await self._conn.call("slam.relocalize", timeout=30.0)

    async def wait_for_localization(self, timeout: float = 10.0) -> float:
        """Block until localization quality is healthy; returns it.
        Raises TimeoutError otherwise."""
        result = await self._conn.call(
            "slam.wait_for_localization", {"timeout": timeout},
            timeout=timeout + 5.0)
        return float(result["localization_quality"])

    async def get_localization_quality(self) -> float:
        """0..1 heuristic (map->odom stability); also watchable as
        "slam.localization_quality"."""
        return float((await self.get_state())["localization_quality"])

    async def get_is_lost(self) -> bool:
        return bool((await self.get_state())["is_lost"])

    async def get_saved_maps(self) -> list[str]:
        return list((await self.get_state())["saved_maps"])

    async def map_info(self, name: str) -> MapInfo:
        result = await self._conn.call("slam.map_info", {"name": name})
        return MapInfo.model_validate(result["info"])

    async def delete(self, name: str) -> None:
        await self._conn.call("slam.delete", {"name": name})


class PlacesAPI:
    """``robot.places`` (spec section 17), async."""

    def __init__(self, conn: Connection) -> None:
        self._conn = conn

    async def save(self, name: str, pose: Pose | None = None) -> Pose:
        """Save a place on the active map; default = current pose."""
        params: dict[str, Any] = {"name": name}
        if pose is not None:
            params["pose"] = pose.model_dump(mode="json")
        result = await self._conn.call("places.save", params)
        return Pose.model_validate(result["pose"])

    async def get(self, name: str) -> Pose:
        """The named place's Pose. Raises UnknownPlace."""
        result = await self._conn.call("places.get", {"name": name})
        return Pose.model_validate(result["pose"])

    async def list(self) -> list[str]:
        """Place names on the active map."""
        return list((await self._conn.call("places.list"))["names"])

    async def delete(self, name: str) -> None:
        await self._conn.call("places.delete", {"name": name})


async def fetch_map(conn: Connection) -> OccupancyGrid:
    """robot.get_map(): the current occupancy grid (data over the payload
    seam)."""
    result = await conn.call("slam.get_map", timeout=15.0)
    return OccupancyGrid(
        width=result["width"],
        height=result["height"],
        resolution=result["resolution"],
        origin=Pose.model_validate(result["origin"]),
        data=fetch_array(PayloadRef.model_validate(result["payload"]), connection=conn),
    )
