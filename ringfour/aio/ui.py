"""Async power surfaces: power lanes and battery.

Power lanes are switchable MOSFET outputs; battery is the standard
BatteryState; ``is_on_battery`` reports the power source.

The LED and button surfaces that used to live here were removed with the
profile's ``ui:`` section (Cristi, 2026-08-23 — Phase 6 decision 16). The
module keeps its name because ``power`` is declared under it in every
shipped profile and renaming it would churn imports for no gain.
"""

from __future__ import annotations

from ..models import BatteryState, PowerLaneState
from .transport import Connection


class PowerLane:
    """One switchable power lane (``robot.power.lanes["main"]``)."""

    def __init__(self, conn: Connection, name: str) -> None:
        self._conn = conn
        self.name = name

    async def get_state(self) -> PowerLaneState:
        """Latest lane state: on, voltage, current."""
        result = await self._conn.call("power.lane_state",
                                        {"name": self.name})
        return PowerLaneState.model_validate(result)

    async def on(self) -> None:
        """Toggle this lane on."""
        await self._conn.call("power.set_lane",
                               {"name": self.name, "on": True})

    async def off(self) -> None:
        """Toggle this lane off."""
        await self._conn.call("power.set_lane",
                               {"name": self.name, "on": False})


class PowerAPI:
    """``robot.power``: battery + switchable power lanes."""

    def __init__(self, conn: Connection) -> None:
        self._conn = conn

    async def get_battery(self) -> BatteryState | None:
        """Battery state (voltage, level, current, charging)."""
        result = await self._conn.call("power.battery")
        return BatteryState.model_validate(result["battery"]) if result["battery"] is not None else None

    async def get_is_on_battery(self) -> bool | None:
        """True = running on battery, False = external power.
        None (returned as False) when no source topic is declared."""
        result = await self._conn.call("power.is_on_battery")
        on_battery = result.get("on_battery")
        return bool(on_battery) if on_battery is not None else None
