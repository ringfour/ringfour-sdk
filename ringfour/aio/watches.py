"""Observational threshold handles with explicit remote polling methods."""

from __future__ import annotations

from typing import Any

from .transport import Connection


class WatchHandle:
    """One bridge-side watch. Use as an async context manager."""

    def __init__(self, conn: Connection, path: str, *,
                 above: float | None, below: float | None,
                 outside: tuple[float, float] | None,
                 stop: bool, debounce: float,
                 lifetime: float | None) -> None:
        self._conn = conn
        self._params: dict[str, Any] = {
            "path": path, "above": above, "below": below,
            "outside": list(outside) if outside is not None else None,
            "stop": stop, "debounce": debounce,
        }
        if lifetime is not None:
            self._params["lifetime"] = lifetime
        self.path = path
        self.id: str | None = None
        self.value: float | None = None
        self.triggered_at: float | None = None
        self.events: list[tuple[float, float]] = []
        self._triggered = False

    async def __aenter__(self) -> "WatchHandle":
        result = await self._conn.call("watch.start", self._params)
        self.id = result["watch_id"]
        return self

    async def __aexit__(self, *exc_info: Any) -> None:
        if self.id is None:
            return
        try:
            await self._conn.call("watch.stop", {"watch_id": self.id})
        except Exception:
            pass  # agent removes plain watches on disconnect anyway

    async def get_triggered(self) -> bool:
        """Refresh from the agent; True once the condition has fired
        (latched)."""
        await self.refresh()
        return self._triggered

    async def refresh(self) -> None:
        result = await self._conn.call("watch.poll", {"watch_id": self.id})
        self._triggered = result["triggered"]
        self.value = result["value"]
        self.triggered_at = result["triggered_at"]
        self.events = [tuple(e) for e in result["events"]]
