"""Async teleop session (spec section 14).

    async with robot.teleop(watchdog=0.5) as t:
        await t.drive(linear=0.2, angular=0.0)

The session holds the robot's motion lock. The agent enforces the
watchdog: stop sending commands and the robot halts within the window,
whatever happened to the client.
"""

from __future__ import annotations

import asyncio
import time
from collections import deque
from typing import Any

from .transport import Connection


class TeleopSession:
    """One streamed-control session. Use as an async context manager."""

    def __init__(self, conn: Connection, watchdog: float) -> None:
        self._conn = conn
        self._requested_watchdog = watchdog
        self._opened=False
        self._cleanup=None
        self.watchdog: float | None = None  # granted value, set on enter
        self._rtts: deque[float] = deque(maxlen=20)

    async def open(self):
        if self._opened:raise RuntimeError('teleop scope already opened')
        result=await self._conn.call('teleop.start',{'watchdog':self._requested_watchdog})
        self.watchdog=result['watchdog'];self._opened=True
        return self

    async def close(self):
        if not self._opened:return
        if self._cleanup is None:self._cleanup=asyncio.create_task(self._conn.call('teleop.end',timeout=60.))
        async def join():return await asyncio.shield(self._cleanup)
        from ..lifecycle import exit_scope
        await exit_scope(join,(),timeout=60.)

    async def __aenter__(self):return await self.open()
    async def __aexit__(self,*exc):
        from ..lifecycle import exit_scope
        await exit_scope(self.close,exc,timeout=60.)

    async def drive(self, linear: float = 0.0, angular: float = 0.0,
                    lateral: float = 0.0) -> None:
        """One velocity command (m/s, rad/s), clamped server-side. Must be
        re-sent within the watchdog period or the robot halts. ``lateral``
        raises CapabilityNotSupported on non-omni bases."""
        start = time.monotonic()
        await self._conn.call("teleop.drive", {
            "linear": linear, "angular": angular, "lateral": lateral,
        })
        self._rtts.append(time.monotonic() - start)

    async def stop(self) -> None:
        """Immediate zero velocity; the session stays alive."""
        await self._conn.call("teleop.stop")

    @property
    def latency(self) -> float | None:
        """Rolling round-trip estimate of drive commands, seconds."""
        if not self._rtts:
            return None
        return sum(self._rtts) / len(self._rtts)
