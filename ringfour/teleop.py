"""Sync teleop session: blocking facade over ringfour.aio.teleop.

    with robot.teleop(watchdog=0.5) as t:
        t.drive(linear=0.2, angular=0.0)
"""

from __future__ import annotations

from typing import Any

from .aio.teleop import TeleopSession as AioTeleopSession
from .runner import Runner


class TeleopSession:
    """One streamed-control session. Use as a context manager."""

    def __init__(self, runner: Runner, session: AioTeleopSession) -> None:
        self._runner = runner
        self._session = session

    def open(self):
        self._runner.run(self._session.open());return self

    def close(self):return self._runner.run(self._session.close())

    def __enter__(self) -> "TeleopSession":
        self._runner.run(self._session.__aenter__())
        return self

    def __exit__(self, *exc_info: Any) -> None:
        self._runner.run(self._session.__aexit__(*exc_info))

    def drive(self, linear: float = 0.0, angular: float = 0.0,
              lateral: float = 0.0) -> None:
        """One velocity command (m/s, rad/s), clamped server-side. Re-send
        within the watchdog period or the robot halts."""
        self._runner.run(self._session.drive(linear, angular, lateral))

    def stop(self) -> None:
        """Immediate zero velocity; the session stays alive."""
        self._runner.run(self._session.stop())

    @property
    def watchdog(self) -> float | None:
        """The granted (server-clamped) watchdog period, seconds."""
        return self._session.watchdog

    @property
    def latency(self) -> float | None:
        """Rolling round-trip estimate of drive commands, seconds."""
        return self._session.latency
