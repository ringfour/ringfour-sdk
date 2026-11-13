"""Observational threshold handles with explicit remote polling methods."""

from __future__ import annotations

from typing import Any

from .aio.watches import WatchHandle as AioWatchHandle
from .runner import Runner


class WatchHandle:
    """One bridge-side watch. Use as a context manager."""

    def __init__(self, runner: Runner, aio: AioWatchHandle) -> None:
        self._runner = runner
        self._aio = aio
        self.path = aio.path

    def __enter__(self) -> "WatchHandle":
        self._runner.run(self._aio.__aenter__())
        return self

    def __exit__(self, *exc_info: Any) -> None:
        self._runner.run(self._aio.__aexit__(*exc_info))

    @property
    def id(self) -> str | None:
        """Agent watch identifier, available after entering the context."""
        return self._aio.id

    def get_triggered(self) -> bool:
        """True once the condition has fired (latched). Polls the agent."""
        return self._runner.run(self._aio.get_triggered())

    def get_value(self) -> float | None:
        """Last value the bridge observed (as of the last poll)."""
        self._runner.run(self._aio.refresh())
        return self._aio.value

    def get_triggered_at(self) -> float | None:
        """Timestamp of the first trigger, None before."""
        self._runner.run(self._aio.refresh())
        return self._aio.triggered_at

    def get_events(self) -> list[tuple[float, float]]:
        """All (stamp, value) trigger events so far (debounced edges)."""
        self._runner.run(self._aio.refresh())
        return self._aio.events
