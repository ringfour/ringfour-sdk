"""Sync audit API: blocking facade over ringfour.aio.audit.AuditAPI."""

from __future__ import annotations

from typing import Iterator

from .aio.audit import AuditAPI as AioAuditAPI
from .enums import AuditKind
from .models import AuditEvent
from .runner import Runner


class AuditAPI:
    """``robot.audit`` on the sync client (spec section 18)."""

    def __init__(self, runner: Runner, api: AioAuditAPI) -> None:
        self._runner = runner
        self._api = api

    def query(
        self,
        since: str | None = None,
        kind: AuditKind | None = None,
        task_id: str | None = None,
        limit: int = 500,
    ) -> list[AuditEvent]:
        """Query recent events (agent-side ring buffer). Newest last."""
        return self._runner.run(
            self._api.query(since=since, kind=kind, task_id=task_id,
                            limit=limit)
        )

    def tail(self) -> Iterator[AuditEvent]:
        """Live generator of audit events. Break to stop tailing."""
        agen = self._api.tail()
        try:
            while True:
                try:
                    yield self._runner.run(agen.__anext__())
                except StopAsyncIteration:
                    return
        finally:
            self._runner.run(agen.aclose())
