"""Async client API for the agent's audit log (spec section 18)."""

from __future__ import annotations

import asyncio
from typing import AsyncIterator

from ..enums import AuditKind
from ..errors import ConnectionLost
from ..models import AuditEvent
from .transport import CLOSED, Connection


class AuditAPI:
    """``robot.audit`` on the async client."""

    def __init__(self, conn: Connection) -> None:
        self._conn = conn

    async def query(
        self,
        since: str | None = None,
        kind: AuditKind | None = None,
        task_id: str | None = None,
        limit: int = 500,
    ) -> list[AuditEvent]:
        """Query recent events from the agent's in-memory ring buffer.

        ``since`` is an ISO 8601 timestamp; ``kind`` an ``AuditKind``;
        ``task_id`` filters one task's lifecycle. Newest events last.
        """
        result = await self._conn.call("audit.query", {
            "since": since, "kind": kind, "task_id": task_id, "limit": limit,
        })
        return [AuditEvent.model_validate(item) for item in result["events"]]

    async def tail(self) -> AsyncIterator[AuditEvent]:
        """Live stream of audit events. Iterate; break (or aclose) to stop.

        Events recorded before the first iteration are not replayed (use
        query() for history). Raises ConnectionLost if the connection drops
        mid-stream.
        """
        queue = self._conn.subscribe("audit.event")
        await self._conn.call("audit.subscribe")
        try:
            while True:
                item = await queue.get()
                if item is CLOSED:
                    raise ConnectionLost("connection lost while tailing audit")
                yield AuditEvent.model_validate(item)
        finally:
            # No awaits here: this finally also runs via GeneratorExit
            # (break / aclose), where awaiting is forbidden. The server-side
            # unsubscribe is fire-and-forget; disconnect cleans up anyway.
            self._conn.unsubscribe("audit.event", queue)
            if not self._conn.closed:
                asyncio.ensure_future(self._unsubscribe_quietly())

    async def _unsubscribe_quietly(self) -> None:
        try:
            await self._conn.call("audit.unsubscribe")
        except Exception:
            pass  # best effort; the agent drops the flag on disconnect
