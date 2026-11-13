"""Synchronous JSON-RPC connection: a blocking facade over
ringfour.aio.transport.Connection.

The public API is blocking (spec: sync-first); user code never sees
asyncio. Failure modes are the aio layer's: ConnectionFailed on dial,
TimeoutError on call timeout, ConnectionLost on a dropped link, or the
ringfour exception the agent returned.
"""

from __future__ import annotations

from typing import Any

from .aio.transport import Connection as AioConnection
from .runner import Runner


class Connection:
    """One live connection to an agent. Thread-safe for concurrent calls."""

    def __init__(self, uri: str, timeout: float = 5.0) -> None:
        self._runner = Runner()
        try:
            self._conn: AioConnection = self._runner.run(
                AioConnection.open(uri, timeout=timeout)
            )
        except BaseException:
            self._runner.stop()
            raise

    def call(
        self,
        method: str,
        params: dict[str, Any] | None = None,
        timeout: float | None = None,
        *, command_id: str | None = None,
    ) -> Any:
        """Send one request and block for its result."""
        return self._runner.run(self._conn.call(method, params, timeout, command_id=command_id))

    def get_payload(
        self, payload_id: int, timeout: float | None = None
    ) -> tuple[str, dict[str, Any], bytes]:
        """Block until an announced binary payload arrives:
        (kind, meta, data)."""
        return self._runner.run(self._conn.get_payload(payload_id, timeout))

    def close(self) -> None:
        """Close the connection. Idempotent. Never raises."""
        try:
            self._runner.run(self._conn.close())
        except Exception:
            pass  # best effort; we are tearing down anyway
        self._runner.stop()

    @property
    def closed(self) -> bool:
        return self._conn.closed
