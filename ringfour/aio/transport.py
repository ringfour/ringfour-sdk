"""Async JSON-RPC connection over a WebSocket (unix socket or TCP).

This is the real transport implementation; the sync API in
``ringfour.transport`` is a blocking facade over it (decided by Cristi
2026-06-11: ship ringfour.aio from the start).

Failure modes:
- ``Connection.open(...)`` raises ConnectionFailed if the dial fails.
- ``call(...)`` raises TimeoutError on timeout, ConnectionLost if the link
  drops mid-call, or the ringfour exception the agent returned.
"""

from __future__ import annotations

import asyncio
import itertools
import json
import math
import time
import uuid
from typing import Any

import websockets

from .. import wire
from ..errors import (
    ConnectionFailed,
    ConnectionLost,
    RingfourError,
    TimeoutError,
    SubmissionUnknown,
    error_from_wire,
)
from ..uri import parse_uri

# How many announced-but-unretrieved binary payloads to keep before dropping
# the oldest. Protects memory if a stream is abandoned mid-flight.
_MAX_PENDING_PAYLOADS = 64

# Subscription queues drop the oldest item beyond this depth: a slow
# consumer must not make the receive loop block (and with it every call).
_MAX_QUEUE_DEPTH = 256

# Largest single WebSocket message we accept. websockets defaults to 1 MiB,
# which a raw camera frame easily exceeds (1080p RGB ≈ 6 MiB, depth ≈ 8 MiB);
# an oversized frame aborts the whole connection, so the cap must be generous.
# Must match the agent server's limit.
MAX_MESSAGE_BYTES = 64 * 1024 * 1024

# Sentinel pushed into subscription queues when the connection dies, so
# consumers blocked on queue.get() wake up.
CLOSED = object()


class _Payload:
    """One announced binary payload being assembled."""

    def __init__(self, kind: str, meta: dict[str, Any]) -> None:
        self.kind = kind
        self.meta = meta
        self.future: asyncio.Future = asyncio.get_running_loop().create_future()


class Connection:
    """One live connection to an agent. Construct via ``await
    Connection.open(uri)``."""

    def __init__(self) -> None:
        self._ws: Any = None
        self._uri = ""
        self._timeout = 5.0
        self._closed = False
        self._ids = itertools.count(1)
        self._pending: dict[int, asyncio.Future] = {}
        self._payloads: dict[int, _Payload] = {}
        # notification method -> queues of interested consumers
        self._subscribers: dict[str, list[asyncio.Queue]] = {}
        self._receiver: asyncio.Task | None = None
        self._lease_renewer: asyncio.Task | None = None
        self.command_contract = None
        self._command_clock = 0.0

    @classmethod
    async def open(cls, uri: str, timeout: float = 5.0) -> "Connection":
        """Dial the agent. Raises ConnectionFailed if the target does not
        answer within ``timeout``."""
        conn = cls()
        conn._uri = uri
        conn._timeout = timeout
        target = parse_uri(uri)
        try:
            if target.kind == "unix":
                conn._ws = await asyncio.wait_for(
                    websockets.unix_connect(target.path, uri=target.ws_uri,
                                            max_size=MAX_MESSAGE_BYTES),
                    timeout,
                )
            else:
                conn._ws = await asyncio.wait_for(
                    websockets.connect(target.ws_uri,
                                       max_size=MAX_MESSAGE_BYTES),
                    timeout,
                )
        except RingfourError:
            raise
        except Exception as exc:
            raise ConnectionFailed(f"could not connect to {uri}: {exc}") from exc
        conn._receiver = asyncio.ensure_future(conn._receive_loop())
        return conn

    # -- public API -----------------------------------------------------------

    async def call(
        self,
        method: str,
        params: dict[str, Any] | None = None,
        timeout: float | None = None,
        *, command_id: str | None = None,
    ) -> Any:
        """Send one request and await its result.

        Raises the mapped ringfour exception if the agent returns an error,
        TimeoutError if no reply arrives in time, ConnectionLost if the
        link drops.
        """
        if self._closed:
            raise ConnectionLost("connection is closed", call=method)
        mutating = self.command_contract is not None and method in self.command_contract['methods']
        if command_id is not None and not mutating:
            from ..errors import CapabilityNotSupported
            raise CapabilityNotSupported('explicit command IDs require an advertised mutating method')
        if mutating and command_id is None:
            command_id = self.new_command_id()
        request_id = next(self._ids)
        future: asyncio.Future = asyncio.get_running_loop().create_future()
        self._pending[request_id] = future
        try:
            message = wire.request(request_id, method, params)
            if command_id is not None:
                message = json.dumps({**json.loads(message), 'command_id': command_id})
            await self._ws.send(message)
            result = await asyncio.wait_for(future, timeout or self._timeout)
            if method == 'hello':
                self.command_contract = result.get('features', {}).get('commands')
                self._command_clock = time.monotonic()
            return result
        except SubmissionUnknown:
            raise
        except (asyncio.TimeoutError, ConnectionLost, websockets.ConnectionClosed) as exc:
            if command_id is not None:
                raise SubmissionUnknown('submission reply unavailable; reconcile the original command ID',
                    call=method, request_id=request_id, details={'command_id': command_id, 'retry': 'reconcile-first'}) from exc
            if isinstance(exc, asyncio.TimeoutError):
                raise TimeoutError(f"no reply within {timeout or self._timeout}s", call=method) from None
            raise
        finally:
            self._pending.pop(request_id, None)

    def new_command_id(self, *, valid_for=None):
        """Expiry is part of the ID, so eviction can never refresh its validity."""
        from ..errors import CapabilityNotSupported
        if self.command_contract is None:
            raise CapabilityNotSupported('agent does not advertise submission reconciliation')
        maximum = self.command_contract['validity_seconds']
        duration = maximum if valid_for is None else valid_for
        if type(duration) not in (float, int) or not math.isfinite(duration) or not 0 < duration <= maximum:
            raise ValueError('valid_for must be positive, finite and within the advertised validity window')
        now = self.command_contract['server_time'] + time.monotonic()-self._command_clock
        return f"{self.command_contract['epoch']}.{int((now+duration)*1000)}.{uuid.uuid4().hex}"

    def take_payload(self, name):
        """Consume bytes already ordered before their response/stream descriptor."""
        from ..errors import FrameExpired
        try:
            payload_id = int(name)
        except (ValueError, TypeError):
            raise RingfourError("invalid network payload ID") from None
        payload = self._payloads.pop(payload_id, None)
        if payload is None or not payload.future.done() or payload.future.cancelled():
            raise FrameExpired("network payload is unavailable or was evicted under backpressure")
        return payload.kind, payload.meta, payload.future.result()

    def start_lease(self, lease):
        """Renew authority only. Never resend a motion command or intent."""
        if lease is None:
            return
        if self._lease_renewer is not None:
            raise RuntimeError("connection lease is already running")
        self._lease_renewer = asyncio.create_task(self._renew_lease(lease))

    async def _renew_lease(self, lease):
        try:
            while not self._closed:
                await asyncio.sleep(lease["renew_interval"])
                await self.call("lease.renew", {"lease_id": lease["id"], "epoch": lease["epoch"]},
                                timeout=lease["lifetime"] / 2)
        except asyncio.CancelledError:
            raise
        except Exception:
            # Uncertain authority is never renewed by retrying motion.
            await self.close()

    async def get_payload(
        self, payload_id: int, timeout: float | None = None
    ) -> tuple[str, dict[str, Any], bytes]:
        """Await an announced binary payload; returns (kind, meta, data).

        Raises TimeoutError if the binary frame does not arrive in time,
        ConnectionLost if the link dropped first, RingfourError if no
        payload.header with this id was seen.
        """
        payload = self._payloads.get(payload_id)
        if payload is None:
            raise RingfourError(f"unknown payload id {payload_id}")
        try:
            data = await asyncio.wait_for(payload.future, timeout or self._timeout)
        except asyncio.TimeoutError:
            raise TimeoutError(f"payload {payload_id} did not arrive") from None
        finally:
            self._payloads.pop(payload_id, None)
        return payload.kind, payload.meta, data

    def subscribe(self, method: str) -> asyncio.Queue:
        """Get a queue of params dicts for a notification method
        (e.g. "task.update"). When the connection dies, the CLOSED sentinel
        is pushed. Pair with unsubscribe()."""
        queue: asyncio.Queue = asyncio.Queue()
        self._subscribers.setdefault(method, []).append(queue)
        return queue

    def unsubscribe(self, method: str, queue: asyncio.Queue) -> None:
        queues = self._subscribers.get(method, [])
        if queue in queues:
            queues.remove(queue)

    async def close(self) -> None:
        """Close the connection. Idempotent. Never raises."""
        if self._closed:
            return
        self._closed = True
        if self._lease_renewer is not None and self._lease_renewer is not asyncio.current_task():
            self._lease_renewer.cancel()
        try:
            if self._ws is not None:
                await self._ws.close()
        except Exception:
            pass  # tearing down anyway
        if self._receiver is not None:
            self._receiver.cancel()

    @property
    def uri(self) -> str:
        """The URI this connection was opened with."""
        return self._uri

    @property
    def closed(self) -> bool:
        return self._closed

    # -- receive path ----------------------------------------------------------

    async def _receive_loop(self) -> None:
        try:
            async for message in self._ws:
                if isinstance(message, bytes):
                    self._handle_binary(message)
                else:
                    self._handle_text(message)
        except websockets.ConnectionClosed:
            pass
        except asyncio.CancelledError:
            raise
        finally:
            self._closed = True
            self._fail_pending(ConnectionLost(f"connection to {self._uri} lost"))

    def _handle_text(self, text: str) -> None:
        msg = wire.parse_message(text)
        if "method" in msg:  # notification from the agent
            self._handle_notification(msg["method"], msg.get("params") or {})
            return
        future = self._pending.get(msg.get("id"))
        if future is None or future.done():
            return  # reply to a call we gave up on
        if "error" in msg:
            err = msg["error"]
            future.set_exception(
                error_from_wire(err.get("message", "agent error"), err.get("data"))
            )
        else:
            future.set_result(msg.get("result"))

    def _handle_notification(self, method: str, params: dict[str, Any]) -> None:
        if method == wire.PAYLOAD_HEADER_METHOD:
            if len(self._payloads) >= _MAX_PENDING_PAYLOADS:
                oldest = self._payloads.pop(next(iter(self._payloads)))
                oldest.future.cancel()
            self._payloads[params["payload_id"]] = _Payload(
                kind=params.get("kind", ""), meta=params.get("meta") or {}
            )
            return
        for queue in self._subscribers.get(method, []):
            if queue.qsize() >= _MAX_QUEUE_DEPTH:
                queue.get_nowait()  # drop oldest; consumer is not keeping up
            queue.put_nowait(params)

    def _handle_binary(self, frame: bytes) -> None:
        payload_id, data = wire.decode_payload_frame(frame)
        payload = self._payloads.get(payload_id)
        if payload is not None and not payload.future.done():
            payload.future.set_result(data)
            budget = 192 * 1024 * 1024
            def retained_bytes():
                return sum(len(p.future.result()) for p in self._payloads.values()
                           if p.future.done() and not p.future.cancelled() and p.future.exception() is None)
            while self._payloads and retained_bytes() > budget:
                oldest = self._payloads.pop(next(iter(self._payloads)))
                oldest.future.cancel()

    def _fail_pending(self, exc: RingfourError) -> None:
        for future in self._pending.values():
            if not future.done():
                future.set_exception(exc)
        self._pending.clear()
        for payload in self._payloads.values():
            if not payload.future.done():
                payload.future.set_exception(exc)
        self._payloads.clear()
        for queues in self._subscribers.values():
            for queue in queues:
                queue.put_nowait(CLOSED)
