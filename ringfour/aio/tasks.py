"""Async task handles (spec section 5: the Task type).

The Robot owns one TaskRouter that consumes every ``task.update``
notification and routes it to the right Task by id, so ``task.done`` /
``task.result`` are always current even when nobody is awaiting wait().
"""

from __future__ import annotations

import asyncio
import time
from typing import Any

from ..errors import ConnectionLost, RingfourError, error_from_wire
from ..models import TaskUpdate
from .transport import CLOSED, Connection


class Task:
    """Handle for one long-running call on the agent.

    ``wait()`` blocks until terminal: returns the result on success, raises
    the mapped ringfour exception on failure, returns None if the task was
    cancelled. ``cancel()`` asks the agent to cancel; updates keep flowing
    until the terminal state arrives.

    This is private wire bookkeeping. Public Operation.result() translates
    cancellation to OperationCancelled and preserves cleanup evidence.
    """

    def __init__(self, conn: Connection, task_id: str, *, query_supported=False) -> None:
        self._conn = conn
        self.sequence=0
        self.report_age=0.0
        self.received_at=time.monotonic()
        self.cleanup=None
        self._listeners=set()
        self.id = task_id
        self.state: str = "pending"
        self.progress: float | None = None
        # Latest task-kind-specific feedback (navigation: pose,
        # distance_remaining, ...). None until the first report.
        self.data: dict[str, Any] | None = None
        self.result: dict[str, Any] | None = None
        self._error: RingfourError | None = None
        self._terminal = asyncio.Event()
        self._query_supported = query_supported

    async def get_status(self):
        """Query authoritative state; limited to the originating connection."""
        from ..errors import CapabilityNotSupported
        if not self._query_supported:
            raise CapabilityNotSupported("agent does not support authoritative task queries")
        update = TaskUpdate.model_validate(await self._conn.call("task.status", {"task_id": self.id}))
        self._apply(update)
        return update

    @property
    def done(self) -> bool:
        return self.state in ("succeeded", "failed", "cancelled")

    async def wait(self, timeout: float | None = None) -> dict[str, Any] | None:
        """Block until the task ends. Raises ringfour.TimeoutError if it
        does not end within ``timeout`` (the task keeps running)."""
        try:
            if not self._query_supported:
                await asyncio.wait_for(self._terminal.wait(), timeout)
            else:
                deadline = None if timeout is None else time.monotonic()+timeout
                while not self.done and self._error is None:
                    remaining = None if deadline is None else deadline-time.monotonic()
                    if remaining is not None and remaining <= 0:
                        raise asyncio.TimeoutError()
                    try:
                        await asyncio.wait_for(self._terminal.wait(), min(1.0, remaining) if remaining is not None else 1.0)
                    except asyncio.TimeoutError:
                        # A dropped terminal notification cannot leave wait hanging.
                        remaining = None if deadline is None else deadline-time.monotonic()
                        if remaining is not None and remaining <= 0:
                            raise
                        await asyncio.wait_for(self.get_status(), remaining)
        except asyncio.TimeoutError:
            from ..errors import TimeoutError as RcTimeout
            raise RcTimeout(
                f"task {self.id} not done within {timeout}s"
            ) from None
        if self._error is not None:
            raise self._error
        return self.result

    async def cancel(self) -> None:
        """Ask the agent to cancel this task. No-op if already terminal."""
        if not self.done:
            await self._conn.call("task.cancel", {"task_id": self.id})

    # called by TaskRouter, loop thread
    def _apply(self, update: TaskUpdate) -> None:
        if self.done:
            return  # terminal outcomes are immutable, including late progress
        if isinstance(self._error, ConnectionLost):
            self._error = None
            self._terminal.clear()
        if update.sequence and update.sequence < self.sequence:
            return  # a delayed notification must not replace a newer report
        self.sequence=update.sequence
        self.report_age=update.report_age
        self.received_at=time.monotonic()
        self.cleanup=update.cleanup
        self.state = update.state
        if update.progress is not None:
            self.progress = update.progress
        if update.data is not None:
            self.data = update.data
        if update.state == "succeeded":
            self.result = update.result
            self._terminal.set()
        elif update.state == "failed":
            error = update.error
            if error is not None:
                self._error = error_from_wire(
                    error.message, {"type": error.type, **error.data}
                )
            else:
                self._error = RingfourError(f"task {self.id} failed")
            self._terminal.set()
        elif update.state == "cancelled":
            self._terminal.set()

        self._notify(update)

    def _notify(self,item):
        for queue in tuple(self._listeners):
            if queue.full():queue.get_nowait()
            queue.put_nowait(item)

    def _fail(self, exc: RingfourError) -> None:
        if not self.done:
            self._error = exc
            self._terminal.set()
            self._notify(exc)


class TaskRouter:
    """Routes task.update notifications to Task objects by id."""

    def __init__(self, conn: Connection, *, query_supported=False) -> None:
        self._conn = conn
        self._query_supported = query_supported
        self._tasks: dict[str, Task] = {}
        # Updates that arrived before their Task was registered (a fast task
        # can emit, even finish, before the submitting call's reply is
        # processed). Applied on registration, in arrival order.
        self._early: dict[str, list[TaskUpdate]] = {}
        self._queue = conn.subscribe("task.update")
        self._runner = asyncio.ensure_future(self._route())

    async def start_task(self, method: str, params: dict[str, Any]) -> Task:
        """Call a task-returning method and hand back its registered Task."""
        result = await self._conn.call(method, params)
        task_id = result["task_id"]
        task = Task(self._conn, task_id, query_supported=self._query_supported)
        for update in self._early.pop(task_id, []):
            task._apply(update)
        if not task.done:
            self._tasks[task_id] = task
        return task

    async def _route(self) -> None:
        while True:
            item = await self._queue.get()
            if item is CLOSED:
                for task in self._tasks.values():
                    task._fail(ConnectionLost("connection lost"))
                return
            update = TaskUpdate.model_validate(item)
            task = self._tasks.get(update.task_id)
            if task is None:
                if len(self._early) > 256:  # bound leak if updates go stray
                    self._early.pop(next(iter(self._early)))
                buffered = self._early.setdefault(update.task_id, [])
                if buffered and buffered[-1].state in ("succeeded", "failed", "cancelled"):
                    continue
                buffered.append(update)
                if len(buffered) > 32:
                    del buffered[0]
                continue
            task._apply(update)
            if task.done:
                del self._tasks[update.task_id]

    def close(self) -> None:
        self._runner.cancel()
