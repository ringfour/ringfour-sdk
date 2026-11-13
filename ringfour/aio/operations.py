"""Accepted work has an explicit lifetime; update iteration only observes it."""
from __future__ import annotations
import asyncio,time
from typing import Generic,TypeVar
from ..models.execution import OperationSnapshot,OperationState,CancelReceipt,StopReport
from ..errors import OperationCancelled,WaitTimeout,OperationTimeout,TimeoutError,CompletionUnverified
from ..lifecycle import exit_scope

TResult=TypeVar('TResult');TProgress=TypeVar('TProgress')


class Operation(Generic[TResult,TProgress]):
    """Already accepted finite work, with typed progress/results and bounded close.

    Breaking updates never stops work. Context exit or close cancels unfinished
    work and joins cleanup. A result wait timeout leaves the operation running.
    """
    def __init__(self,task,result_factory,progress_factory,*,resource,cleanup_timeout=60.):
        self._task=task;self._result_factory=result_factory;self._progress_factory=progress_factory
        self._resource=resource;self._cleanup_timeout=cleanup_timeout
        self._cancelling=False;self._cleanup=None

    @property
    def id(self):return self._task.id

    @property
    def snapshot(self):
        state='accepted' if self._task.state=='pending' else self._task.state
        if self._cancelling and not self._task.done:state='cancelling'
        return OperationSnapshot(operation_id=self.id,state=OperationState(state),sequence=self._task.sequence,
            observed_age=self._task.report_age+max(0.,time.monotonic()-self._task.received_at),progress=self._progress_factory(self._task))

    async def get_status(self):
        await self._task.get_status()
        return self.snapshot

    async def updates(self):
        queue=asyncio.Queue(maxsize=64)
        self._task._listeners.add(queue)
        try:
            yield self.snapshot
            while not self._task.done:
                update=await queue.get()
                if isinstance(update,Exception):raise update
                yield self.snapshot
        finally:self._task._listeners.discard(queue)

    async def result(self,timeout=None):
        try:result=await self._task.wait(timeout)
        except TimeoutError as exc:
            if self._task.done:raise
            raise WaitTimeout(str(exc),details={'operation_id':self.id,'timeout':timeout}) from exc
        if self._task.state=='cancelled':
            raise OperationCancelled(f'operation {self.id} was cancelled',details={'operation_id':self.id,'cleanup':self._task.cleanup})
        return self._result_factory(self.id,result or {})

    async def cancel(self):
        if self._task.done:return CancelReceipt(operation_id=self.id,accepted=False,already_terminal=True)
        response=await self._task._conn.call('task.cancel',{'task_id':self.id})
        receipt=CancelReceipt(operation_id=self.id,accepted=response['accepted'],already_terminal=response['already_terminal'])
        if receipt.accepted:self._cancelling=True
        return receipt

    async def _close(self):
        if not self._task.done:await self.cancel()
        try:await self.result(self._cleanup_timeout)
        except OperationCancelled:
            evidence=self._task.cleanup
            if not evidence or not evidence.get('verified'):
                raise CompletionUnverified('cancelled operation lacks its stopping evidence',details={'operation_id':self.id})
            return StopReport(resource=self._resource,verified=evidence['verified'],criterion=evidence['criterion'],
                physical_stop=evidence['physical_stop'],elapsed=evidence.get('elapsed'),dwell=evidence.get('dwell'))
        return None

    async def close(self):
        if self._cleanup is None:self._cleanup=asyncio.create_task(self._close())
        async def join():return await asyncio.shield(self._cleanup)
        await exit_scope(join,(),timeout=self._cleanup_timeout)
        return self._cleanup.result()

    async def __aenter__(self):return self
    async def __aexit__(self,*exc):await exit_scope(self.close,exc,timeout=self._cleanup_timeout)


async def complete(operation,timeout=None):
    """Blocking convenience timeout/cancellation stops work before returning."""
    try:return await operation.result(timeout)
    except WaitTimeout as original:
        report=None;failure=None
        try:report=await operation.close()
        except Exception as exc:failure=exc
        details={'operation_id':operation.id,'timeout':timeout,
                 'cleanup':report.model_dump(mode='json') if report else None,
                 'cleanup_error':str(failure) if failure else None}
        error=OperationTimeout(f'operation {operation.id} exceeded {timeout}s; cleanup attempted',details=details)
        if failure:error.cleanup_error=failure
        raise error from original
    except asyncio.CancelledError as original:
        await exit_scope(operation.close,(type(original),original,original.__traceback__),timeout=operation._cleanup_timeout)
        raise
