"""Blocking facade with the same explicit lifetime as async operations."""
from typing import Generic,TypeVar
TResult=TypeVar('TResult');TProgress=TypeVar('TProgress')
class Operation(Generic[TResult,TProgress]):
    """Accepted work. Result waits and update streams do not acquire or cancel it."""
    def __init__(self,runner,operation):self._runner,self._operation=runner,operation
    @property
    def id(self):return self._operation.id
    @property
    def snapshot(self):return self._operation.snapshot
    def get_status(self):return self._runner.run(self._operation.get_status())
    def result(self,timeout=None):return self._runner.run(self._operation.result(timeout))
    def cancel(self):return self._runner.run(self._operation.cancel())
    def close(self):return self._runner.run(self._operation.close())
    def updates(self):
        iterator=self._operation.updates()
        try:
            while True:
                try:yield self._runner.run(anext(iterator))
                except StopAsyncIteration:return
        finally:self._runner.run(iterator.aclose())
    def __enter__(self):return self
    def __exit__(self,*exc):return self._runner.run(self._operation.__aexit__(*exc))
