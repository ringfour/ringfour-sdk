"""Agent-enforced scoped guard; independent of observational watch streams."""
import asyncio
from ..models.measurements import SignalCondition,GuardStatus
from ..lifecycle import exit_scope

class Guard:
    def __init__(self, robot, condition):
        if not isinstance(condition,SignalCondition):raise TypeError('guard requires a discovered signal condition')
        if condition.robot_id!=robot.name:raise ValueError('condition belongs to another robot')
        self._robot,self.condition=robot,condition
        self._id=None;self._close_task=None
    @property
    def id(self):return self._id
    async def open(self):
        if self._id is not None:raise RuntimeError('guard already opened')
        result=await self._robot.connection.call('guard.open',{'condition':self.condition.model_dump(mode='json')})
        status=GuardStatus.model_validate(result);self._id=status.guard_id
        return self
    async def get_status(self):
        if self._id is None:raise RuntimeError('guard is not open')
        return GuardStatus.model_validate(await self._robot.connection.call('guard.status',{'guard_id':self._id}))
    async def reset(self):
        if self._id is None:raise RuntimeError('guard is not open')
        return GuardStatus.model_validate(await self._robot.connection.call('guard.reset',{'guard_id':self._id}))
    async def close(self):
        if self._id is None:return
        if self._close_task is None:
            self._close_task=asyncio.create_task(self._robot.connection.call('guard.remove',{'guard_id':self._id},timeout=60.))
        async def join():return await asyncio.shield(self._close_task)
        await exit_scope(join,(),timeout=60.)
    async def __aenter__(self):return await self.open()
    async def __aexit__(self,*exc):await exit_scope(self.close,exc,timeout=60.)
