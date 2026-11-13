"""Scoped control of model-bound trajectories; updates only observe accepted work."""

from __future__ import annotations

import time
import math
from numbers import Real
from typing import Any

from ..enums import Controller, Phase
from ..errors import RingfourError

__all__ = ["ControlSession"]


class ControlSession:
    """An open control session on one group. Use via ``group.control()``."""

    def __init__(self, robot: Any, group: str, controller: str,
                 role: Controller | None, loaded: list[str],
                 deactivated: list[str], session_id: str | None = None) -> None:
        self._robot = robot
        self._conn = robot.connection
        self.group = group
        # The resolved controller name, for the mission's own logs. Spec
        # 8.2 lists it as a session member precisely so a log line can say
        # which controller was driven rather than which role was asked for.
        self._controller = controller
        self.role = role
        self._loaded = tuple(loaded)
        self._deactivated = tuple(deactivated)
        self._closed = False
        self._session_id = session_id


    @property
    def integration(self):
        from types import MappingProxyType
        return MappingProxyType({'controller':self._controller,'loaded':self._loaded,'deactivated':self._deactivated})

    async def start_trajectory(self,trajectory):
        """Submit a model-bound JointTrajectory now; return typed accepted work."""
        from ..models.execution import JointTrajectory
        from ..configuration import validate_configuration
        from .execution import trajectory_operation
        if self._closed:raise RingfourError('control session is closed')
        if not isinstance(trajectory,JointTrajectory):raise TypeError('expected a JointTrajectory from model-bound configurations')
        if not self._robot._welcome.features.get('verified_operations'):
            from ..errors import CapabilityNotSupported
            raise CapabilityNotSupported('agent does not support verified operations')
        group=self._robot.groups[self.group]
        for configuration in trajectory.configurations:validate_configuration(group,configuration)
        task=await self._robot._tasks.start_task('group.start_trajectory',{'group':self.group,
            'session_id':self._session_id,'trajectory':trajectory.model_dump(mode='json')})
        return trajectory_operation(self._robot,task,trajectory)

    async def run(self,trajectory,*,timeout=None):
        """Wait for verified joint arrival; timeout/cancellation joins cleanup."""
        from .operations import complete
        return await complete(await self.start_trajectory(trajectory),timeout)

    async def hold(self) -> dict[str, float]:
        """Command the current position and stay in the session."""
        result = await self._conn.call("group.hold", {"group": self.group, "session_id": self._session_id})
        return result["positions"]

    async def stop(self) -> int:
        """Halt any running trajectory. The session stays open."""
        result = await self._conn.call("group.stop", {"group": self.group, "session_id": self._session_id})
        return int(result.get("cancelled", 0))

    async def close(self) -> None:
        """End the session and restore the controller. Idempotent."""
        if self._closed:
            return
        await self._conn.call("group.control_stop", {"group": self.group, "session_id": self._session_id})
        self._closed = True

    async def __aenter__(self) -> "ControlSession":
        return self

    async def __aexit__(self, *exc_info: Any) -> None:
        from ..lifecycle import exit_scope
        await exit_scope(self.close, exc_info)
