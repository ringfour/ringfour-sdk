"""Explicit reconciliation. No method automatically replays a submission."""
from ..models.commands import CommandRecord, OperationRecord


class Commands:
    def __init__(self, connection):
        self._conn = connection

    def new(self, *, valid_for=None):
        """Generate an epoch-bound ID using the agent's advertised clock/window."""
        return self._conn.new_command_id(valid_for=valid_for)

    async def get(self, command_id):
        """Query acceptance/error after reply loss; unknown never means unexecuted."""
        return CommandRecord.model_validate(await self._conn.call('commands.get', {'command_id': command_id}))

    async def retry(self, command_id, method, params=None, *, timeout=None):
        """Repeat an identical request under its original ID after reconciliation.

        A current, unexpired duplicate returns its original reply. Conflicting,
        expired or old-epoch IDs cannot dispatch. Never invent a replacement ID
        to work around an uncertain outcome.
        """
        return await self._conn.call(method, params, timeout=timeout, command_id=command_id)


class Operations:
    def __init__(self, connection):
        self._conn = connection

    async def get(self, operation_id):
        """Query retained evidence, including after authenticated reconnection."""
        return OperationRecord.model_validate(await self._conn.call('operations.get', {'operation_id': operation_id}))
