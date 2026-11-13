"""Blocking reconciliation APIs; sync and async use the same wire contract."""
class Commands:
    def __init__(self, runner, commands):
        self._runner, self._commands = runner, commands

    def new(self, *, valid_for=None):
        """Generate a command ID within the advertised validity window."""
        return self._commands.new(valid_for=valid_for)

    def get(self, command_id):
        """Query retained acceptance/error without replaying the request."""
        return self._runner.run(self._commands.get(command_id))

    def retry(self, command_id, method, params=None, *, timeout=None):
        """Explicitly retry the original ID/request; never generate a replacement."""
        return self._runner.run(self._commands.retry(command_id, method, params, timeout=timeout))


class Operations:
    def __init__(self, runner, operations):
        self._runner, self._operations = runner, operations

    def get(self, operation_id):
        """Query retained terminal or explicitly unknown restart evidence."""
        return self._runner.run(self._operations.get(operation_id))
