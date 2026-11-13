"""Synchronous facade for the same persistent agent guard contract."""
class Guard:
    def __init__(self, runner, guard):self._runner,self._guard=runner,guard
    @property
    def id(self):return self._guard.id
    @property
    def condition(self):return self._guard.condition
    def open(self):self._runner.run(self._guard.open());return self
    def get_status(self):return self._runner.run(self._guard.get_status())
    def reset(self):return self._runner.run(self._guard.reset())
    def close(self):return self._runner.run(self._guard.close())
    def __enter__(self):return self.open()
    def __exit__(self,*exc):return self._runner.run(self._guard.__aexit__(*exc))
