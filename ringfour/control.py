"""Scoped control of model-bound trajectories; updates only observe accepted work."""

from __future__ import annotations

from typing import Any, Iterator

from .aio import control as aio_control
from .runner import Runner

__all__ = ["ControlSession"]




class ControlSession:
    """An open control session. Use via ``group.control(...)``."""

    def __init__(self, runner: Runner, aio: aio_control.ControlSession
                 ) -> None:
        self._runner = runner
        self._aio = aio

    @property
    def group(self) -> str:
        return self._aio.group


    @property
    def role(self) -> Any:
        return self._aio.role




    @property
    def integration(self):return self._aio.integration

    def start_trajectory(self,trajectory):
        """Submit now and return the same explicit operation lifetime as aio."""
        from .operations import Operation
        return Operation(self._runner,self._runner.run(self._aio.start_trajectory(trajectory)))

    def run(self,trajectory,*,timeout=None):
        """Return complete model-bound arrival evidence after execution."""
        return self._runner.run(self._aio.run(trajectory,timeout=timeout))

    def hold(self) -> dict[str, float]:
        """Command the current position and stay in the session."""
        return self._runner.run(self._aio.hold())

    def stop(self) -> int:
        """Halt any running trajectory. The session stays open."""
        return self._runner.run(self._aio.stop())

    def close(self) -> None:
        """End the session and restore the controller. Idempotent."""
        self._runner.run(self._aio.close())

    def __enter__(self) -> "ControlSession":
        return self

    def __exit__(self, *exc_info: Any) -> None:
        self._runner.run(self._aio.__aexit__(*exc_info))


class SessionOpener:
    """What ``group.control(...)`` returns on the sync side.

    A context manager, and callable-free: `with group.control(role) as c`.
    Opening eagerly in `control()` would mean a session leaked whenever
    someone wrote the call without a `with`, so the switch happens on
    __enter__.
    """

    def __init__(self, runner: Runner, opener: Any) -> None:
        self._runner = runner
        self._opener = opener
        self._session: ControlSession | None = None

    def open(self) -> ControlSession:
        """Open without a `with` block. You own the close()."""
        if self._session is None:
            self._session = ControlSession(
                self._runner, self._runner.run(self._opener.open()))
        return self._session

    def __enter__(self) -> ControlSession:
        return self.open()

    def __exit__(self, *exc_info: Any) -> None:
        if self._session is not None:
            self._session.__exit__(*exc_info)
