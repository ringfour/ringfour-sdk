"""`robot.sensors` and `robot.actions` — whatever the vendor put on a topic.

The capability registry covers what most robots have. This covers what one
robot has: a servo's temperature, a bus voltage, a front LED, an outrigger.
Both halves are named by the profile, so they are reached under a fixed
namespace rather than on `robot` itself:

    robot.sensors.base_motor.get_temperature()      # 41.0
    robot.actions.set_front_led(r=1.0, a=1.0)

**Two reasons for the namespace.** Profile-controlled names cannot shadow
`robot.get_pose()`, `robot.stop` or `robot.arm` — a typo gives a clear error
instead of a confusing one. And a mission that touches `robot.sensors` is
visibly robot-specific at the callsite, the same property
`group.control(raw=)` has and for the same reason: portability you cannot
see is portability you will lose.

Watching is where sensors earn their keep. Every signal is a typed watch
handle, so an overheating servo can halt the robot through the shipped
Phase 4 stop path:

    with robot.sensors.base_motor.watch_temperature(above=60, stop=True):
        ...
"""

from __future__ import annotations

from typing import Any, Awaitable, Mapping
from .models import DispatchReceipt

__all__ = ["ActionGroup", "SensorGroup"]


class SensorGroup:
    """One named group of vendor signals — `robot.sensors.<name>`.

    Signals appear as attributes carrying the latest value, and each also
    gets a `watch_<signal>` factory. Both are built from the handshake, so
    an unknown name is an `AttributeError` naming what does exist rather
    than a call that silently never fires.
    """

    def __init__(self, robot: Any, group: str,
                 signals: Mapping[str, Mapping[str, Any]]) -> None:
        self._robot = robot
        self._group = group
        self._signals = dict(signals)

    @property
    def signals(self) -> tuple[str, ...]:
        return tuple(self._signals)

    def units(self, signal: str) -> str | None:
        """What the number is in. Reported, never converted — `Int32`
        celsius beside `Float32` milliamps is the pair that misleads."""
        return self._entry(signal).get("units")

    def get_reading(self, signal: str) -> dict[str, Any]:
        """{value, stamp, units}. `value` is None before the first
        message, which is different from the signal not existing."""
        self._entry(signal)
        return self._robot._sensor_reading(self._group, signal)

    def _entry(self, signal: str) -> Mapping[str, Any]:
        try:
            return self._signals[signal]
        except KeyError:
            raise AttributeError(
                f"sensors.{self._group} has no signal {signal!r}; it has "
                f"{sorted(self._signals)}") from None

    def __getattr__(self, name: str) -> Any:
        if name.startswith("watch_"):
            signal = name[len("watch_"):]
            self._entry(signal)
            return _WatchFactory(self._robot, f"sensors.{self._group}."
                                              f"{signal}")
        if name.startswith("_"):
            raise AttributeError(name)
        if name.startswith('get_'):
            signal=name[4:]
            self._entry(signal)
            return lambda:self._robot._sensor_value(self._group,signal)
        raise AttributeError(f'use get_<signal>() to read sensors.{self._group}')

    def __dir__(self) -> list[str]:
        return sorted({*super().__dir__(), *self._signals,
                       *(f"watch_{s}" for s in self._signals)})

    def __repr__(self) -> str:
        return f"<SensorGroup {self._group}: {', '.join(self._signals)}>"


class _WatchFactory:
    """`sensors.x.watch_y(...)` — a typed handle bound to one quantity."""

    def __init__(self, robot: Any, path: str) -> None:
        self._robot = robot
        self._path = path

    def __call__(self, *, above: float | None = None,
                 below: float | None = None, stop: bool = False,
                 debounce: float | None = None) -> Any:
        return self._robot._watch_path(
            self._path, above=above, below=below, stop=stop,
            debounce=debounce)

    def __repr__(self) -> str:
        return f"<watch {self._path}>"


class ActionGroup:
    """`robot.actions` — the vendor verbs this robot declares.

    Each becomes a callable taking the parameters the profile named.
    Calling one publishes, and the agent audits it; an action the profile
    marked `is_motion: true` also takes the motion lock and is refused
    under e-stop, exactly like any other thing that moves the robot.
    """

    def __init__(self, robot: Any,
                 actions: Mapping[str, Mapping[str, Any]]) -> None:
        self._robot = robot
        self._actions = dict(actions)

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(self._actions)

    def describe(self, name: str) -> Mapping[str, Any]:
        """What this action takes, and whether it moves the robot."""
        return self._entry(name)

    def moves_the_robot(self, name: str) -> bool:
        return bool(self._entry(name)["is_motion"])

    def _entry(self, name: str) -> Mapping[str, Any]:
        try:
            return self._actions[name]
        except KeyError:
            raise AttributeError(
                f"this robot has no action {name!r}; it has "
                f"{sorted(self._actions)}") from None

    def __getattr__(self, name: str) -> Any:
        if name.startswith("_"):
            raise AttributeError(name)
        self._entry(name)
        return _Action(self._robot, name, self._actions[name])

    def __dir__(self) -> list[str]:
        return sorted({*super().__dir__(), *self._actions})

    def __repr__(self) -> str:
        return f"<ActionGroup: {', '.join(self._actions)}>"


class _Action:
    """One declared action, as something you can call."""

    def __init__(self, robot: Any, name: str,
                 spec: Mapping[str, Any]) -> None:
        self._robot = robot
        self._name = name
        self._spec = spec

    def __call__(self, **arguments: Any) -> DispatchReceipt | Awaitable[DispatchReceipt]:
        return self._robot._invoke_action(self._name, arguments)

    @property
    def is_motion(self) -> bool:
        return bool(self._spec["is_motion"])

    @property
    def parameters(self) -> Mapping[str, Any]:
        return self._spec.get("parameters", {})

    def __repr__(self) -> str:
        args = ", ".join(self.parameters)
        motion = ", moves the robot" if self.is_motion else ""
        return f"<action {self._name}({args}){motion}>"
