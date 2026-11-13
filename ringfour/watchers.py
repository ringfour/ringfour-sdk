"""Named observational watch helpers; persistent motion conditions use guard()."""

from __future__ import annotations

from typing import Any, Mapping

from .errors import CapabilityNotSupported
from .vendor import SensorGroup

__all__ = ["SensorGroups", "Watches"]


class Watches:
    """`robot.watches` — one method per quantity every robot may have."""

    def __init__(self, robot: Any) -> None:
        self._robot = robot

    # -- power ---------------------------------------------------------------

    def battery_level(self, **limits: Any) -> Any:
        """Percent remaining. The classic `stop=True` guard."""
        return self._at("battery.level", **limits)

    def battery_voltage(self, **limits: Any) -> Any:
        return self._at("battery.voltage", **limits)

    def battery_current(self, **limits: Any) -> Any:
        return self._at("battery.current", **limits)

    # -- where the robot is and how fast ------------------------------------

    def pose_x(self, **limits: Any) -> Any:
        return self._at("pose.x", **limits)

    def pose_y(self, **limits: Any) -> Any:
        return self._at("pose.y", **limits)

    def distance_traveled(self, **limits: Any) -> Any:
        return self._at("distance_traveled", **limits)

    def forward_speed(self, **limits: Any) -> Any:
        """Forward velocity, m/s."""
        return self._at("velocity.linear.x", **limits)

    def lateral_speed(self, **limits: Any) -> Any:
        return self._at("velocity.linear.y", **limits)

    def turn_rate(self, **limits: Any) -> Any:
        """Yaw rate, rad/s."""
        return self._at("velocity.angular.z", **limits)

    # -- sensors that exist per instance ------------------------------------

    def range(self, sensor: str, **limits: Any) -> Any:
        """One single-beam ranger, by the name the profile gave it."""
        # The property raises when the capability is absent, so it is
        # evaluated lazily — passing it as an argument would raise before
        # `_known` could turn it into a message about THIS name.
        self._known(sensor, lambda: self._robot.range_sensors,
                    "range sensor")
        return self._at(f"range.{sensor}.range", **limits)

    def imu_acceleration(self, axis: str, **limits: Any) -> Any:
        return self._at(
            f"imu.linear_acceleration.{self._axis(axis)}", **limits)

    def imu_angular_rate(self, axis: str, **limits: Any) -> Any:
        return self._at(f"imu.angular_velocity.{self._axis(axis)}", **limits)

    # -- environment ---------------------------------------------------------

    def temperature(self, **limits: Any) -> Any:
        return self._at("environment.temperature", **limits)

    def pressure(self, **limits: Any) -> Any:
        return self._at("environment.pressure", **limits)

    def illuminance(self, **limits: Any) -> Any:
        return self._at("environment.illuminance", **limits)

    # -- autonomy ------------------------------------------------------------

    def localization_quality(self, **limits: Any) -> Any:
        """Drops when SLAM starts relocalising. A good `stop=True`."""
        return self._at("slam.localization_quality", **limits)

    # -- plumbing ------------------------------------------------------------

    def _at(self, path: str, **limits: Any) -> Any:
        return self._robot._watch_path(path, **limits)

    @staticmethod
    def _axis(axis: str) -> str:
        if axis not in ("x", "y", "z"):
            raise ValueError(f"axis must be x, y or z; got {axis!r}")
        return axis

    @staticmethod
    def _known(name: str, available: Any, what: str) -> None:
        try:
            names = tuple(available())
        except CapabilityNotSupported:
            raise ValueError(
                f"this robot has no {what}s at all, so there is no "
                f"{name!r} to watch") from None
        if name not in names:
            raise ValueError(
                f"no {what} called {name!r} on this robot; it has "
                f"{list(names)}")

    def __repr__(self) -> str:
        return "<robot.watches: typed watch handles>"


class SensorGroups:
    """`robot.sensors` — the vendor signal groups, by name."""

    def __init__(self, robot: Any,
                 declared: Mapping[str, Mapping[str, Mapping[str, Any]]]
                 ) -> None:
        self._robot = robot
        self._declared = dict(declared)

    @property
    def groups(self) -> tuple[str, ...]:
        return tuple(self._declared)

    def __getattr__(self, name: str) -> SensorGroup:
        if name.startswith("_"):
            raise AttributeError(name)
        try:
            signals = self._declared[name]
        except KeyError:
            raise CapabilityNotSupported(
                f"this robot has no sensor group {name!r}; it has "
                f"{sorted(self._declared)}") from None
        return SensorGroup(self._robot, name, signals)

    def __getitem__(self, name: str) -> SensorGroup:
        return getattr(self, name)

    def __iter__(self):
        return iter(self._declared)

    def __dir__(self) -> list[str]:
        return sorted({*super().__dir__(), *self._declared})

    def __repr__(self) -> str:
        return f"<robot.sensors: {', '.join(self._declared)}>"
