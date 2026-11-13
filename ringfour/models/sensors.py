"""Sensor reading types: pure-JSON wire models.

Each maps 1:1 to a ROS message the agent subscribes to per the profile.
Array-bearing sensor types (Frame, DepthFrame, PointCloud, LaserScan,
OccupancyGrid) live in ringfour.data instead — their bulk travels via
PayloadRef, not JSON.
"""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict

from .spatial import Quaternion, Vec3


class RangeReading(BaseModel):
    """A single-beam distance measurement (ultrasonic / IR / ToF point
    sensor). ``range`` is inf when there is no return.

    A discovered range signal can form a persistent guard condition. The
    deployment must review its threshold, freshness and covered motion.
    """

    model_config = ConfigDict(frozen=True)

    range: float            # meters; inf = no return
    min_range: float
    max_range: float
    field_of_view: float    # radians, cone width
    radiation_type: Literal["ultrasonic", "infrared"]
    stamp: float
    frame: str


class ImuReading(BaseModel):
    """IMU output. ``orientation`` is None when the IMU provides no
    orientation estimate. Covariances are 9-element row-major; ROS marks
    "unknown" with -1 in element [0], preserved as None here."""

    model_config = ConfigDict(frozen=True)

    orientation: Quaternion | None
    angular_velocity: Vec3          # rad/s
    linear_acceleration: Vec3       # m/s^2, includes gravity
    orientation_covariance: tuple[float, ...] | None = None
    angular_velocity_covariance: tuple[float, ...] | None = None
    linear_acceleration_covariance: tuple[float, ...] | None = None
    stamp: float
    frame: str


class MagReading(BaseModel):
    """Magnetometer measurement (Tesla)."""

    model_config = ConfigDict(frozen=True)

    magnetic_field: Vec3
    covariance: tuple[float, ...] | None = None
    stamp: float
    frame: str


class ForceTorque(BaseModel):
    """6-axis force/torque reading (wrist sensors and the like).

    A discovered wrench signal can form a persistent guard condition;
    threshold and fresh source/receive evidence belong to the deployment.
    """

    model_config = ConfigDict(frozen=True)

    force: Vec3     # Newtons
    torque: Vec3    # Newton-meters
    stamp: float
    frame: str


class Temperature(BaseModel):
    """Environmental temperature, Celsius."""

    model_config = ConfigDict(frozen=True)

    temperature: float
    variance: float = 0.0  # 0 = unknown
    stamp: float
    frame: str


class FluidPressure(BaseModel):
    """Barometric/fluid pressure, Pascals."""

    model_config = ConfigDict(frozen=True)

    pressure: float
    variance: float = 0.0
    stamp: float
    frame: str


class Illuminance(BaseModel):
    """Ambient light level, Lux."""

    model_config = ConfigDict(frozen=True)

    illuminance: float
    variance: float = 0.0
    stamp: float
    frame: str


class BatteryState(BaseModel):
    """Battery status (spec section 19).

    ``level`` is 0-100; None until data arrives or when the driver sends
    NaN and the profile provides no voltage curve.
    """

    model_config = ConfigDict(frozen=True)

    level: float | None
    voltage: float | None = None
    current: float | None = None       # Amps; negative = discharging
    temperature: float | None = None   # Celsius
    is_charging: bool | None = None
    power_supply_status: Literal["charging", "discharging", "not_charging",
                                 "full", "unknown"] = "unknown"
    stamp: float | None = None


class SubsystemStatus(BaseModel):
    """One subsystem's health line in a DiagnosticsReport."""

    model_config = ConfigDict(frozen=True)

    name: str
    level: Literal["ok", "warn", "error", "stale"]
    message: str = ""
    values: dict[str, str] = {}


class DiagnosticsReport(BaseModel):
    """System health snapshot from /diagnostics."""

    model_config = ConfigDict(frozen=True)

    subsystems: tuple[SubsystemStatus, ...]
    stamp: float

    def check(self) -> bool:
        """True when no subsystem reports error or stale."""
        return all(s.level in ("ok", "warn") for s in self.subsystems)


class PowerLaneState(BaseModel):
    """Latest reading of one switchable power lane: on/off, voltage,
    current. ``on`` reflects the MOSFET state the firmware reports
    (may lag briefly after a command while the switch settles)."""

    model_config = ConfigDict(frozen=True)

    name: str
    on: bool
    voltage: float       # Volts
    current: float       # Amps
    stamp: float
