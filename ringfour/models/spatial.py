"""Core spatial types (spec section 5).

All types are frozen pydantic models. Distances in meters, angles in
radians. Every located type carries a ``frame: str``; math across two
different frames raises FrameMismatch rather than silently combining.

Frame aliases ("@map", "@odom", "@base") are plain strings here; the agent
resolves them against the profile's frame names. Client-side math requires
literally equal frame strings ("@map" != "map" — resolve first via tf).
"""

from __future__ import annotations

import math
from numbers import Real

from pydantic import BaseModel, ConfigDict, model_validator, field_validator

from ..errors import FrameMismatch


class Vec3(BaseModel):
    """A frameless 3-vector."""

    model_config = ConfigDict(frozen=True)

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


class Quaternion(BaseModel):
    """Orientation as a unit quaternion. Identity by default."""

    model_config = ConfigDict(frozen=True)

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0

    @field_validator('x','y','z','w',mode='before')
    @classmethod
    def finite(cls,value):
        if isinstance(value,bool) or not isinstance(value,Real) or not math.isfinite(value):raise ValueError('quaternion components must be finite numbers')
        return value

    @model_validator(mode='after')
    def unit_length(self):
        if abs(math.hypot(self.x,self.y,self.z,self.w)-1)>1e-6:raise ValueError('quaternion must have unit length')
        return self

    @classmethod
    def from_euler(cls, roll: float, pitch: float, yaw: float) -> "Quaternion":
        """Build from intrinsic roll/pitch/yaw (radians, REP-103)."""
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        return cls(
            x=sr * cp * cy - cr * sp * sy,
            y=cr * sp * cy + sr * cp * sy,
            z=cr * cp * sy - sr * sp * cy,
            w=cr * cp * cy + sr * sp * sy,
        )

    def to_euler(self) -> tuple[float, float, float]:
        """(roll, pitch, yaw) in radians. Pitch is clamped at the
        gimbal-lock poles (|pitch| = pi/2)."""
        roll = math.atan2(2 * (self.w * self.x + self.y * self.z),
                          1 - 2 * (self.x ** 2 + self.y ** 2))
        sin_pitch = max(-1.0, min(1.0, 2 * (self.w * self.y - self.z * self.x)))
        pitch = math.asin(sin_pitch)
        yaw = math.atan2(2 * (self.w * self.z + self.x * self.y),
                         1 - 2 * (self.y ** 2 + self.z ** 2))
        return roll, pitch, yaw

    def multiply(self, other: "Quaternion") -> "Quaternion":
        """Hamilton product self * other (apply ``other``'s rotation first,
        then self's)."""
        return Quaternion(
            w=self.w * other.w - self.x * other.x - self.y * other.y
              - self.z * other.z,
            x=self.w * other.x + self.x * other.w + self.y * other.z
              - self.z * other.y,
            y=self.w * other.y - self.x * other.z + self.y * other.w
              + self.z * other.x,
            z=self.w * other.z + self.x * other.y - self.y * other.x
              + self.z * other.w,
        )

    def rotate(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        """Rotate the vector (x, y, z) by this quaternion."""
        # v' = v + 2*qv x (qv x v + w*v)  (qv = quaternion vector part)
        tx = 2 * (self.y * z - self.z * y)
        ty = 2 * (self.z * x - self.x * z)
        tz = 2 * (self.x * y - self.y * x)
        return (
            x + self.w * tx + self.y * tz - self.z * ty,
            y + self.w * ty + self.z * tx - self.x * tz,
            z + self.w * tz + self.x * ty - self.y * tx,
        )

    def inverse(self) -> "Quaternion":
        """The opposite rotation (conjugate; quaternions here are unit)."""
        return Quaternion(x=-self.x, y=-self.y, z=-self.z, w=self.w)


class Point(BaseModel):
    """A located point: position + reference frame."""

    model_config = ConfigDict(frozen=True)

    x: float
    y: float
    z: float = 0.0
    frame: str


class Pose(BaseModel):
    """Position + orientation in a frame.

    Shorthand: ``Pose(x=1, y=2, yaw=1.57, frame="map")`` builds the
    quaternion for you (passing both ``yaw`` and ``q`` is an error).
    """

    model_config = ConfigDict(frozen=True)

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    q: Quaternion = Quaternion()
    frame: str

    @field_validator('x','y','z',mode='before')
    @classmethod
    def finite(cls,value):
        if isinstance(value,bool) or not isinstance(value,Real) or not math.isfinite(value):raise ValueError('pose coordinates must be finite numbers')
        return value

    @field_validator('frame')
    @classmethod
    def framed(cls,value):
        if not value.strip():raise ValueError('pose requires a reference frame')
        return value

    @model_validator(mode="before")
    @classmethod
    def _yaw_shorthand(cls, values):
        if isinstance(values, dict) and "yaw" in values:
            if "q" in values:
                raise ValueError("give yaw or q, not both")
            values = dict(values)
            yaw = values.pop("yaw")
            if isinstance(yaw,bool) or not isinstance(yaw,Real) or not math.isfinite(yaw):raise ValueError("yaw must be a finite number")
            values["q"] = Quaternion.from_euler(0.0, 0.0, yaw)
        return values

    @property
    def yaw(self) -> float:
        return self.q.to_euler()[2]

    @property
    def pitch(self) -> float:
        return self.q.to_euler()[1]

    @property
    def roll(self) -> float:
        return self.q.to_euler()[0]

    def distance_to(self, other: "Pose") -> float:
        """Euclidean distance. Raises FrameMismatch on different frames."""
        self._same_frame(other)
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2
                         + (self.z - other.z) ** 2)

    def angle_to(self, other: "Pose") -> float:
        """Planar bearing from this pose's position to the other's, in the
        shared frame (radians, atan2 convention). Raises FrameMismatch."""
        self._same_frame(other)
        return math.atan2(other.y - self.y, other.x - self.x)

    def shifted(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
                dyaw: float = 0.0) -> "Pose":
        """A new Pose offset in this pose's OWN frame (dx = forward)."""
        ox, oy, oz = self.q.rotate(dx, dy, dz)
        return Pose(
            x=self.x + ox, y=self.y + oy, z=self.z + oz,
            q=self.q.multiply(Quaternion.from_euler(0.0, 0.0, dyaw)),
            frame=self.frame,
        )

    def _same_frame(self, other: "Pose") -> None:
        if self.frame != other.frame:
            raise FrameMismatch(
                f"poses are in different frames: "
                f"{self.frame!r} vs {other.frame!r}"
            )


class Transform(BaseModel):
    """A rigid transform between two frames: applying it takes coordinates
    expressed in ``child`` to coordinates expressed in ``parent``."""

    model_config = ConfigDict(frozen=True)

    translation: Vec3
    rotation: Quaternion
    parent: str
    child: str
    stamp: float | None = None   # ROS time; absent on older agents

    def apply(self, point: Point) -> Point:
        """Re-express a point from the child frame in the parent frame.
        Raises FrameMismatch if the point is not in the child frame."""
        if point.frame != self.child:
            raise FrameMismatch(
                f"transform applies to frame {self.child!r}, "
                f"point is in {point.frame!r}"
            )
        x, y, z = self.rotation.rotate(point.x, point.y, point.z)
        return Point(
            x=x + self.translation.x,
            y=y + self.translation.y,
            z=z + self.translation.z,
            frame=self.parent,
        )

    def inverse(self) -> "Transform":
        """The reverse transform (parent-frame coordinates -> child)."""
        inv = self.rotation.inverse()
        x, y, z = inv.rotate(-self.translation.x, -self.translation.y,
                             -self.translation.z)
        return Transform(
            translation=Vec3(x=x, y=y, z=z),
            rotation=inv,
            parent=self.child,
            child=self.parent,
        )


class Twist(BaseModel):
    """Linear (m/s) and angular (rad/s) velocity."""

    model_config = ConfigDict(frozen=True)

    linear: Vec3 = Vec3()
    angular: Vec3 = Vec3()


class Polygon(BaseModel):
    """A 2D boundary: exploration bounds, exclusion zones, geofences.

    ``points`` are (x, y) in ``frame``, or (lat, lon) when ``frame`` is
    None (geo polygon).
    """

    model_config = ConfigDict(frozen=True)

    points: tuple[tuple[float, float], ...]
    frame: str | None = None


class GeoPose(BaseModel):
    """A position on Earth (WGS84). Accepted by navigation verbs
    alongside Pose once GPS lands (spec section 13; spec-only in v1)."""

    model_config = ConfigDict(frozen=True)

    lat: float
    lon: float
    alt: float | None = None      # meters above ellipsoid
    heading: float | None = None  # radians, ENU yaw


class MobilityState(BaseModel):
    """One mobility reply, including robot-clock odometry and localized pose.

    Older agents omit stamp/odometry_pose; neither is fabricated from wall time.
    """
    model_config = ConfigDict(frozen=True)
    pose: Pose
    velocity: Twist
    is_moving: bool
    distance_traveled: float
    stamp: float | None = None
    odometry_pose: Pose | None = None


class KinematicSnapshot(BaseModel):
    """Measured joint positions and their common ROS time, if available."""

    model_config = ConfigDict(frozen=True)
    positions: dict[str, float]
    stamp: float | None = None
