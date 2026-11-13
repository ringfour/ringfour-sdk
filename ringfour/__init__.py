"""ringfour: semantic, robot-agnostic Python SDK for ROS2 robots.

Pure-Python client. Never imports rclpy or any ROS package.

    from ringfour import Robot

    with Robot("ws://robot.local:10101") as robot:
        print(robot.name, robot.capabilities)

Every wire type is importable from
this package: spatial primitives and JSON wire models come from
``ringfour.models``; array-bearing data types from ``ringfour.data``.
"""

from .client import PowerAPI, PowerLane, Robot
from .description import JointSpec, RobotDescription
from .data import (
    DepthFrame,
    Event,
    ImageFrame,
    JointInfo,
    LaserScan,
    OccupancyGrid,
    PointCloud,
)
from .models import (
    BatteryState,
    CameraIntrinsics,
    DiagnosticsReport,
    FluidPressure,
    ForceTorque,
    Frontier,
    GeoPose,
    Illuminance,
    ImuReading,
    MagReading,
    MapInfo,
    Point,
    Polygon,
    Pose,
    PowerLaneState,
    Quaternion,
    RangeReading,
    SubsystemStatus,
    Temperature,
    Transform,
    Twist,
    Vec3,
)
from .enums import (
    AuditKind,
    Controller,
    Frame,
    GroupKind,
    Locomotion,
    Outcome,
    SlamMode,
)
from .errors import (
    AmbiguousCapability,
    CapabilityNotSupported,
    ConnectionFailed,
    ConnectionLost,
    ExecutionFailed,
    FrameExpired,
    FrameMismatch,
    NavigationFailed,
    Unreachable,
    UnknownPlace,
    ProtocolMismatch,
    RingfourError,
    SafetyViolation,
    TimeoutError,
)
from .version import CLIENT_VERSION, PROTOCOL_VERSION

__version__ = CLIENT_VERSION

__all__ = [
    "Robot",
    # spatial
    "Pose",
    "Point",
    "Quaternion",
    "Transform",
    "Twist",
    "Vec3",
    "Polygon",
    "GeoPose",
    # vision / ranging / arrays
    "CameraIntrinsics",
    "ImageFrame",
    "JointSpec",
    "RobotDescription",
    "DepthFrame",
    "PointCloud",
    "LaserScan",
    "OccupancyGrid",
    # sensors
    "RangeReading",
    "ImuReading",
    "MagReading",
    "ForceTorque",
    "Temperature",
    "FluidPressure",
    "Illuminance",
    "BatteryState",
    "DiagnosticsReport",
    "SubsystemStatus",
    # joints / status
    "JointInfo",
    "PowerAPI",
    "PowerLane",
    "PowerLaneState",
    "Event",
    "Frontier",
    "MapInfo",
    # Enums (spec 27) — every member is its own wire value.
    "AuditKind",
    "Controller",
    "Frame",
    "GroupKind",
    "Locomotion",
    "Outcome",
    "SlamMode",
    # errors
    "RingfourError",
    "ConnectionFailed",
    "ConnectionLost",
    "ProtocolMismatch",
    "CapabilityNotSupported",
    "AmbiguousCapability",
    "SafetyViolation",
    "FrameMismatch",
    "FrameExpired",
    "TimeoutError",
    "NavigationFailed",
    "Unreachable",
    "UnknownPlace",
    "ExecutionFailed",
    "PROTOCOL_VERSION",
    "CLIENT_VERSION",
]

from .models.execution import (JointValue, JointConfiguration, JointTrajectory, OperationState, CancelReceipt, StopReport, JointResidual, NavigationResult, TrajectoryResult, NavigationProgress, TrajectoryProgress, OperationSnapshot)
from .errors import ModelChanged, OperationCancelled, WaitTimeout, OperationTimeout
__all__ += ['JointValue', 'JointConfiguration', 'JointTrajectory', 'OperationState', 'CancelReceipt', 'StopReport', 'JointResidual', 'NavigationResult', 'TrajectoryResult', 'NavigationProgress', 'TrajectoryProgress', 'OperationSnapshot', 'ModelChanged', 'OperationCancelled', 'WaitTimeout', 'OperationTimeout']

from .operations import Operation
__all__ += ["Operation"]

from .models.measurements import Measurement, SignalCondition, GuardStatus
__all__ += ['Measurement', 'SignalCondition', 'GuardStatus']

from .models.execution import RouteResult, RouteProgress, ToolResult, ToolProgress
__all__ += ['RouteResult', 'RouteProgress', 'ToolResult', 'ToolProgress']

from .errors import StateUnavailable, StateStale, StopUnverified, CompletionUnverified, CleanupFailed, PermissionDenied, ResourceBusy
__all__ += ['StateUnavailable', 'StateStale', 'StopUnverified', 'CompletionUnverified', 'CleanupFailed', 'PermissionDenied', 'ResourceBusy']

from .errors import SubmissionUnknown, CommandRequired, CommandConflict, CommandExpired, AgentEpochChanged
from .models.commands import CommandRecord, OperationRecord
__all__ += ['SubmissionUnknown', 'CommandRequired', 'CommandConflict', 'CommandExpired', 'AgentEpochChanged', 'CommandRecord', 'OperationRecord']
