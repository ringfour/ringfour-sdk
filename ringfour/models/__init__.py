"""Shared pydantic models: the single source of truth for the wire schema.

The client SDK and the agent both import these. ``protocol.json`` is
generated mechanically from them (scripts/gen_protocol.py). Array-bearing
types (ImageFrame, PointCloud, LaserScan, OccupancyGrid, ...) live in
``ringfour.data`` instead — their bulk travels via PayloadRef, not JSON.
"""

from .audit import AuditEvent
from .protocol import (
    GroupInfo,
    Hello,
    PayloadHeader,
    ProfileInfo,
    ToolInfo,
    Welcome,
)
from .sensing import CameraIntrinsics, PayloadRef
from .sensors import (
    BatteryState,
    DiagnosticsReport,
    FluidPressure,
    ForceTorque,
    Illuminance,
    ImuReading,
    MagReading,
    PowerLaneState,
    RangeReading,
    SubsystemStatus,
    Temperature,
)
from .spatial import (
    KinematicSnapshot,
    MobilityState,
    GeoPose,
    Point,
    Polygon,
    Pose,
    Quaternion,
    Transform,
    Twist,
    Vec3,
)
from .status import Frontier, MapInfo
from .tasks import DispatchReceipt, TaskError, TaskUpdate

__all__ = [
    "DispatchReceipt",
    "AuditEvent",
    "BatteryState",
    "CameraIntrinsics",
    "DiagnosticsReport",
    "FluidPressure",
    "ForceTorque",
    "Frontier",
    "GroupInfo",
    "GeoPose",
    "Hello",
    "Illuminance",
    "ImuReading",
    "MagReading",
    "MobilityState",
    "KinematicSnapshot",
    "MapInfo",
    "PayloadHeader",
    "PayloadRef",
    "Point",
    "Polygon",
    "Pose",
    "PowerLaneState",
    "ProfileInfo",
    "Quaternion",
    "RangeReading",
    "SubsystemStatus",
    "TaskError",
    "TaskUpdate",
    "Temperature",
    "ToolInfo",
    "Transform",
    "Twist",
    "Vec3",
    "Welcome",
]

from .execution import (JointValue, JointConfiguration, JointTrajectory, OperationState, CancelReceipt, StopReport, JointResidual, NavigationResult, TrajectoryResult, NavigationProgress, TrajectoryProgress, OperationSnapshot)
__all__ += ['JointValue', 'JointConfiguration', 'JointTrajectory', 'CancelReceipt', 'StopReport', 'JointResidual', 'NavigationResult', 'TrajectoryResult', 'NavigationProgress', 'TrajectoryProgress', 'OperationSnapshot']

from .measurements import Measurement, SignalCondition, GuardStatus
__all__ += ['Measurement', 'SignalCondition', 'GuardStatus']

from .execution import RouteResult, RouteProgress, ToolResult, ToolProgress
__all__ += ['RouteResult', 'RouteProgress', 'ToolResult', 'ToolProgress']

from .commands import CommandRecord, OperationRecord
__all__ += ['CommandRecord', 'OperationRecord']
