# ringfour type reference

**Generated — do not edit.** Produced by `ringfour-sdk/scripts/gen_types_reference.py`
from the shipped source; regenerate with:

```
python3 ringfour-sdk/scripts/gen_protocol.py
python3 ringfour-sdk/scripts/gen_types_reference.py
```

Client 0.1.0.dev0, protocol 1.

Everything here is importable from the `ringfour` package root
(`from ringfour import Pose, Phase, ImageFrame`). The wire schema for the same
types — including JSON-schema `enum` constraints — is `engine/protocol.json`.

> The RGB image type is `ImageFrame`; `Frame` is the enum of frame
> *aliases* (`@map`/`@odom`/`@base`).


## Enums

Every member subclasses `str`, so the member **is** the wire value and the YAML value: `Controller.TRAJECTORY == "trajectory"` is True and bare strings are accepted anywhere a member is.


### `AuditKind`

The kind of occurrence an AuditEvent records (spec section 18).

| Member | Value |
|---|---|
| `AuditKind.COMMAND` | `'command'` |
| `AuditKind.RESULT` | `'result'` |
| `AuditKind.SAFETY` | `'safety'` |
| `AuditKind.WATCH` | `'watch'` |
| `AuditKind.ESTOP` | `'estop'` |
| `AuditKind.CONNECTION` | `'connection'` |


### `Controller`

The *role* a controller plays for a group, not its ROS node name.

The profile maps role -> controller name (``{trajectory: arm_trajectory_controller, ...}``), so a mission asks for a capability and the profile answers with the robot's spelling of it. Every declared role must be loaded at boot (plan 6.0b) so the switch is a switch, never a load.

| Member | Value |
|---|---|
| `Controller.TRAJECTORY` | `'trajectory'` |
| `Controller.STREAM` | `'stream'` |
| `Controller.VELOCITY` | `'velocity'` |
| `Controller.TOOL` | `'tool'` |


### `Frame`

The three frame aliases the agent resolves against the profile.

These stay strings on the wire — ``"@map"`` is resolved agent-side against ``profile.frames``, so the client never needs to know that this robot calls its base ``base_link`` and the next one ``base_footprint``.

(The Phase 4 RGB image type that used to own this name is now ``ImageFrame`` — decision 17, revised 2026-08-24. The enum keeps the spec section 27 spelling, which is the one missions read.)

| Member | Value |
|---|---|
| `Frame.MAP` | `'@map'` |
| `Frame.ODOM` | `'@odom'` |
| `Frame.BASE` | `'@base'` |


### `GroupKind`

What a joint group is for, which decides what it is allowed to do.

ARM groups take IK and cartesian targets (they have a ``tip``); POINTING (pan/tilt) and LINEAR (lifts, rails) are joint-space only.

| Member | Value |
|---|---|
| `GroupKind.ARM` | `'arm'` |
| `GroupKind.POINTING` | `'pointing'` |
| `GroupKind.LINEAR` | `'linear'` |


### `Locomotion`

The base's drive type. Gates what teleop accepts: ``lateral`` velocity needs OMNI.

| Member | Value |
|---|---|
| `Locomotion.DIFFERENTIAL` | `'differential'` |
| `Locomotion.OMNI` | `'omni'` |
| `Locomotion.LEGGED` | `'legged'` |


### `Outcome`

How an audited call ended.

REJECTED is the safety layer refusing before the handler ran; ERROR is the handler (or a task body) failing; CANCELLED is a running task stopped on request.

CANCELLED is a fourth member beyond the three in spec section 27 (decision 18, 2026-08-24). The audit log is the safety record, and it has to distinguish "the operator stopped it" from "it broke" — collapsing a cancellation into ERROR would make the log lie about the one thing it exists to answer. The free text that used to be stuffed in here ("error: <msg>", "failed: <msg>") moved to ``detail``, which is where the model always said it belonged.

| Member | Value |
|---|---|
| `Outcome.SUCCESS` | `'success'` |
| `Outcome.ERROR` | `'error'` |
| `Outcome.REJECTED` | `'rejected'` |
| `Outcome.CANCELLED` | `'cancelled'` |


### `Phase`

Progress state of a motion, shared by NavStatus and TrajStatus.

Not every phase is reachable from every verb: navigation never reports PLANNING (Nav2 plans inside MOVING) and trajectory execution has no RECOVERING. The union is one vocabulary on purpose — a mission that switches on ``status.phase`` reads the same either way.

| Member | Value |
|---|---|
| `Phase.PLANNING` | `'planning'` |
| `Phase.MOVING` | `'moving'` |
| `Phase.ARRIVED` | `'arrived'` |
| `Phase.RECOVERING` | `'recovering'` |
| `Phase.STUCK` | `'stuck'` |
| `Phase.CANCELLED` | `'cancelled'` |
| `Phase.FAILED` | `'failed'` |


### `SlamMode`

slam_toolbox's mode. Switched at service level, never by restarting the process (see the nav restructure).

| Member | Value |
|---|---|
| `SlamMode.MAPPING` | `'mapping'` |
| `SlamMode.LOCALIZATION` | `'localization'` |
| `SlamMode.IDLE` | `'idle'` |


### `Transport`

How array bytes reach the client.

SHM maps local Linux /dev/shm read-only. NET sends raw numeric arrays in tagged binary WebSocket frames, independently of JSON-RPC command objects. New TCP clients negotiate NET; Unix clients default to SHM. Both can select explicitly with Robot(..., transport=Transport.NET or Transport.SHM).

| Member | Value |
|---|---|
| `Transport.SHM` | `'shm'` |
| `Transport.NET` | `'net'` |


## Wire models

Pydantic models shared by the client and the agent. These are what `protocol.json` is generated from; they cross the JSON-RPC link verbatim.


### `DispatchReceipt`

A vendor command was dispatched; physical completion is unverified.

| Field | Type | Default |
|---|---|---|
| `action` | `str` | *required* |
| `dispatched` | `bool` | *required* |
| `completion` | `'unverified'` | `'unverified'` |


### `AuditEvent`

One audited occurrence on the agent.

``stamp`` is ISO 8601 UTC. ``client`` is the connection id that caused the event (None for agent-internal events). ``call`` is the wire method plus its params with binary payloads elided. ``outcome`` carries the reason in ``detail``.

| Field | Type | Default |
|---|---|---|
| `stamp` | `str` | *required* |
| `kind` | `AuditKind` | *required* |
| `client` | `int | None` | `None` |
| `call` | `str | None` | `None` |
| `outcome` | `Outcome | None` | `None` |
| `task_id` | `str | None` | `None` |
| `detail` | `dict[str, Any]` | `{}` |


### `BatteryState`

Battery status (spec section 19).

``level`` is 0-100; None until data arrives or when the driver sends NaN and the profile provides no voltage curve.

| Field | Type | Default | Notes |
|---|---|---|---|
| `level` | `float | None` | *required* |  |
| `voltage` | `float | None` | `None` |  |
| `current` | `float | None` | `None` | Amps; negative = discharging |
| `temperature` | `float | None` | `None` | Celsius |
| `is_charging` | `bool | None` | `None` |  |
| `power_supply_status` | `'charging' | 'discharging' | 'not_charging' | 'full' | 'unknown'` | `'unknown'` |  |
| `stamp` | `float | None` | `None` |  |


### `CameraIntrinsics`

Camera calibration (spec section 10), from the camera_info topic.

| Field | Type | Default |
|---|---|---|
| `fx` | `float` | *required* |
| `fy` | `float` | *required* |
| `cx` | `float` | *required* |
| `cy` | `float` | *required* |
| `width` | `int` | *required* |
| `height` | `int` | *required* |
| `distortion_model` | `str` | `'plumb_bob'` |
| `distortion_coeffs` | `tuple[float, ...]` | `()` |


### `DiagnosticsReport`

System health snapshot from /diagnostics.

| Field | Type | Default |
|---|---|---|
| `subsystems` | `tuple[SubsystemStatus, ...]` | *required* |
| `stamp` | `float` | *required* |

- `check() -> bool` — True when no subsystem reports error or stale.


### `FluidPressure`

Barometric/fluid pressure, Pascals.

| Field | Type | Default |
|---|---|---|
| `pressure` | `float` | *required* |
| `variance` | `float` | `0.0` |
| `stamp` | `float` | *required* |
| `frame` | `str` | *required* |


### `ForceTorque`

6-axis force/torque reading (wrist sensors and the like).

A discovered wrench signal can form a persistent guard condition; threshold and fresh source/receive evidence belong to the deployment.

| Field | Type | Default | Notes |
|---|---|---|---|
| `force` | `Vec3` | *required* | Newtons |
| `torque` | `Vec3` | *required* | Newton-meters |
| `stamp` | `float` | *required* |  |
| `frame` | `str` | *required* |  |


### `Frontier`

A candidate exploration target (Phase 5).

| Field | Type | Default | Notes |
|---|---|---|---|
| `pose` | `Pose` | *required* |  |
| `area` | `float` | *required* | estimated unmapped area beyond, m^2 |
| `distance` | `float` | *required* | from current position, meters |


### `GroupInfo`

One group's static shape, delivered in the handshake.

Everything here is profile config plus the joint list the agent resolved off the controller at startup. It is in the handshake and not behind a call because kinematics is client-side (spec 26.2): a mission doing `kin.reachable()` in a loop must not pay a round trip per solve, and it cannot solve at all without knowing the tip and the joint order.

| Field | Type | Default |
|---|---|---|
| `kind` | `GroupKind` | *required* |
| `tip` | `str | None` | `None` |
| `dof_mask` | `tuple[int, ...] | None` | `None` |
| `joints` | `tuple[str, ...]` | `()` |
| `tool` | `ToolInfo | None` | `None` |
| `pointing_frame` | `str | None` | `None` |
| `workspace` | `dict[str, tuple[float, float]] | None` | `None` |
| `control_modes` | `tuple[Controller, ...]` | `()` |


### `GeoPose`

A position on Earth (WGS84). Accepted by navigation verbs alongside Pose once GPS lands (spec section 13; spec-only in v1).

| Field | Type | Default | Notes |
|---|---|---|---|
| `lat` | `float` | *required* |  |
| `lon` | `float` | *required* |  |
| `alt` | `float | None` | `None` | meters above ellipsoid |
| `heading` | `float | None` | `None` | radians, ENU yaw |


### `Hello`

First client message. The agent rejects a mismatched protocol with a ProtocolMismatch error before anything else is allowed.

| Field | Type | Default |
|---|---|---|
| `protocol` | `int` | *required* |
| `client_version` | `str` | *required* |


### `Illuminance`

Ambient light level, Lux.

| Field | Type | Default |
|---|---|---|
| `illuminance` | `float` | *required* |
| `variance` | `float` | `0.0` |
| `stamp` | `float` | *required* |
| `frame` | `str` | *required* |


### `ImuReading`

IMU output. ``orientation`` is None when the IMU provides no orientation estimate. Covariances are 9-element row-major; ROS marks "unknown" with -1 in element [0], preserved as None here.

| Field | Type | Default | Notes |
|---|---|---|---|
| `orientation` | `Quaternion | None` | *required* |  |
| `angular_velocity` | `Vec3` | *required* | rad/s |
| `linear_acceleration` | `Vec3` | *required* | m/s^2, includes gravity |
| `orientation_covariance` | `tuple[float, ...] | None` | `None` |  |
| `angular_velocity_covariance` | `tuple[float, ...] | None` | `None` |  |
| `linear_acceleration_covariance` | `tuple[float, ...] | None` | `None` |  |
| `stamp` | `float` | *required* |  |
| `frame` | `str` | *required* |  |


### `MagReading`

Magnetometer measurement (Tesla).

| Field | Type | Default |
|---|---|---|
| `magnetic_field` | `Vec3` | *required* |
| `covariance` | `tuple[float, ...] | None` | `None` |
| `stamp` | `float` | *required* |
| `frame` | `str` | *required* |


### `MobilityState`

One mobility reply, including robot-clock odometry and localized pose.

Older agents omit stamp/odometry_pose; neither is fabricated from wall time.

| Field | Type | Default |
|---|---|---|
| `pose` | `Pose` | *required* |
| `velocity` | `Twist` | *required* |
| `is_moving` | `bool` | *required* |
| `distance_traveled` | `float` | *required* |
| `stamp` | `float | None` | `None` |
| `odometry_pose` | `Pose | None` | `None` |


### `KinematicSnapshot`

Measured joint positions and their common ROS time, if available.

| Field | Type | Default |
|---|---|---|
| `positions` | `dict[str, float]` | *required* |
| `stamp` | `float | None` | `None` |


### `MapInfo`

Metadata about a saved map (Phase 5).

| Field | Type | Default | Notes |
|---|---|---|---|
| `name` | `str` | *required* |  |
| `created_at` | `str` | *required* | ISO timestamp |
| `resolution` | `float` | *required* | meters per cell |
| `dimensions` | `tuple[float, float]` | *required* | (width, height) meters |


### `PayloadHeader`

Announces one binary frame. ``kind`` says what the bytes are ("image/jpeg", "depth/f32", ...); ``meta`` carries kind-specific fields (camera name, stamp, array shape, ...).

| Field | Type | Default |
|---|---|---|
| `payload_id` | `int` | *required* |
| `kind` | `str` | *required* |
| `meta` | `dict[str, Any]` | *required* |


### `PayloadRef`

Array metadata: name is a SHM segment or connection-local NET payload ID.

NET preserves exact raw dtype/shape values, including metric float depth; it does not apply image compression or infer completion of robot commands.

| Field | Type | Default | Notes |
|---|---|---|---|
| `transport` | `Transport` | `'shm'` |  |
| `name` | `str` | *required* | shm segment name (file name under /dev/shm) |
| `dtype` | `str` | *required* | numpy dtype string ("uint8", "float32", ...) |
| `shape` | `tuple[int, ...]` | *required* | array shape, C-contiguous |
| `size` | `int` | *required* | total bytes, == prod(shape) * itemsize |


### `Point`

A located point: position + reference frame.

| Field | Type | Default |
|---|---|---|
| `x` | `float` | *required* |
| `y` | `float` | *required* |
| `z` | `float` | `0.0` |
| `frame` | `str` | *required* |


### `Polygon`

A 2D boundary: exploration bounds, exclusion zones, geofences.

``points`` are (x, y) in ``frame``, or (lat, lon) when ``frame`` is None (geo polygon).

| Field | Type | Default |
|---|---|---|
| `points` | `tuple[tuple[float, float], ...]` | *required* |
| `frame` | `str | None` | `None` |


### `Pose`

Position + orientation in a frame.

Shorthand: ``Pose(x=1, y=2, yaw=1.57, frame="map")`` builds the quaternion for you (passing both ``yaw`` and ``q`` is an error).

| Field | Type | Default |
|---|---|---|
| `x` | `float` | `0.0` |
| `y` | `float` | `0.0` |
| `z` | `float` | `0.0` |
| `q` | `Quaternion` | `Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)` |
| `frame` | `str` | *required* |

- `angle_to(other: Pose) -> float` — Planar bearing from this pose's position to the other's, in the shared frame (radians, atan2 convention). Raises FrameMismatch.
- `distance_to(other: Pose) -> float` — Euclidean distance. Raises FrameMismatch on different frames.
- `pitch` *(property)*
- `roll` *(property)*
- `shifted(dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, dyaw: float = 0.0) -> Pose` — A new Pose offset in this pose's OWN frame (dx = forward).
- `yaw` *(property)*


### `PowerLaneState`

Latest reading of one switchable power lane: on/off, voltage, current. ``on`` reflects the MOSFET state the firmware reports (may lag briefly after a command while the switch settles).

| Field | Type | Default | Notes |
|---|---|---|---|
| `name` | `str` | *required* |  |
| `on` | `bool` | *required* |  |
| `voltage` | `float` | *required* | Volts |
| `current` | `float` | *required* | Amps |
| `stamp` | `float` | *required* |  |


### `ProfileInfo`

What the client learns about the robot at handshake time.

``capabilities`` lists the capability names the profile provides (e.g. "mobility", "lidar"); see spec section 4 for the registry.

| Field | Type | Default |
|---|---|---|
| `name` | `str` | *required* |
| `version` | `str` | *required* |
| `model` | `str` | *required* |
| `capabilities` | `tuple[str, ...]` | *required* |


### `Quaternion`

Orientation as a unit quaternion. Identity by default.

| Field | Type | Default |
|---|---|---|
| `x` | `float` | `0.0` |
| `y` | `float` | `0.0` |
| `z` | `float` | `0.0` |
| `w` | `float` | `1.0` |

- `inverse() -> Quaternion` — The opposite rotation (conjugate; quaternions here are unit).
- `multiply(other: Quaternion) -> Quaternion` — Hamilton product self * other (apply ``other``'s rotation first, then self's).
- `rotate(x: float, y: float, z: float) -> tuple[float, float, float]` — Rotate the vector (x, y, z) by this quaternion.
- `to_euler() -> tuple[float, float, float]` — (roll, pitch, yaw) in radians. Pitch is clamped at the gimbal-lock poles (|pitch| = pi/2).
- `unit_length()`


### `RangeReading`

A single-beam distance measurement (ultrasonic / IR / ToF point sensor). ``range`` is inf when there is no return.

A discovered range signal can form a persistent guard condition. The deployment must review its threshold, freshness and covered motion.

| Field | Type | Default | Notes |
|---|---|---|---|
| `range` | `float` | *required* | meters; inf = no return |
| `min_range` | `float` | *required* |  |
| `max_range` | `float` | *required* |  |
| `field_of_view` | `float` | *required* | radians, cone width |
| `radiation_type` | `'ultrasonic' | 'infrared'` | *required* |  |
| `stamp` | `float` | *required* |  |
| `frame` | `str` | *required* |  |


### `SubsystemStatus`

One subsystem's health line in a DiagnosticsReport.

| Field | Type | Default |
|---|---|---|
| `name` | `str` | *required* |
| `level` | `'ok' | 'warn' | 'error' | 'stale'` | *required* |
| `message` | `str` | `''` |
| `values` | `dict[str, str]` | `{}` |


### `TaskError`

Why a task failed. ``type`` names a ringfour exception class; ``data`` carries its extras (reason, limit, ...).

| Field | Type | Default |
|---|---|---|
| `type` | `str` | *required* |
| `message` | `str` | *required* |
| `data` | `dict[str, Any]` | `{}` |


### `TaskUpdate`

One ``task.update`` notification.

| Field | Type | Default | Notes |
|---|---|---|---|
| `task_id` | `str` | *required* |  |
| `sequence` | `int` | `0` |  |
| `report_age` | `float` | `0.0` | agent-computed age of the retained report |
| `cleanup` | `dict[str, Any] | None` | `None` |  |
| `state` | `str` | *required* | one of TASK_STATES |
| `progress` | `float | None` | `None` | 0..1 when the task can estimate it |
| `message` | `str | None` | `None` |  |
| `data` | `dict[str, Any] | None` | `None` |  |
| `result` | `dict[str, Any] | None` | `None` | set on "succeeded" |
| `error` | `TaskError | None` | `None` | set on "failed" |


### `Temperature`

Environmental temperature, Celsius.

| Field | Type | Default | Notes |
|---|---|---|---|
| `temperature` | `float` | *required* |  |
| `variance` | `float` | `0.0` | 0 = unknown |
| `stamp` | `float` | *required* |  |
| `frame` | `str` | *required* |  |


### `ToolInfo`

The tool fitted to a group, as the client sees it.

| Field | Type | Default |
|---|---|---|
| `type` | `str` | *required* |
| `joint` | `str | None` | `None` |
| `open` | `float | None` | `None` |
| `closed` | `float | None` | `None` |


### `Transform`

A rigid transform between two frames: applying it takes coordinates expressed in ``child`` to coordinates expressed in ``parent``.

| Field | Type | Default | Notes |
|---|---|---|---|
| `translation` | `Vec3` | *required* |  |
| `rotation` | `Quaternion` | *required* |  |
| `parent` | `str` | *required* |  |
| `child` | `str` | *required* |  |
| `stamp` | `float | None` | `None` | ROS time; absent on older agents |

- `apply(point: Point) -> Point` — Re-express a point from the child frame in the parent frame. Raises FrameMismatch if the point is not in the child frame.
- `inverse() -> Transform` — The reverse transform (parent-frame coordinates -> child).


### `Twist`

Linear (m/s) and angular (rad/s) velocity.

| Field | Type | Default |
|---|---|---|
| `linear` | `Vec3` | `Vec3(x=0.0, y=0.0, z=0.0)` |
| `angular` | `Vec3` | `Vec3(x=0.0, y=0.0, z=0.0)` |


### `Vec3`

A frameless 3-vector.

| Field | Type | Default |
|---|---|---|
| `x` | `float` | `0.0` |
| `y` | `float` | `0.0` |
| `z` | `float` | `0.0` |


### `Welcome`

The agent's handshake reply: protocol version + capability report.

``instances`` lists the named instances of multi-instance capabilities ({"arms": ("left", "right"), ...}); it is what lets the client resolve singular sugar (robot.arm vs robot.arms["left"]) without extra calls.

| Field | Type | Default |
|---|---|---|
| `protocol` | `int` | *required* |
| `profile` | `ProfileInfo` | *required* |
| `capabilities` | `tuple[str, ...]` | *required* |
| `instances` | `dict[str, tuple[str, ...]]` | `{}` |
| `features` | `dict[str, Any]` | `{}` |
| `description` | `str | None` | `None` |
| `description_sha` | `str | None` | `None` |
| `group_description_shas` | `dict[str, str]` | `{}` |
| `groups` | `dict[str, GroupInfo]` | `{}` |
| `sensors` | `dict[str, dict[str, dict[str, Any]]]` | `{}` |
| `actions` | `dict[str, dict[str, Any]]` | `{}` |


### `JointValue`

One model-bound joint value in radians or metres, with model bounds.

| Field | Type | Default |
|---|---|---|
| `name` | `str` | *required* |
| `position` | `float` | *required* |
| `unit` | `'rad' | 'm'` | *required* |
| `lower` | `float | None` | `None` |
| `upper` | `float | None` | `None` |

- `bounded()`


### `JointConfiguration`

Exact named joint coverage tied to one robot, group and model revision.

| Field | Type | Default |
|---|---|---|
| `robot` | `str` | *required* |
| `group` | `str` | *required* |
| `model_revision` | `str` | *required* |
| `joints` | `tuple[JointValue, ...]` | *required* |

- `identity()`
- `positions` *(property)*
- `vector(names)`


### `JointTrajectory`

Model-bound configurations at strictly increasing finite times from start.

Endpoint validation does not establish interpolation speed/acceleration safety.

| Field | Type | Default |
|---|---|---|
| `configurations` | `tuple[JointConfiguration, ...]` | *required* |
| `times` | `tuple[float, ...]` | *required* |

- `coherent()`


### `CancelReceipt`

Cancellation request acknowledgement, never evidence of measured rest.

| Field | Type | Default |
|---|---|---|
| `operation_id` | `str` | *required* |
| `accepted` | `bool` | *required* |
| `already_terminal` | `bool` | *required* |


### `StopReport`

Measured software stopping criterion; hardware commissioning is separate.

| Field | Type | Default |
|---|---|---|
| `resource` | `str` | *required* |
| `verified` | `bool` | *required* |
| `criterion` | `str` | *required* |
| `physical_stop` | `'unverified' | 'uncommissioned'` | *required* |
| `elapsed` | `float | None` | `None` |
| `dwell` | `float | None` | `None` |


### `JointResidual`

One measured endpoint residual with its own unit and acceptance tolerance.

| Field | Type | Default |
|---|---|---|
| `name` | `str` | *required* |
| `error` | `float` | *required* |
| `unit` | `'rad' | 'm'` | *required* |
| `tolerance` | `float` | *required* |


### `NavigationResult`

Verified arrival at a measured framed pose, not controller acknowledgement.

| Field | Type | Default |
|---|---|---|
| `operation_id` | `str` | *required* |
| `final_pose` | `Pose` | *required* |
| `position_error` | `float` | *required* |
| `orientation_error` | `float` | *required* |
| `position_tolerance` | `float` | *required* |
| `orientation_tolerance` | `float` | *required* |
| `stop` | `StopReport` | *required* |

- `verified_arrival()`


### `TrajectoryResult`

Complete measured joint endpoint residuals and stopping evidence.

| Field | Type | Default |
|---|---|---|
| `operation_id` | `str` | *required* |
| `configuration` | `JointConfiguration` | *required* |
| `residuals` | `tuple[JointResidual, ...]` | *required* |
| `stop` | `StopReport` | *required* |


### `NavigationProgress`

Available navigation feedback; distance is an estimate, not completion.

| Field | Type | Default |
|---|---|---|
| `pose` | `Pose | None` | `None` |
| `remaining_distance` | `float | None` | `None` |


### `TrajectoryProgress`

Measured joint positions when the controller provides complete feedback.

| Field | Type | Default |
|---|---|---|
| `positions` | `tuple[JointValue, ...] | None` | `None` |
| `elapsed_fraction` | `float | None` | `None` |
| `fraction_basis` | `'elapsed_time' | None` | `None` |


### `OperationSnapshot`

Immutable cached state; receipt age and sequence make staleness explicit.

| Field | Type | Default |
|---|---|---|
| `operation_id` | `str` | *required* |
| `state` | `OperationState` | *required* |
| `sequence` | `int` | *required* |
| `observed_age` | `float` | *required* |
| `progress` | `TProgress | None` | `None` |


### `Measurement`

A value and agent-computed freshness; unknown source age stays explicit.

| Field | Type | Default |
|---|---|---|
| `value` | `T | None` | *required* |
| `robot_id` | `str` | *required* |
| `model_revision` | `str | None` | *required* |
| `source_stamp` | `float | None` | `None` |
| `clock_domain` | `str | None` | `None` |
| `received_at` | `float | None` | `None` |
| `source_age` | `float | None` | `None` |
| `receive_age` | `float | None` | `None` |
| `validity` | `'valid' | 'unavailable' | 'stale' | 'invalid'` | *required* |
| `reason` | `str | None` | `None` |
| `frame` | `str | None` | `None` |


### `SignalCondition`

A validated signal threshold with a required source/receipt age budget.

| Field | Type | Default |
|---|---|---|
| `robot_id` | `str` | *required* |
| `path` | `str` | *required* |
| `comparison` | `'below' | 'above'` | *required* |
| `threshold` | `float` | *required* |
| `max_age` | `float` | `0.5` |
| `frame` | `str | None` | `None` |


### `GuardStatus`

Guard state is independent of operator e-stop and cleanup quarantine.

| Field | Type | Default |
|---|---|---|
| `guard_id` | `str` | *required* |
| `condition` | `SignalCondition` | *required* |
| `tripped` | `bool` | *required* |
| `reason` | `str | None` | `None` |
| `observation` | `Measurement[float]` | *required* |
| `cleanup_confirmed` | `bool | None` | `None` |
| `evaluation_age` | `float` | `0.0` |


### `RouteResult`

Verified arrival evidence for every waypoint in a finite route.

| Field | Type | Default |
|---|---|---|
| `operation_id` | `str` | *required* |
| `arrivals` | `tuple[NavigationResult, ...]` | *required* |


### `RouteProgress`

Route index and available navigation feedback; no inferred arrival.

| Field | Type | Default |
|---|---|---|
| `waypoint_index` | `int` | *required* |
| `waypoint_count` | `int` | *required* |
| `navigation` | `NavigationProgress | None` | `None` |


### `ToolResult`

Measured positioning evidence. This does not prove an object is held.

| Field | Type | Default |
|---|---|---|
| `operation_id` | `str` | *required* |
| `group` | `str` | *required* |
| `joint` | `str` | *required* |
| `position` | `float` | *required* |
| `unit` | `'rad' | 'm'` | *required* |
| `target` | `float` | *required* |
| `residual` | `float` | *required* |
| `tolerance` | `float` | *required* |
| `stop` | `StopReport` | *required* |


### `ToolProgress`

!!! abstract "Usage Documentation" [Models](../concepts/models.md)

A base class for creating Pydantic models.

Attributes: __class_vars__: The names of the class variables defined on the model. __private_attributes__: Metadata about the private attributes of the model. __signature__: The synthesized `__init__` [`Signature`][inspect.Signature] of the model.

__pydantic_complete__: Whether model building is completed, or if there are still undefined fields. __pydantic_core_schema__: The core schema of the model. __pydantic_custom_init__: Whether the model has a custom `__init__` function. __pydantic_decorators__: Metadata containing the decorators defined on the model. This replaces `Model.__validators__` and `Model.__root_validators__` from Pydantic V1. __pydantic_generic_metadata__: A dictionary containing metadata about generic Pydantic models. The `origin` and `args` items map to the [`__origin__`][genericalias.__origin__] and [`__args__`][genericalias.__args__] attributes of [generic aliases][types-genericalias], and the `parameter` item maps to the `__parameter__` attribute of generic classes. __pydantic_parent_namespace__: Parent namespace of the model, used for automatic rebuilding of models. __pydantic_post_init__: The name of the post-init method for the model, if defined. __pydantic_root_model__: Whether the model is a [`RootModel`][pydantic.root_model.RootModel]. __pydantic_serializer__: The `pydantic-core` `SchemaSerializer` used to dump instances of the model. __pydantic_validator__: The `pydantic-core` `SchemaValidator` used to validate instances of the model.

__pydantic_fields__: A dictionary of field names and their corresponding [`FieldInfo`][pydantic.fields.FieldInfo] objects. __pydantic_computed_fields__: A dictionary of computed field names and their corresponding [`ComputedFieldInfo`][pydantic.fields.ComputedFieldInfo] objects.

__pydantic_extra__: A dictionary containing extra values, if [`extra`][pydantic.config.ConfigDict.extra] is set to `'allow'`. __pydantic_fields_set__: The names of fields explicitly set during instantiation. __pydantic_private__: Values of private attributes set on the model instance.

| Field | Type | Default |
|---|---|---|
| `position` | `float | None` | `None` |
| `unit` | `'rad' | 'm'` | *required* |


### `CommandRecord`

Durable acceptance reply, or explicit unknown outcome after a crash.

| Field | Type | Default |
|---|---|---|
| `command_id` | `str` | *required* |
| `epoch` | `str` | *required* |
| `method` | `str` | *required* |
| `state` | `'pending' | 'completed' | 'unknown'` | *required* |
| `result` | `Any` | `None` |
| `error` | `dict[str, Any] | None` | `None` |
| `operation_id` | `str | None` | `None` |
| `expires_at` | `float` | *required* |
| `retained_until` | `float` | *required* |


### `OperationRecord`

Queryable terminal evidence; active work at a crash remains unknown.

| Field | Type | Default |
|---|---|---|
| `operation_id` | `str` | *required* |
| `epoch` | `str` | *required* |
| `state` | `'active' | 'terminal' | 'unknown'` | *required* |
| `snapshot` | `TaskUpdate` | *required* |
| `retained_until` | `float | None` | `None` |
| `physical_outcome` | `'recorded' | 'unknown'` | *required* |


## Array-bearing types

Array-bearing data types handed to user code (spec sections 5 and 10).

These are frozen dataclasses, not pydantic models: they carry numpy arrays, which never cross the wire as JSON. The wire carries a ``PayloadRef`` descriptor (ringfour.models.sensing); the transport layer (ringfour.payloads) maps it to the array, and these types are how the user receives it.

Arrays are read-only views over shared memory (zero copy, local v1). They stay valid for the lifetime of the Python object even after the agent expires the segment — POSIX keeps mapped memory alive until the mapping is dropped. Call ``np.array(x.image)`` if you need a private mutable copy.


### `DepthFrame`

One depth frame, meters.

| Field | Type | Default | Notes |
|---|---|---|---|
| `id` | `str` | *required* |  |
| `depth` | `ndarray` | *required* | (H, W) float32 meters |
| `stamp` | `float` | *required* |  |
| `camera` | `str` | *required* |  |


### `Event`

One system event from ``robot.events()`` (estop, watch fires; nav aborts join in Phase 5).

| Field | Type | Default |
|---|---|---|
| `kind` | `str` | *required* |
| `stamp` | `float` | *required* |
| `data` | `dict[str, Any]` | `{}` |


### `ImageFrame`

One RGB camera frame. ``id`` is the server-side handle for deprojection, valid for the profile's ``frame_ttl`` (default 30 s, sized for a cloud VLM round trip).

| Field | Type | Default | Notes |
|---|---|---|---|
| `id` | `str` | *required* |  |
| `image` | `ndarray` | *required* | (H, W, 3) uint8 |
| `stamp` | `float` | *required* |  |
| `camera` | `str` | *required* |  |


### `JointInfo`

One joint's state (spec section 5). ``limits`` from the URDF; None for continuous joints or before the robot description arrived.

| Field | Type | Default |
|---|---|---|
| `position` | `float` | *required* |
| `velocity` | `float` | *required* |
| `effort` | `float` | *required* |
| `limits` | `tuple[float, float] | None` | *required* |


### `LaserScan`

One 2D lidar sweep (spec section 5).

| Field | Type | Default | Notes |
|---|---|---|---|
| `ranges` | `ndarray` | *required* | (N,) float32 meters; inf/nan = no return |
| `angle_min` | `float` | *required* |  |
| `angle_max` | `float` | *required* |  |
| `angle_increment` | `float` | *required* |  |
| `stamp` | `float` | *required* |  |
| `frame` | `str` | *required* |  |
| `range_min` | `float` | `0.0` |  |
| `range_max` | `float` | `inf` |  |
| `intensities` | `ndarray | None` | `None` |  |


### `Observation`

Experimental synchronized-policy bundle; no stable acquisition producer.

Not exported from the SDK root. Use Measurement for produced provenance.

| Field | Type | Default |
|---|---|---|
| `stamp` | `float` | *required* |
| `frames` | `dict[str, ImageFrame]` | *required* |
| `joint_positions` | `dict[str, float]` | *required* |
| `joint_velocities` | `dict[str, float]` | *required* |
| `joint_efforts` | `dict[str, float]` | *required* |
| `ee_poses` | `dict[str, Any]` | *required* |
| `gripper_widths` | `dict[str, float]` | *required* |


### `OccupancyGrid`

A 2D grid map, from SLAM or loaded from disk (Phase 5 verbs).

``data`` follows the ROS convention: -1 unknown, 0 free, 100 occupied. Coordinate queries take world meters in the map frame.

| Field | Type | Default | Notes |
|---|---|---|---|
| `width` | `int` | *required* | cells |
| `height` | `int` | *required* | cells |
| `resolution` | `float` | *required* | meters per cell |
| `origin` | `Any` | *required* | Pose of cell (0, 0) in the world |
| `data` | `ndarray` | *required* | (H, W) int8 |

- `cost_at(x: float, y: float) -> int` — Occupancy value at world (x, y): -1/0..100.
- `is_free(x: float, y: float) -> bool` — True when the cell is known and unoccupied.
- `to_numpy() -> np.ndarray` — The raw (H, W) int8 grid.


### `PointCloud`

A voxel-downsampled point cloud from ``camera.get_cloud(voxel=)``.

| Field | Type | Default | Notes |
|---|---|---|---|
| `points` | `ndarray` | *required* | (N, 3) float32, in ``frame`` |
| `colors` | `ndarray | None` | *required* | (N, 3) uint8, or None without aligned RGB |
| `stamp` | `float` | *required* |  |
| `frame` | `str` | *required* |  |
| `intensities` | `ndarray | None` | `None` |  |


## Profile schema

The YAML a robot ships to describe itself. `RobotProfile` is the top level; every other class here is one of its sections. A section being present is what declares the capability (see `KNOWN_CAPABILITIES`).

**Capabilities:** `mobility`, `groups`, `sensors`, `actions`, `cameras`, `lidar`, `range_sensors`, `imu`, `force_torque`, `environment`, `slam`, `exploration`, `teleop`, `control`, `watches`, `places`, `audit`, `status`, `power`


### `ActionConfig`

One thing a vendor lets you command (spec: actions).

The write half of the escape hatch: an LED, a horn, a fan, an outrigger. Published on request, audited like any other command, and rate-limited because an arbitrary topic is otherwise a hole straight through to the robot's DDS.

| Field | Type | Default | Notes |
|---|---|---|---|
| `topic` | `str` | *required* |  |
| `type` | `str` | *required* |  |
| `is_motion` | `bool` | *required* |  |
| `motion_resource` | `str | None` | `None` | mobility or group:<declared name> |
| `stop_arguments` | `dict[str, Any] | None` | `None` |  |
| `max_rate` | `float` | `10.0` |  |
| `parameters` | `dict[str, ActionParameter]` | `{}` |  |


### `ActionParameter`

One argument a vendor action accepts.

| Field | Type | Default |
|---|---|---|
| `type` | `'float' | 'int' | 'bool' | 'string'` | `'float'` |
| `min` | `float | None` | `None` |
| `max` | `float | None` | `None` |
| `default` | `Any` | `None` |


### `AuditConfig`

Base for profile sections: frozen, extra keys allowed (config for future phases rides along without schema churn) — except a file path to a description, which is never allowed anywhere.

| Field | Type | Default |
|---|---|---|
| `retention_mb` | `int` | `200` |
| `dir` | `str | None` | `None` |


### `CameraConfig`

Base for profile sections: frozen, extra keys allowed (config for future phases rides along without schema churn) — except a file path to a description, which is never allowed anywhere.

| Field | Type | Default |
|---|---|---|
| `rgb` | `str | None` | `None` |
| `depth` | `str | None` | `None` |
| `info` | `str | None` | `None` |
| `cloud` | `str | None` | `None` |
| `frame` | `str | None` | `None` |
| `optical` | `bool` | `False` |
| `frame_ttl` | `float` | `30.0` |
| `sync_tolerance` | `float | None` | `None` |
| `max_stream_fps` | `float` | `15.0` |


### `EnvironmentConfig`

Base for profile sections: frozen, extra keys allowed (config for future phases rides along without schema churn) — except a file path to a description, which is never allowed anywhere.

| Field | Type | Default | Notes |
|---|---|---|---|
| `temperature` | `str | None` | `None` | sensor_msgs/Temperature topic |
| `pressure` | `str | None` | `None` | sensor_msgs/FluidPressure topic |
| `illuminance` | `str | None` | `None` | sensor_msgs/Illuminance topic |


### `ExplorationConfig`

Base for profile sections: frozen, extra keys allowed (config for future phases rides along without schema churn) — except a file path to a description, which is never allowed anywhere.

| Field | Type | Default |
|---|---|---|
| `backend` | `str` | `'explore_lite'` |
| `strategy` | `'nearest' | 'largest' | 'balanced'` | `'balanced'` |


### `ForceTorqueConfig`

One arm's wrist force/torque sensor (WrenchStamped topic).

| Field | Type | Default |
|---|---|---|
| `topic` | `str` | *required* |


### `FramesConfig`

TF frame names this robot uses; aliases @map/@odom/@base resolve here.

| Field | Type | Default |
|---|---|---|
| `map` | `str` | *required* |
| `odom` | `str` | *required* |
| `base` | `str` | *required* |


### `GroupConfig`

One actuated structure: an arm, a pan/tilt head, a torso lift, a leg.

Replaces v0.4's ``manipulation.arms.<name>`` and ``joint_groups.<name>`` (spec 8.6) - the split is gone, an arm is just a group that declares a tip and a tool.

Porting a group needs three things: ``kind``, ``controllers.trajectory``, and - if you want IK - ``tip``. Everything else is derived: joint names come from the controller's claimed interfaces, limits and geometry come from the description.

No ``named_poses`` (decision 5, 2026-08-23): joint vectors in YAML are not portable, and a mission that wants "home" writes the vector where it can be seen.

| Field | Type | Default |
|---|---|---|
| `kind` | `GroupKind` | *required* |
| `tip` | `str | None` | `None` |
| `dof_mask` | `tuple[int, ...] | None` | `None` |
| `controllers` | `dict[Controller, str]` | *required* |
| `joints` | `tuple[str, ...]` | `()` |
| `tool` | `ToolConfig | None` | `None` |
| `workspace` | `WorkspaceConfig | None` | `None` |
| `pointing_frame` | `str | None` | `None` |
| `description_topic` | `str | None` | `None` |
| `goal_tolerance` | `float` | `0.05` |
| `goal_tolerances` | `dict[str, float]` | `{}` |
| `feedback_max_age` | `float` | `0.5` |
| `stop_verification` | `JointStopPolicy | None` | `None` |


### `ImuConfig`

Base for profile sections: frozen, extra keys allowed (config for future phases rides along without schema churn) — except a file path to a description, which is never allowed anywhere.

| Field | Type | Default |
|---|---|---|
| `topic` | `str` | *required* |
| `mag_topic` | `str | None` | `None` |


### `JointStopPolicy`

Integrator-reviewed measured-rest thresholds, never morphology guesses.

Angular and linear limits use distinct units. Missing limits cannot verify joints of that type. Dwell and timeout are explicit deployment policy.

| Field | Type | Default |
|---|---|---|
| `angular_velocity_limit` | `float | None` | `None` |
| `linear_velocity_limit` | `float | None` | `None` |
| `dwell` | `float` | *required* |
| `timeout` | `float` | *required* |


### `LeasePolicy`

Explicit integrator timing policy; no implied hard real-time guarantee.

| Field | Type | Default |
|---|---|---|
| `lifetime` | `float` | *required* |
| `renew_interval` | `float` | *required* |


### `LidarConfig`

Base for profile sections: frozen, extra keys allowed (config for future phases rides along without schema churn) — except a file path to a description, which is never allowed anywhere.

| Field | Type | Default |
|---|---|---|
| `scan` | `str` | *required* |


### `MobilityConfig`

Base for profile sections: frozen, extra keys allowed (config for future phases rides along without schema churn) — except a file path to a description, which is never allowed anywhere.

| Field | Type | Default | Notes |
|---|---|---|---|
| `locomotion` | `Locomotion` | *required* |  |
| `cmd_vel` | `str | None` | `None` |  |
| `cmd_vel_stamped` | `bool` | `True` |  |
| `odom_topic` | `str | None` | `None` |  |
| `nav2_navigate_to_pose` | `str | None` | `None` |  |
| `nav2_follow_waypoints` | `str | None` | `None` |  |
| `max_linear` | `float | None` | `None` | m/s, server-side safety clamp |
| `max_angular` | `float | None` | `None` | rad/s |
| `arrival_position_tolerance` | `float` | `0.1` | m, full 3D |
| `arrival_orientation_tolerance` | `float` | `0.1` | rad |


### `PowerConfig`

Power management: battery + switchable power lanes.

``battery_topic`` carries a ``sensor_msgs/BatteryState`` (voltage, level, current, charging state). ``source_topic`` carries ``std_msgs/Bool`` (True = running on battery, False = external power).

| Field | Type | Default |
|---|---|---|
| `battery_topic` | `str | None` | `None` |
| `source_topic` | `str | None` | `None` |
| `lanes` | `dict[str, PowerLaneConfig]` | `{}` |


### `PowerLaneConfig`

One switchable power lane (MOSFET output on the bridge board).

The firmware publishes ``std_msgs/Float64MultiArray`` on ``topic`` with three elements: ``[on (0|1), voltage (V), current (A)]``. The agent commands on/off via ``std_msgs/Bool`` on ``command_topic``.

| Field | Type | Default |
|---|---|---|
| `topic` | `str` | *required* |
| `command_topic` | `str` | *required* |


### `RangeSensorConfig`

One single-beam ranger (ultrasonic/IR/ToF point sensor).

| Field | Type | Default |
|---|---|---|
| `topic` | `str` | *required* |
| `role` | `'cliff' | 'bumper' | 'proximity'` | `'proximity'` |


### `RobotProfile`

A fully parsed profile. ``capabilities``/``instances`` feed the handshake capability report.

| Field | Type | Default | Notes |
|---|---|---|---|
| `name` | `str` | *required* |  |
| `model` | `str` | *required* |  |
| `version` | `str` | `'0'` |  |
| `protocol` | `int` | *required* |  |
| `frames` | `FramesConfig` | *required* |  |
| `safety` | `SafetyConfig` | `SafetyConfig(restart_recovery_controllers=None, lease=None, stop_verification=None, feedback_max_age=0.5, stop_on_disconnect=False, confirm_required=())` |  |
| `joint_states` | `str` | `'/joint_states'` |  |
| `mobility` | `MobilityConfig | None` | `None` |  |
| `groups` | `dict[str, GroupConfig] | None` | `None` |  |
| `sensors` | `dict[str, dict[str, SensorSignal]] | None` | `None` |  |
| `actions` | `dict[str, ActionConfig] | None` | `None` |  |
| `cameras` | `dict[str, CameraConfig] | None` | `None` |  |
| `lidar` | `LidarConfig | None` | `None` |  |
| `range_sensors` | `dict[str, RangeSensorConfig] | None` | `None` |  |
| `imu` | `ImuConfig | None` | `None` |  |
| `force_torque` | `dict[str, ForceTorqueConfig] | None` | `None` |  |
| `environment` | `EnvironmentConfig | None` | `None` |  |
| `slam` | `SlamConfig | None` | `None` |  |
| `exploration` | `ExplorationConfig | None` | `None` |  |
| `teleop` | `TeleopConfig | None` | `None` |  |
| `control` | `_Section | None` | `None` | schema lands post-v1.0 |
| `watches` | `_Section | None` | `None` |  |
| `places` | `_Section | None` | `None` |  |
| `audit` | `AuditConfig | None` | `None` |  |
| `status` | `StatusConfig | None` | `None` |  |
| `power` | `PowerConfig | None` | `None` |  |


### `SafetyConfig`

Base for profile sections: frozen, extra keys allowed (config for future phases rides along without schema churn) — except a file path to a description, which is never allowed anywhere.

| Field | Type | Default |
|---|---|---|
| `restart_recovery_controllers` | `dict[str, 'active' | 'inactive' | 'unconfigured'] | None` | `None` |
| `lease` | `LeasePolicy | None` | `None` |
| `stop_verification` | `JointStopPolicy | None` | `None` |
| `feedback_max_age` | `float` | `0.5` |
| `stop_on_disconnect` | `bool` | `False` |
| `confirm_required` | `tuple[str, ...]` | `()` |


### `SensorSignal`

One scalar a vendor publishes, bound to a name (spec: sensors).

The escape hatch for anything on a topic that the capability registry does not already cover — servo temperature, bus voltage, a load cell. Read-only by construction: a signal is subscribed, never written.

``type`` is a ROS type name resolved at runtime, so the agent needs no per-vendor code. ``field`` digs into a compound message; leave it out for the single-field messages most telemetry uses.

| Field | Type | Default |
|---|---|---|
| `topic` | `str` | *required* |
| `type` | `str` | `'std_msgs/Float32'` |
| `field` | `str | None` | `None` |
| `units` | `str | None` | `None` |


### `SlamConfig`

Base for profile sections: frozen, extra keys allowed (config for future phases rides along without schema churn) — except a file path to a description, which is never allowed anywhere.

| Field | Type | Default |
|---|---|---|
| `backend` | `str` | `'slam_toolbox'` |
| `map_dir` | `str | None` | `None` |


### `StatusConfig`

Base for profile sections: frozen, extra keys allowed (config for future phases rides along without schema churn) — except a file path to a description, which is never allowed anywhere.

| Field | Type | Default | Notes |
|---|---|---|---|
| `battery` | `str | None` | `None` | BatteryState topic; None = no battery data |
| `diagnostics` | `str | None` | `None` | DiagnosticArray topic |


### `TeleopConfig`

Base for profile sections: frozen, extra keys allowed (config for future phases rides along without schema churn) — except a file path to a description, which is never allowed anywhere.

| Field | Type | Default | Notes |
|---|---|---|---|
| `max_watchdog` | `float` | `1.0` | seconds; clients may request shorter, never longer |
| `arm_jog` | `str | None` | `None` |  |


### `ToolConfig`

The end-effector fitted to a group (spec 8.3).

``type`` is free text describing the mechanism ("jaws", "parallel", "suction"); the agent does not branch on it, it is reported so a mission can. ``joint`` is the single actuated joint the tool controller commands.

``joint`` is validated against the ``<ros2_control>`` declarations as well as the kinematic tree, because a tool joint need only be in the former: a gripper can be commandable without its fingers being modelled, and that is a robot's choice to make, not a reason to refuse to boot.

| Field | Type | Default | Notes |
|---|---|---|---|
| `type` | `str` | *required* |  |
| `joint` | `str | None` | `None` |  |
| `open` | `float | None` | `None` | meters (or radians), fully open |
| `closed` | `float | None` | `None` | meters (or radians), fully closed |


### `WorkspaceConfig`

Server-side reachability box for a group, in the robot's base frame.

A non-bypassable clamp (spec 21), not a hint: a target outside the box is rejected before any controller sees it. ``None`` means unclamped, which is the right default only for a group that cannot reach anything dangerous.

| Field | Type | Default |
|---|---|---|
| `box` | `dict[str, tuple[float, float]] | None` | `None` |
