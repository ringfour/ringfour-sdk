# Ringfour Python API

The sync SDK is `from ringfour import Robot`; the async SDK is
`from ringfour.aio import Robot`. Both have the same effects and lifetime rules.
Properties read local metadata/caches. Remote reads use `get_*()`; remote actions
are methods. The agent owns ROS integration, admission, stopping and cleanup.

This reference describes the A–C implementation. See [milestone B](../../ringfour-testing/docs/milestone-b.md), [milestone C](../../ringfour-testing/docs/milestone-c.md) and [the implementation log](../../ringfour-testing/HARDENING_LOG.md) for contracts and acceptance boundaries. Platform D validation is separate from robot integration qualification. Examples, mission recipes and the course live in [missions](../../missions/README.md).

## Connect and discover

```python
from ringfour import Robot

with Robot("ws://127.0.0.1:10101") as robot:
    print(robot.name, robot.profile.model, robot.capabilities)
    print(robot.groups, robot.cameras)
```

The default connection tries a local Unix socket then localhost TCP. Network
motion requires an explicitly trusted deployment; ordinary TCP is observation
only. Singular `robot.arm`, `robot.camera`, etc. raise `AmbiguousCapability` when
multiple instances exist. Optional capabilities raise `CapabilityNotSupported`.
`robot.groups` includes arm, pointing and linear groups; no morphology is inferred.

Local metadata includes `name`, `profile`, `capabilities`, `description`,
`description_sha`, `group_infos`, `group_info(name)`, resource mappings and
`signals`. `refresh_description(group=None)` is explicit. Named group descriptions
have separate caches; refresh the group when its model differs from the composed
robot. Old configurations and sessions cannot command a changed model.

## Fresh observations

`get_pose(frame='@map', max_age=.5, timeout=1.)` returns a usable measured Pose or
raises `StateUnavailable`/`StateStale`. `get_pose_measurement(...)` returns its
`Measurement[Pose]` envelope. With `require_valid=False`, inspect unavailable,
invalid and stale observations without treating them as usable state.

`group.get_joint_measurement(max_age=.5, timeout=1., require_valid=True)` returns
`Measurement[tuple[JointValue,...]]` with every group joint in native units.
Observed values have no command bounds, so out-of-bound feedback remains visible.
`get_signal(discovered_signal, ...)` produces `Measurement[float]`.

Measurements contain immutable value, robot/model identity, source timestamp,
clock domain, agent monotonic receipt time, separately computed source/receive
ages, validity and reason. Unknown source freshness is explicit. Newly received
old data remains stale. Client, agent and ROS clock epochs are never subtracted
from one another. Returned ages describe acquisition, not a permanently fresh
object: obtain another observation before a later decision.

Other explicit reads include `get_mobility_state()`, `get_velocity()`,
`get_is_moving()`, `get_distance_traveled()`, `get_battery()`,
`get_estop_engaged()` and `get_map()`. These legacy state readers report latest
values; use the measurement producers above when a freshness budget is required.
Joint reads include `get_joints()`, `get_joint_positions()`,
`get_kinematic_positions()` and `get_kinematic_snapshot(stamp=None)`.

Camera methods include `get_info()`, `get_intrinsics()`, `get_frame()`,
`get_depth()`, `get_synced()`, `deproject(pixel, frame, in_frame=...)`, `cloud()`
and `stream()`. Retained shared-memory arrays are read-only mappings; agent
segments are written once and unlinked at expiry, never overwritten. Retained
bytes remain valid after unlink; later frame-handle geometry requests can raise
`FrameExpired`. Copy with `frame.image.copy()` when writable owned bytes are
needed. A camera frame carries its capture stamp; it is not a fabricated fresh
pose observation or a synchronized learned-policy bundle.

## Accepted finite operations

| Resource | Eager submission | Blocking convenience | Verified result |
|---|---|---|---|
| Navigation | `robot.start_navigation(Pose)` | `navigate_to(Pose, timeout=...)` | `NavigationResult` |
| Finite route | `robot.start_route(poses)` | `run_route(poses, timeout=...)` | `RouteResult` |
| Joint trajectory | `control.start_trajectory(JointTrajectory)` | `control.run(trajectory, timeout=...)` | `TrajectoryResult` |
| Tool positioning | `tool.start_position(value, execution_timeout=30.)` | `tool.set(value, timeout=...)`; `open()` / `close()` | `ToolResult` |

All producers return accepted work before any iteration. Navigation accepts an
explicit framed Pose; resolve saved names through `robot.places.get(name)` first.
Every route waypoint needs measured arrival and rest before advancing. Routes
hold one mobility acquisition across their finite list; repeated patrol logic is
ordinary mission code.

```python
with robot.start_navigation(reviewed_goal) as op:
    for update in op.updates():
        print(update.state, update.progress)
    result = op.result()
    print(result.final_pose, result.stop)
```

The generic `Operation[TResult,TProgress]` has:

- `id`, immutable local `snapshot`, and authoritative `get_status()`.
- `updates()`, a bounded observation stream. Closing/breaking it never cancels.
- `result(timeout=None)`: typed result or typed failure/cancellation.
- `cancel()`: `CancelReceipt`, acknowledging the request rather than physical rest.
- `close()` and scope exit: idempotent bounded cancellation/cleanup of unfinished work.

Tool `execution_timeout` bounds reaching the target, independently of the profile stop-verification timeout. The convenience `timeout` bounds the caller's wait and performs cancellation/cleanup on expiry. Neither changes measured-rest tolerances.

States are accepted, running, cancelling, succeeded, failed and cancelled.
Terminal outcomes do not change. Losing a connection means lost knowledge, not a
made-up terminal result. `snapshot.observed_age` retains the agent report age plus
elapsed client receipt age; querying a retained report cannot refresh old progress.
Progress can be unavailable; trajectory fractions are explicitly elapsed-time
estimates, not measured physical completion.

`op.result(timeout=...)` raises `WaitTimeout` and leaves work running. Convenience
action deadlines cancel/join finite cleanup before `OperationTimeout`, retaining
operation ID and cleanup evidence. Async observer cancellation stops observation;
async convenience cancellation joins cleanup before propagating. Scope body and
cleanup failures are retained together through `error.cleanup_error`/exception
notes. The agent advertises a finite operation cleanup budget, currently 60 s.

Navigation results include framed measured pose, position/orientation residuals
and tolerances, plus `StopReport`. Trajectory results include model-bound measured
configuration and every joint's native residual/tolerance. Tool positioning does
not prove contact, grasp or an object being held. `get_state()` reports the
separate availability-aware holding inference.

## Joint configurations and control

```python
from ringfour import JointTrajectory
from ringfour import kinematics as kin

arm = robot.arm
approach = kin.ik(arm, reviewed_approach_pose)
contact = kin.ik(arm, reviewed_contact_pose, seed=approach)
trajectory = JointTrajectory.from_waypoints([approach, contact], step_duration=2.)
with arm.control() as control:
    result = control.run(trajectory, timeout=15.)
```

`group.configuration({joint_name: value, ...})` binds exact named coverage,
robot/group/model identity, model-derived units and bounds. Configurations and
trajectories are immutable. Units are radians for revolute/continuous joints and
metres for prismatic joints. Finite increasing `times=` support unequal intervals;
`from_waypoints()` supplies equal step durations. Mixed models, units, coverage,
booleans, nonfinite values and out-of-bound commands reject before dispatch.
The agent repeats binding/coverage/bounds checks against the current session/model.
Endpoint bounds do not establish collision-free paths or interpolation speed and
acceleration limits.

`kin.fk`, `ik`, `jacobian`, `limits`, `links`, `reach`, `reachable`,
`manipulability`, `point_at`, `snapshot`, `collision_model`, `collisions`,
`in_collision` and `reach_map` remain robotics tools. Calculations do not submit
motion. IK returns `JointConfiguration`; raw mathematical vectors can be bound
explicitly with `group.configuration()`. Unavailable external joint state is an
error, never a guessed zero.

`control()` is lazy; `open()` acquires manually, or use `with`/`async with`.
An optional `Controller` mode selects trajectory/stream/velocity robotics
behavior; actual backend names are integration concerns. `hold()`, `stop()` and
`close()` remain scoped. `teleop(watchdog=...)` is also lazy and has `open()` /
`close()`; commands must be refreshed explicitly within the granted watchdog.
Disconnect stops/drains owned work and restores controllers; failures quarantine
motion. Shared policy requires one motion writer and rejects overlapping physical
resources even for the same owner.

## Persistent guards and observational watches

```python
condition = robot.signals.battery.level.below(10., max_age=1.)
with robot.guard(condition) as guard:
    robot.navigate_to(reviewed_goal, timeout=60.)
```

Signals are local discovered references with units metadata. A runtime joint name
can be indexed explicitly, e.g. `robot.signals.arms[group.name].effort[joint]`.
A guard currently covers robot-wide motion conservatively. Entry confirms fresh
safe evidence before the body. Threshold crossing, stale/invalid/unavailable
required data trips a latch and starts verified stopping/draining. `get_status()`
reports condition, the retained observation, latch reason and cleanup status.
`evaluation_age` is time since the last agent evaluation; status queries do not
reacquire or refresh that observation. Owner-only
`reset()` requires fresh safe evidence and never resumes old work. `close()` /
scope exit drains work before removing only that guard's inhibit. Operator e-stop,
other guards and quarantine remain intact. Failed removal retains the inhibit.
Disconnected-owner guards are removed only after confirmed cleanup.

Named watches remain observational: `robot.watches.battery_level(...)`,
`forward_speed(...)`, `lateral_speed(...)`, `turn_rate(...)`,
`imu_angular_rate(axis, ...)`, range/environment helpers and group joint helpers.
`get_triggered()`, `get_value()`, etc. poll explicitly. A named watch does not
establish source freshness. Effectful `stop=True` and public dotted-string watch
entry are retired; use `guard()` for motion enforcement. Header-less vendor
signals cannot provide source-fresh guards until their adapter supplies provenance.

## Mapping, power and advanced integration

`robot.slam.get_state()` / `get_mode()` / `get_saved_maps()` /
`get_localization_quality()` / `get_is_lost()` read explicitly. `start`, `stop`,
`save`, `load`, `localize`, `relocalize` and `wait_for_localization` retain their
mapping meanings. Public map metadata has name, creation time, resolution and
2D dimensions; backend format provenance stays private for loading compatibility.
Places provide explicit `get`, `save`, `list` and `delete` operations.

`robot.power.get_battery()` and `get_is_on_battery()` have identical sync/async
value contracts. Missing battery/source remains None. Power lane `get_state()`,
`on()` and `off()` are explicit methods. Vendor sensors use `get_<signal>()` or
`get_reading(name)`; attribute lookup itself does not perform RPC. Vendor actions
return a limited `DispatchReceipt`; publication never claims physical completion.

Integration-only data lives under `robot.integration.controllers`,
`control.integration`, `group.integration.control(raw=...)` and
`description.integration['hardware_joints']`. Raw control retains admission,
ownership and measured-stop requirements. Transport extensions live in
`ringfour.payloads`/`ringfour.enums`, not the ordinary root vocabulary.

## Errors and migration

Produced errors include `CapabilityNotSupported`, `AmbiguousCapability`,
`PermissionDenied`, `SafetyViolation`, `ResourceBusy`, `StateUnavailable`,
`StateStale`, `ModelChanged`, `NavigationFailed`, `ExecutionFailed`,
`CompletionUnverified`, `StopUnverified`, `CleanupFailed`, `OperationCancelled`,
`WaitTimeout`, `OperationTimeout`, `ConnectionFailed`, `ConnectionLost`,
`FrameMismatch`, `FrameExpired`, `Unreachable` and `UnknownPlace`.
`.details` retains wire evidence and cleanup context. Software stopping evidence
is separate from physical deployment commissioning.

Removed unreleased surfaces: `navigate_to_async`, `navigate`, `patrol`,
`control.follow`, raw positional `control.run(..., dt=...)`, remote properties,
`joint_groups`, `gripper`, dotted `watch`, root Task/Transport/speculative
Observation and unproduced planning/grasp error promises. Update agent and both
SDKs together; old competing aliases are not retained.

`GeoPose` remains a standalone geographic value only; no geographic navigation is
advertised. The old synchronized-policy `data.Observation` is experimental and
has no stable acquisition producer. Historical NavStatus/TrajStatus parsers are
internal artifact helpers, not current operation results. See the generated
[type reference](types-reference.md) for current public wire/value fields.


## Payload transport across hosts

```python
from ringfour import Robot
from ringfour.enums import Transport

with Robot("ws://127.0.0.1:10101", transport=Transport.NET) as robot:
    frame = robot.camera.get_frame()  # raw numeric pixels; no shared filesystem
```

The same `transport=` keyword is accepted by `await Robot.connect(...)` in the
async API. With a capable agent, TCP defaults to NET and Unix sockets to SHM.
NET carries all array payloads in binary frames; commands stay JSON-RPC.
The native Mac SDK therefore needs no ROS installation or Linux /dev/shm.
Explicit SHM requires a shared Linux IPC namespace. Transport selection grants
no motion permissions; default TCP remains observation-only.


## Submission reconciliation and retained results

Every mutating call to a hardened agent carries a client-generated command ID.
`SubmissionUnknown.details['command_id']` identifies an uncertain acceptance;
query `robot.commands.get(id)` before retrying. `robot.operations.get(operation_id)`
returns retained execution evidence, including after authenticated reconnection.
These sync APIs have identical async counterparts. Neither replays or resumes work.
`commands.new(valid_for=...)` and `commands.retry(id, method, params)` support
explicit protocol integrations. Conflict, expiry and changed epoch are typed
errors. A completed command reply is not proof of completed motion: inspect its
operation result. See [milestone C](../../ringfour-testing/docs/milestone-c.md) for the clock, retention,
principal, crash and recovery contracts.
