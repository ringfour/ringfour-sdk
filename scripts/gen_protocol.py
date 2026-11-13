#!/usr/bin/env python3
"""Generate protocol.json from the pydantic models (single source of truth).

Run from anywhere:  python3 ringfour-sdk/scripts/gen_protocol.py
Writes ringfour-sdk/protocol.json. Regenerate whenever a model or method changes;
the file is committed so diffs show wire-schema changes in review.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

ENGINE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ENGINE_DIR))

from ringfour import enums, models  # noqa: E402
from ringfour.version import PROTOCOL_VERSION  # noqa: E402

# The wire methods. Grows phase by phase; keep alphabetical.
# params/result name a model in ringfour.models or describe inline fields.
METHODS = {
    "commands.get": {
        "doc": "Query retained acceptance without replaying a mutating request. Mutations carry top-level command_id (epoch.expiry_milliseconds.uuid).",
        "params": {"command_id": "str"}, "result": "CommandRecord",
    },
    "operations.get": {
        "doc": "Query retained terminal evidence or explicit unknown outcome after restart; authenticated principal ownership is required across connections.",
        "params": {"operation_id": "str"}, "result": "OperationRecord",
    },
    "lease.renew": {
        "doc": "Renew this connection's finite authority; never refreshes drive intent.",
        "params": {"lease_id": "str", "epoch": "str"},
        "result": {"lease_id": "str", "epoch": "str", "lifetime": "float"},
    },
    "payload.set_transport": {
        "doc": "Select the connection-local array transport advertised by welcome.",
        "params": {"transport": "shm | net"},
        "result": {"transport": "shm | net"},
    },
    "group.start_trajectory": {
        "doc":"Submit a model-bound trajectory; task updates carry verified endpoint/stopping evidence.",
        "params":{"group":"str","session_id":"str","trajectory":"JointTrajectory"},
        "result":{"task_id":"str"},
    },
    "action.invoke": {
        "doc": "Invoke a validated profile action. Returns an explicit unverified dispatch receipt; topic/payload details are integration diagnostics.",
        "params": {"action": "str", "arguments": "dict"}, "result": "DispatchReceipt",
    },
    "audit.query": {
        "doc": "Query the agent's in-memory audit ring buffer. All filters "
               "optional; newest events last.",
        "params": {"since": "str|null, ISO 8601",
                   "kind": "AuditKind|null",
                   "task_id": "str|null", "limit": "int, default 500"},
        "result": {"events": "list[AuditEvent]"},
    },
    "audit.subscribe": {
        "doc": "Start receiving audit.event notifications on this "
               "connection.",
        "params": None,
        "result": {},
    },
    "audit.unsubscribe": {
        "doc": "Stop receiving audit.event notifications.",
        "params": None,
        "result": {},
    },
    "arm.joint_states": {
        "doc": "Controller-level joint state of one arm (pre-MoveIt). "
               "Limits parsed from /robot_description (null for "
               "continuous joints / before the description arrives).",
        "params": {"arm": "str, profile group name",
                   "include_external": "bool, optional; default false",
                   "stamp": "optional float ROS seconds; select a recorded snapshot"},
        "result": {"joints": "{name: {position, velocity, effort, "
                             "limits: [lo, hi]|null}}",
                   "positions": "optional {joint: position}, measured robot snapshot when include_external=true",
                   "stamp": "optional float ROS seconds | null, common timestamp of snapshot"},
    },
    "arm.wrench": {
        "doc": "Latest force/torque reading of one arm's wrist sensor "
               "(profile force_torque.<arm>.topic).",
        "params": {"arm": "str"},
        "result": "ForceTorque",
    },
    "env.read": {
        "doc": "Latest environmental reading (profile environment.*).",
        "params": {"kind": "'temperature'|'pressure'|'illuminance'"},
        "result": "Temperature | FluidPressure | Illuminance",
    },
    "imu.mag": {
        "doc": "Latest magnetometer reading (profile imu.mag_topic).",
        "params": None,
        "result": "MagReading",
    },
    "imu.read": {
        "doc": "Latest IMU reading (profile imu.topic).",
        "params": None,
        "result": "ImuReading",
    },
    "range.read": {
        "doc": "Latest reading of one single-beam ranger (profile "
               "range_sensors.<name>).",
        "params": {"name": "str"},
        "result": "RangeReading",
    },
    "status.diagnostics": {
        "doc": "System health snapshot (profile status.diagnostics).",
        "params": None,
        "result": "DiagnosticsReport",
    },
    "camera.deproject": {
        "doc": "Pixel -> 3D point using the depth + TF snapshotted with "
               "THAT frame (FrameExpired past frame_ttl). in_frame must "
               "be the camera frame or a capture-time snapshot "
               "(@map/@odom/@base).",
        "params": {"frame_id": "str, from a get_* result",
                   "pixel": "[u, v]", "in_frame": "str|absent"},
        "result": {"point": "Point"},
    },
    "camera.get_cloud": {
        "doc": "Voxel-downsampled point cloud from the latest depth "
               "(+rgb colors when aligned). voxel is mandatory: it is "
               "the size budget of the call.",
        "params": {"camera": "str", "voxel": "float m, > 0"},
        "result": {"stamp": "float", "frame": "str", "count": "int",
                   "points": "PayloadRef (N,3) float32",
                   "colors": "PayloadRef (N,3) uint8 | null"},
    },
    "camera.get_depth": {
        "doc": "Latest depth frame, float32 meters, via PayloadRef.",
        "params": {"camera": "str"},
        "result": {"frame_id": "str", "camera": "str", "stamp": "float",
                   "payload": "PayloadRef"},
    },
    "camera.get_frame": {
        "doc": "Latest RGB frame as a PayloadRef (raw pixels over shared "
               "memory in v1 — local execution; compressed transports "
               "are a future PayloadRef.transport). frame_id stays valid "
               "for deprojection for frame_ttl seconds.",
        "params": {"camera": "str"},
        "result": {"frame_id": "str", "camera": "str", "stamp": "float",
                   "payload": "PayloadRef"},
    },
    "camera.get_synced": {
        "doc": "RGB-D pair with closest stamps (within the agent's sync "
               "tolerance); both halves share one frame_id.",
        "params": {"camera": "str"},
        "result": {"frame": "as camera.get_frame",
                   "depth": "as camera.get_depth"},
    },
    "camera.info": {
        "doc": "Camera metadata: intrinsics (null without a camera_info "
               "topic), depth availability, frame_ttl, TF frame.",
        "params": {"camera": "str"},
        "result": {"camera": "str", "has_rgb": "bool", "has_depth": "bool",
                   "frame_ttl": "float s", "max_stream_fps": "float",
                   "intrinsics": "CameraIntrinsics|null", "frame": "str"},
    },
    "camera.stream_start": {
        "doc": "Start a server-pushed frame stream (stream.data "
               "notifications). fps is clamped to the profile's "
               "max_stream_fps.",
        "params": {"camera": "str", "fps": "float, default 5"},
        "result": {"stream_id": "str", "fps": "float, granted"},
    },
    "camera.stream_stop": {
        "doc": "Stop a frame stream started by this client.",
        "params": {"stream_id": "str"},
        "result": {},
    },
    "debug.run_task": {
        "doc": "Test-only: a cancellable timer task emitting progress "
               "updates. Exercises the task lifecycle until real "
               "long-running verbs exist.",
        "params": {"duration": "float s, default 1.0",
                   "ticks": "int, default 10",
                   "fail": "bool, fail on purpose"},
        "result": {"task_id": "str"},
    },
    "debug.send_payload": {
        "doc": "Test-only: agent sends a random binary payload of the "
               "requested size, returns its id and sha256. Exercises the "
               "binary channel until real image methods exist (Phase 4).",
        "params": {"size": "int, bytes to send"},
        "result": {"payload_id": "int", "sha256": "str, hex digest"},
    },
    "debug.set_estop": {
        "doc": "Simulated e-stop toggle (for profiles with no hardware "
               "e-stop topic). Engaged e-stop rejects all motion methods "
               "with SafetyViolation(reason='estop').",
        "params": {"engaged": "bool"},
        "result": {"engaged": "bool"},
    },
    "events.subscribe": {
        "doc": "Start receiving event notifications (estop changes, "
               "watch fires) on this connection.",
        "params": None,
        "result": {},
    },
    "events.unsubscribe": {
        "doc": "Stop receiving event notifications.",
        "params": None,
        "result": {},
    },
    "hello": {
        "doc": "Handshake. Must be the first call on a connection.",
        "params": "Hello",
        "result": "Welcome",
    },
    "lidar.scan": {
        "doc": "Latest 2D sweep; ranges travel as a PayloadRef "
               "(float32 meters).",
        "params": None,
        "result": {"stamp": "float", "frame": "str", "angle_min": "float",
                   "angle_max": "float", "angle_increment": "float",
                   "ranges": "PayloadRef (N,) float32"},
    },
    "lidar.stream_start": {
        "doc": "Start a server-pushed scan stream (stream.data "
               "notifications). hz is server-clamped.",
        "params": {"hz": "float, default 5"},
        "result": {"stream_id": "str", "hz": "float, granted"},
    },
    "lidar.stream_stop": {
        "doc": "Stop a scan stream started by this client.",
        "params": {"stream_id": "str"},
        "result": {},
    },
    "mobility.get_state": {
        "doc": "Pose (map frame when localized, else odom — pose.frame "
               "says which), velocity, is_moving, distance_traveled.",
        "params": None,
        "result": {"pose": "Pose", "velocity": "Twist",
                   "is_moving": "bool", "distance_traveled": "float m",
                   "stamp": "optional float seconds, odometry clock", "odometry_pose": "optional Pose in odometry frame"},
    },
    "mobility.navigate_to": {
        "doc": "Navigate to a goal via Nav2 (task). goal is a Pose or "
               "{place: name} (spec 17 sugar). The agent clears both "
               "costmaps first. task.update notifications carry feedback "
               "in data (distance_remaining, pose, ...). Failure: "
               "NavigationFailed with reason no_path|stuck|aborted.",
        "params": {"goal": "Pose | {place: str}"},
        "result": {"task_id": "str"},
    },
    "mobility.stop": {
        "doc": "Immediate halt: cancels running mobility tasks + the "
               "active Nav2 goal, zeroes velocity. No resume (Q3).",
        "params": None,
        "result": {"cancelled": "int, tasks cancelled"},
    },
    "places.delete": {
        "doc": "Delete a named place from the active map (UnknownPlace "
               "if absent).",
        "params": {"name": "str"},
        "result": {},
    },
    "places.get": {
        "doc": "A named place's Pose on the active map (UnknownPlace "
               "if absent — including places saved on OTHER maps).",
        "params": {"name": "str"},
        "result": {"pose": "Pose"},
    },
    "places.list": {
        "doc": "Place names on the active map.",
        "params": None,
        "result": {"names": "list[str]"},
    },
    "places.save": {
        "doc": "Save a named place on the active map; pose defaults to "
               "the robot's current pose. Persists alongside the map "
               "files (places.yaml, human-editable).",
        "params": {"name": "str", "pose": "Pose|absent"},
        "result": {"pose": "Pose, as saved"},
    },
    "slam.delete": {
        "doc": "Delete a saved map (refuses the active one).",
        "params": {"name": "str"},
        "result": {},
    },
    "slam.get_map": {
        "doc": "The current occupancy grid; data travels as a PayloadRef "
               "((H,W) int8, ROS convention: -1 unknown, 0 free, 100 "
               "occupied).",
        "params": None,
        "result": {"width": "int", "height": "int", "resolution": "float",
                   "origin": "Pose of cell (0,0)", "stamp": "float",
                   "payload": "PayloadRef"},
    },
    "slam.get_state": {
        "doc": "SLAM status in one call.",
        "params": None,
        "result": {"mode": "SlamMode",
                   "backend": "str", "active_map": "str|null",
                   "localization_quality": "float 0..1 (heuristic: "
                                           "map->odom stability)",
                   "is_lost": "bool", "saved_maps": "list[str]", "supports_preserve_graph": "bool, optional"},
    },
    "slam.load": {
        "doc": "Load a saved map's pose graph and continue mapping on "
               "top of it.",
        "params": {"name": "str"},
        "result": {"mode": "SlamMode", "active_map": "str"},
    },
    "slam.localize": {
        "doc": "Switch to localization mode against a saved map "
               "(default: the active one) at the given pose (default: "
               "current estimate).",
        "params": {"pose": "Pose|absent", "name": "str|absent"},
        "result": {"mode": "SlamMode", "active_map": "str"},
    },
    "slam.map_info": {
        "doc": "Metadata of a saved map.",
        "params": {"name": "str"},
        "result": {"info": "MapInfo"},
    },
    "slam.relocalize": {
        "doc": "Re-run localization at the current pose estimate (the "
               "is_lost recovery path).",
        "params": None,
        "result": {"mode": "SlamMode"},
    },
    "slam.save": {
        "doc": "Serialize the pose graph + export the 2D grid + write "
               "MapInfo under map_dir/<name>/. Scratch-map places "
               "migrate to the named map.",
        "params": {"name": "str"},
        "result": {"info": "MapInfo"},
    },
    "slam.start": {
        "doc": "Ensure mapping mode: unpause from idle, or reset to a "
               "fresh map from localization (modes are service-level, "
               "never process restarts). Set preserve_graph to resume "
               "the loaded graph without clearing it.",
        "params": {"preserve_graph": "bool, optional; default false"},
        "result": {"mode": "SlamMode"},
    },
    "slam.stop": {
        "doc": "Pause map building (idle); map + TF stay published.",
        "params": None,
        "result": {"mode": "SlamMode"},
    },
    "slam.wait_for_localization": {
        "doc": "Block until localization quality is healthy; "
               "TimeoutError otherwise.",
        "params": {"timeout": "float s, default 10"},
        "result": {"localization_quality": "float 0..1"},
    },
    "ping": {
        "doc": "Round-trip check; returns an empty object.",
        "params": None,
        "result": {},
    },
    "status.get": {
        "doc": "Battery (null without a battery topic) and e-stop state.",
        "params": None,
        "result": {"battery": "{level: float 0..100|null, "
                              "is_charging: bool|null} | null",
                   "estop": "bool"},
    },
    "task.status": {
        "doc": "Owner-only authoritative state, retained after completion for the handshake's advertised window.",
        "params": {"task_id": "str"}, "result": "TaskUpdate",
    },
    "group.control_start": {
        "doc": "Acquire counted control authority; controller changes are serialized and unwound on failure.",
        "params": {"group": "str", "role": "Controller|null", "raw": "str|null"},
        "result": {"session_id": "str", "controller": "str", "loaded": "list[str]", "deactivated": "list[str]"},
    },
    "group.control_stop": {
        "doc": "Cancel this group's work, restore controllers with readback, and release only this acquisition. Cleanup failure quarantines motion.",
        "params": {"group": "str", "session_id": "str"}, "result": {},
    },
    "group.follow": {
        "doc": "Submit finite bounded joint waypoints; require a current session and fresh complete finite arrival evidence. Single active trajectory adapter.",
        "params": {"group": "str", "session_id": "str", "waypoints": "list[list[float]]", "dt": "positive finite float seconds"},
        "result": {"task_id": "str"},
    },
    "group.hold": {
        "doc": "Command fresh finite measured positions in a current session. Reject while a trajectory occupies the adapter.",
        "params": {"group": "str", "session_id": "str"}, "result": {"positions": "dict[str,float]"},
    },
    "group.stop": {
        "doc": "Cancel only this group's accepted tasks; returns dispatch count, not verified physical-rest evidence.",
        "params": {"group": "str", "session_id": "str"}, "result": {"cancelled": "int"},
    },
    "group.tool_set": {
        "doc": "Submit a finite bounded tool position after admission. Dispatch receipt is not verified arrival or grasp evidence.",
        "params": {"group": "str", "position": "float"}, "result": {"position": "float"},
    },
    "group.tool_state": {
        "doc": "Read measured tool state and holding inference.",
        "params": {"group": "str"}, "result": "tool feedback object",
    },
    "task.cancel": {
        "doc": "Owner-only cancellation. Terminal cancellation is idempotent; unknown or expired IDs fail.",
        "params": {"task_id": "str"},
        "result": {},
    },
    "watch.start": {
        "doc": "Arm a bridge-side condition monitor over a dotted state "
               "path. Exactly one of above/below/outside. stop=true "
               "halts all motion when it fires (teleop sessions ended, "
               "tasks cancelled, zero twist), audit-logged, and survives "
               "owner disconnect for `lifetime` seconds.",
        "params": {"path": "str, e.g. battery.level",
                   "above": "float|absent", "below": "float|absent",
                   "outside": "[lo, hi]|absent",
                   "stop": "bool, default false",
                   "debounce": "float s the condition must hold, default 0",
                   "lifetime": "float s|absent (agent default 600)"},
        "result": {"watch_id": "str"},
    },
    "watch.poll": {
        "doc": "Read a watch's latched state (owner only).",
        "params": {"watch_id": "str"},
        "result": {"triggered": "bool, latched", "value": "float|null",
                   "triggered_at": "float|null, wall stamp of first fire",
                   "events": "[[stamp, value], ...] debounced edges"},
    },
    "watch.stop": {
        "doc": "Disarm and remove a watch (owner only).",
        "params": {"watch_id": "str"},
        "result": {},
    },
    "teleop.drive": {
        "doc": "One velocity command, clamped by profile limits. Must "
               "recur within the watchdog period or the agent halts the "
               "robot. lateral needs omni locomotion.",
        "params": {"linear": "float m/s", "angular": "float rad/s",
                   "lateral": "float m/s, omni only"},
        "result": {},
    },
    "teleop.end": {
        "doc": "Zero velocity, stop the watchdog, release the motion lock.",
        "params": None,
        "result": {},
    },
    "teleop.start": {
        "doc": "Acquire the motion lock and start a teleop session. "
               "Watchdog is clamped to the profile's max_watchdog.",
        "params": {"watchdog": "float s, requested"},
        "result": {"watchdog": "float s, granted"},
    },
    "teleop.stop": {
        "doc": "Immediate zero velocity; the session stays alive.",
        "params": None,
        "result": {},
    },
    "tf.frames": {
        "doc": "All frame names currently in the TF tree.",
        "params": None,
        "result": {"frames": "list[str]"},
    },
    "tf.lookup": {
        "doc": "Latest transform that re-expresses child-frame coordinates "
               "in the parent frame. @map/@odom/@base aliases resolve "
               "against the profile.",
        "params": {"parent": "str", "child": "str",
                   "timeout": "float s, default 2.0",
                   "stamp": "optional float ROS seconds; omit for latest"},
        "result": "Transform",
    },
    "tf.transform": {
        "doc": "Re-express a Pose or Point in another frame.",
        "params": {"kind": "'pose'|'point'", "entity": "Pose|Point",
                   "to_frame": "str", "timeout": "float s, default 2.0"},
        "result": {"entity": "Pose|Point in to_frame"},
    },
    "tf.wait_for": {
        "doc": "Block until a frame exists in TF; TimeoutError otherwise.",
        "params": {"frame": "str", "timeout": "float s, default 5.0"},
        "result": {},
    },
}
METHODS.pop('group.follow',None)
METHODS.pop('group.tool_set',None)
METHODS.update({
    'mobility.start_route':{'doc':'Accept a finite framed route with measured arrival at every endpoint.','params':{'goals':'list[Pose]'},'result':{'task_id':'str'}},
    'group.start_tool':{'doc':'Accept verified positioning work; physical contact/holding is separate.','params':{'group':'str','position':'finite float native units'},'result':{'task_id':'str'}},
    'measurement.get':{'doc':'Read a bounded fresh pose/joint observation with source/receive provenance.','params':{'kind':'pose|joints','group':'str|null','frame':'str|null','max_age':'positive float','timeout':'nonnegative float','require_valid':'bool'},'result':'Measurement'},
    'signal.get':{'doc':'Read a discovered signal with explicit source/receive freshness.','params':{'path':'str','max_age':'positive float','timeout':'nonnegative float','require_valid':'bool'},'result':'Measurement'},
    'guard.open':{'doc':'Owner-scoped persistent guard; initial fresh safe evaluation precedes acceptance.','params':{'condition':'SignalCondition'},'result':'GuardStatus'},
    'guard.status':{'doc':'Owner-only guard status.','params':{'guard_id':'str'},'result':'GuardStatus'},
    'guard.reset':{'doc':'Reset after verified cleanup and fresh safe evaluation; never resumes old work.','params':{'guard_id':'str'},'result':'GuardStatus'},
    'guard.remove':{'doc':'Stop/drain before removing only this guard inhibit.','params':{'guard_id':'str'},'result':{'removed':'bool','cleanup_confirmed':'bool'}},
})
METHODS['task.cancel']['result']='CancelReceipt (operation_id supplied by SDK)'


def _first_line(doc: str | None) -> str:
    """The summary line of a docstring, whitespace-collapsed."""
    if not doc:
        return ""
    for block in doc.strip().split("\n\n"):
        return " ".join(block.split())
    return ""


NOTIFICATIONS = {
    "audit.event": {
        "doc": "One audit event, pushed to connections that called "
               "audit.subscribe.",
        "params": "AuditEvent",
    },
    "event": {
        "doc": "One system event (kind: estop|watch — a subset of "
               "AuditKind), pushed to "
               "connections that called events.subscribe.",
        "params": {"kind": "str", "stamp": "float", "data": "object"},
    },
    "stream.data": {
        "doc": "One tick of a camera/lidar stream (kind: camera.frame | "
               "lidar.scan; arrays via PayloadRef), or its termination "
               "({stream_id, end: true, error?}).",
        "params": {"stream_id": "str", "...": "tick fields by kind"},
    },
    "payload.header": {
        "doc": "Announces one binary WebSocket frame. The frame's first "
               "8 bytes are the big-endian payload id, then the raw bytes.",
        "params": "PayloadHeader",
    },
    "task.update": {
        "doc": "Task state/progress change, pushed to the client that "
               "started the task.",
        "params": "TaskUpdate",
    },
}


def build() -> dict:
    """The full wire schema. Split out so tests can assert the committed
    protocol.json still matches the models it was generated from."""
    return {
        "protocol": PROTOCOL_VERSION,
        # The fixed vocabularies (spec 27), hoisted so a consumer that is
        # not walking a model's $defs — the agent's param validation, a
        # future MCP manifest — can read the legal values directly.
        "enums": {
            name: {
                "doc": _first_line(getattr(enums, name).__doc__),
                "values": [member.value for member in getattr(enums, name)],
            }
            for name in enums.__all__
        },
        "models": {
            name: getattr(models, name).model_json_schema()
            for name in models.__all__
        },
        "methods": METHODS,
        "notifications": NOTIFICATIONS,
    }


def render() -> str:
    return json.dumps(build(), indent=2, sort_keys=True) + "\n"


def main() -> None:
    out = ENGINE_DIR / "protocol.json"
    out.write_text(render())
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
