"""Fixed vocabularies (spec section 27) — no stringly-typed API.

Every enum here subclasses ``str``, so the member **is** the wire value and
the YAML value. Nothing converts at the boundary:

    Controller.TRAJECTORY == "trajectory"    -> True
    json.dumps(Controller.TRAJECTORY)        -> '"trajectory"'
    profile.controllers[Controller.TOOL]     -> works on a YAML-loaded dict

Bare strings keep working everywhere a member is accepted — that is the
point of the ``str`` base, and it is what keeps the shipped Phase 3-5
surface source-compatible. Pydantic coerces a legal string to the member
on parse and rejects an illegal one, which is exactly what the ``Literal``
unions these replace already did.

The enums also give ``protocol.json`` real JSON-schema ``enum``
constraints, which the agent validates against and a future MCP manifest
inherits: an LLM calling ``control`` sees four legal roles instead of
guessing a string.
"""

from __future__ import annotations

from enum import Enum

__all__ = [
    "AuditKind",
    "Controller",
    "Frame",
    "GroupKind",
    "Locomotion",
    "Outcome",
    "Phase",
    "SlamMode",
    "Transport",
]


class _WireEnum(str, Enum):
    """Base for every ringfour enum: the member is its own wire value.

    ``__str__`` and ``__format__`` are pinned to ``str``'s deliberately.
    CPython changed mixin-enum formatting twice between 3.10 and 3.12, and
    the default in some of those versions renders ``Controller.TRAJECTORY``
    where the wire says ``trajectory``. A log line or an f-stringed error
    message that disagrees with the protocol is precisely the confusion
    these enums exist to remove, so we pin the behavior rather than inherit
    whichever one the interpreter happens to ship.
    """

    __str__ = str.__str__
    __format__ = str.__format__

    def __repr__(self) -> str:  # "<Controller.TRAJECTORY: 'trajectory'>" is noise
        return f"{type(self).__name__}.{self.name}"


class Controller(_WireEnum):
    """The *role* a controller plays for a group, not its ROS node name.

    The profile maps role -> controller name
    (``{trajectory: arm_trajectory_controller, ...}``), so a mission asks
    for a capability and the profile answers with the robot's spelling of
    it. Every declared role must be loaded at boot (plan 6.0b) so the
    switch is a switch, never a load.
    """

    TRAJECTORY = "trajectory"   # JTC — follow(), waypoints
    STREAM = "stream"           # forward position/velocity — rate sessions
    VELOCITY = "velocity"       # joint velocity commands
    TOOL = "tool"               # gripper action controller


class Phase(_WireEnum):
    """Progress state of a motion, shared by NavStatus and TrajStatus.

    Not every phase is reachable from every verb: navigation never reports
    PLANNING (Nav2 plans inside MOVING) and trajectory execution has no
    RECOVERING. The union is one vocabulary on purpose — a mission that
    switches on ``status.phase`` reads the same either way.
    """

    PLANNING = "planning"
    MOVING = "moving"
    ARRIVED = "arrived"
    RECOVERING = "recovering"
    STUCK = "stuck"
    CANCELLED = "cancelled"
    FAILED = "failed"


class Frame(_WireEnum):
    """The three frame aliases the agent resolves against the profile.

    These stay strings on the wire — ``"@map"`` is resolved agent-side
    against ``profile.frames``, so the client never needs to know that this
    robot calls its base ``base_link`` and the next one ``base_footprint``.

    (The Phase 4 RGB image type that used to own this name is now
    ``ImageFrame`` — decision 17, revised 2026-08-24. The enum keeps the
    spec section 27 spelling, which is the one missions read.)
    """

    MAP = "@map"
    ODOM = "@odom"
    BASE = "@base"


class GroupKind(_WireEnum):
    """What a joint group is for, which decides what it is allowed to do.

    ARM groups take IK and cartesian targets (they have a ``tip``);
    POINTING (pan/tilt) and LINEAR (lifts, rails) are joint-space only.
    """

    ARM = "arm"
    POINTING = "pointing"
    LINEAR = "linear"


class AuditKind(_WireEnum):
    """The kind of occurrence an AuditEvent records (spec section 18)."""

    COMMAND = "command"
    RESULT = "result"
    SAFETY = "safety"
    WATCH = "watch"
    ESTOP = "estop"
    CONNECTION = "connection"


class Outcome(_WireEnum):
    """How an audited call ended.

    REJECTED is the safety layer refusing before the handler ran; ERROR is
    the handler (or a task body) failing; CANCELLED is a running task
    stopped on request.

    CANCELLED is a fourth member beyond the three in spec section 27
    (decision 18, 2026-08-24). The audit log is the safety record, and it
    has to distinguish "the operator stopped it" from "it broke" —
    collapsing a cancellation into ERROR would make the log lie about the
    one thing it exists to answer. The free text that used to be stuffed
    in here ("error: <msg>", "failed: <msg>") moved to ``detail``, which
    is where the model always said it belonged.
    """

    SUCCESS = "success"
    ERROR = "error"
    REJECTED = "rejected"
    CANCELLED = "cancelled"


class Locomotion(_WireEnum):
    """The base's drive type. Gates what teleop accepts: ``lateral``
    velocity needs OMNI."""

    DIFFERENTIAL = "differential"
    OMNI = "omni"
    LEGGED = "legged"


class SlamMode(_WireEnum):
    """slam_toolbox's mode. Switched at service level, never by restarting
    the process (see the nav restructure)."""

    MAPPING = "mapping"
    LOCALIZATION = "localization"
    IDLE = "idle"


class Transport(_WireEnum):
    """How array bytes reach the client.

    SHM maps local Linux /dev/shm read-only. NET sends raw numeric arrays in
    tagged binary WebSocket frames, independently of JSON-RPC command objects.
    New TCP clients negotiate NET; Unix clients default to SHM. Both can select
    explicitly with Robot(..., transport=Transport.NET or Transport.SHM).
    """

    SHM = "shm"
    NET = "net"


class OperationState(str,Enum):
    ACCEPTED='accepted'
    RUNNING='running'
    CANCELLING='cancelling'
    SUCCEEDED='succeeded'
    FAILED='failed'
    CANCELLED='cancelled'
