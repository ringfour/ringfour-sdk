"""Handshake and protocol-level models (spec sections 3, 22, 23).

All models are frozen: instances are immutable after construction.
"""

from __future__ import annotations

from typing import Any

from pydantic import BaseModel, ConfigDict

from ..enums import Controller, GroupKind


class ProfileInfo(BaseModel):
    """What the client learns about the robot at handshake time.

    ``capabilities`` lists the capability names the profile provides
    (e.g. "mobility", "lidar"); see spec section 4 for the registry.
    """

    model_config = ConfigDict(frozen=True)

    name: str
    version: str
    model: str
    capabilities: tuple[str, ...]


class Hello(BaseModel):
    """First client message. The agent rejects a mismatched protocol with
    a ProtocolMismatch error before anything else is allowed."""

    model_config = ConfigDict(frozen=True)

    protocol: int
    client_version: str


class ToolInfo(BaseModel):
    """The tool fitted to a group, as the client sees it."""

    model_config = ConfigDict(frozen=True)

    type: str
    joint: str | None = None
    open: float | None = None
    closed: float | None = None


class GroupInfo(BaseModel):
    """One group's static shape, delivered in the handshake.

    Everything here is profile config plus the joint list the agent
    resolved off the controller at startup. It is in the handshake and not
    behind a call because kinematics is client-side (spec 26.2): a mission
    doing `kin.reachable()` in a loop must not pay a round trip per solve,
    and it cannot solve at all without knowing the tip and the joint order.
    """

    model_config = ConfigDict(frozen=True)

    kind: GroupKind
    # None disables IK for this group - a lift or a head has nothing to
    # reach with. That is the gate, not an error.
    tip: str | None = None
    # Which Cartesian DOF the chain can control, [x, y, z, rx, ry, rz].
    dof_mask: tuple[int, ...] | None = None
    # Resolved at startup from the controller's claimed interfaces, in
    # command order. The profile usually does not state these.
    joints: tuple[str, ...] = ()
    tool: ToolInfo | None = None
    pointing_frame: str | None = None
    # Server-side reachability box in the base frame, {"x": (lo, hi), ...}.
    workspace: dict[str, tuple[float, float]] | None = None
    control_modes: tuple[Controller,...] = ()
    # Role -> controller name, resolved. Reported so a mission can name
    # the controller it is about to drive in its own logs.


class Welcome(BaseModel):
    """The agent's handshake reply: protocol version + capability report.

    ``instances`` lists the named instances of multi-instance capabilities
    ({"arms": ("left", "right"), ...}); it is what lets the client resolve
    singular sugar (robot.arm vs robot.arms["left"]) without extra calls.
    """

    model_config = ConfigDict(frozen=True)

    protocol: int
    profile: ProfileInfo
    capabilities: tuple[str, ...]
    instances: dict[str, tuple[str, ...]] = {}
    features: dict[str, Any] = {}
    # The live /robot_description, verbatim, and the sha256 of exactly
    # these bytes (spec 26.2). Carried in the handshake so a client can
    # build its chain with no extra round trip, and so there is never a
    # file path anywhere in the system that could describe a different
    # robot than the one moving.
    #
    # None means the latched publisher had not been heard from when this
    # client connected — a robot_state_publisher that starts after the
    # agent. That is a real state, not an error: `model.changed` arrives
    # when it shows up.
    description: str | None = None
    description_sha: str | None = None
    # Groups whose profile overrides `description_topic` (namespaced
    # launches publish per-arm descriptions). {group: sha}; fetch the XML
    # with `model.description`. Empty on every robot that publishes one
    # composed description, which is all of them so far.
    group_description_shas: dict[str, str] = {}
    # Static shape of every declared group. `instances` still carries the
    # names (that is the generic capability mechanism); this carries what
    # you need to compute with them.
    groups: dict[str, GroupInfo] = {}
    # Vendor escape hatches the profile declares. Shipped in the
    # handshake so a client can build `robot.sensors` / `robot.actions`
    # (and their typed watch handles) without a round trip, and so a
    # mission can ask what a robot offers before calling anything.
    sensors: dict[str, dict[str, dict[str, Any]]] = {}
    actions: dict[str, dict[str, Any]] = {}


class PayloadHeader(BaseModel):
    """Announces one binary frame. ``kind`` says what the bytes are
    ("image/jpeg", "depth/f32", ...); ``meta`` carries kind-specific
    fields (camera name, stamp, array shape, ...)."""

    model_config = ConfigDict(frozen=True)

    payload_id: int
    kind: str
    meta: dict[str, Any]
