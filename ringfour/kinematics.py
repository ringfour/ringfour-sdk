"""Client-side kinematics (spec 26). Optional: ``pip install ringfour[kinematics]``.

Free functions over a group, not methods on one. A group is data — its
joint names, its tip, its DOF mask, and the description it lives in — and
keeping the maths outside the handle is what stops `ringfour.sensing` from
importing a solver.

    from ringfour import kinematics as kin

    q = kin.ik(robot.arm, target_pose)      # raises Unreachable
    pose = kin.fk(robot.arm, q=q)           # hypothetical, no robot involved

**TF answers actual, this answers hypothetical** (spec 26.3). Where *is*
the gripper right now, in map frame? `robot.tf.pose_of(Frame.MAP, tip)`.
Where *would* it be at configuration q? `kin.fk(group, q=q)`. If a mission
reaches for `fk` to find the current pose, it wants TF.

**Determinism is the contract.** Same robot state plus same pose gives the
same joints, every time. `seed`
defaults to the group's current joint positions so successive solves stay
in the same branch and the arm does not flip configuration mid-mission.
That is the whole difference from a planner.

**The backend is wrapped and never exposed.** No ``roboticstoolbox`` type
crosses any signature here; poses go in and out as ``ringfour.Pose``,
joints as plain lists. Swapping to pinocchio should touch this file only.

**Frames.** Everything is in the robot's base frame — the description's
root link. `fk` returns a Pose with `frame` set to that root, and `ik`
expects the same. Converting to or from map frame is `robot.tf`'s job,
and doing it here would quietly mix live state into a hypothetical
answer.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from functools import lru_cache
from typing import TYPE_CHECKING, Any, Sequence, Mapping

import numpy as np

from .description import JointSpec, RobotDescription
from .enums import GroupKind
from .errors import CapabilityNotSupported, Unreachable, ModelChanged
from .models.execution import JointConfiguration
from .models import Pose, Quaternion

if TYPE_CHECKING:      # pragma: no cover
    from .models import GroupInfo

__all__ = [
    "LinkInfo",
    "collision_model",
    "reach_map",
    "snapshot",
    "collisions",
    "fk",
    "in_collision",
    "ik",
    "jacobian",
    "limits",
    "links",
    "manipulability",
    "point_at",
    "reach",
    "reachable",
]

# Default solver tolerance, meters/radians. Tighter than any arm's
# repeatability, loose enough that damped least-squares converges.
_AXES = ("x", "y", "z", "rx", "ry", "rz")

_TOL = 1e-6
_MAX_ITER = 500


@dataclass(frozen=True)
class LinkInfo:
    """One link on a group's chain, as `kin.links` reports it."""

    name: str          # the child link
    joint: str         # the joint that places it
    type: str          # revolute | continuous | prismatic | fixed
    moves: bool
    limits: tuple[float, float] | None


def _backend() -> Any:
    """Import the solver, lazily and with a message worth reading.

    The core SDK must import and run with no ``roboticstoolbox`` present
    (spec 26.2 / 1.6's dependency promise), so nothing at module scope may
    touch it — this is called only from the functions that solve.
    """
    try:
        import roboticstoolbox as rtb
        import spatialmath
    except ImportError as exc:                   # pragma: no cover
        raise CapabilityNotSupported(
            "kinematics needs the optional backend: "
            "pip install 'ringfour[kinematics]'. The core SDK "
            "deliberately does not depend on it - everything except "
            f"ik/fk/jacobian works without it. ({exc})"
        ) from exc
    assert spatialmath is not None      # both halves must be importable
    return rtb


# -- turning a description into a chain ------------------------------------------

def _rpy_matrix(rpy: tuple[float, float, float]) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw -> 3x3. R = Rz(yaw) Ry(pitch) Rx(roll)."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def _origin_matrix(joint: JointSpec) -> np.ndarray:
    """The joint's fixed parent-to-joint transform, as 4x4."""
    T = np.eye(4)
    T[:3, :3] = _rpy_matrix(joint.origin_rpy)
    T[:3, 3] = joint.origin_xyz
    return T


def _align_z_to(axis: tuple[float, float, float]) -> np.ndarray:
    """A rotation taking +z onto ``axis``, as 4x4.

    URDF allows a joint axis in any direction; rtb's elementary transforms
    are about the principal axes. Rotating the frame so that +z is the
    axis, applying Rz(q), then rotating back is exact for any axis, and
    the "back" half folds into the next link's fixed transform (see
    `_build_chain`) so no extra links appear.
    """
    a = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(a)
    if norm < 1e-12:
        raise ValueError("joint axis is the zero vector")
    a = a / norm
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(z, a)
    c = float(np.dot(z, a))
    T = np.eye(4)
    if np.linalg.norm(v) < 1e-12:
        # Parallel or antiparallel: a 180 degree turn about x for the
        # antiparallel case, identity otherwise.
        if c < 0:
            T[:3, :3] = np.diag([1.0, -1.0, -1.0])
        return T
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    T[:3, :3] = np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))
    return T


def _build_chain(chain: Sequence[JointSpec], name: str,
                 frozen: Mapping[str, float] | None = None) -> Any:
    """A serial rtb model from the description's own joints.

    Deliberately not rtb's URDF reader: that wants a file path, and this
    SDK never has one (spec 26.2). Building from the parsed description
    also means the client's chain and the agent's validation are looking
    at the same objects.
    """
    rtb = _backend()
    from spatialmath import SE3

    links: list[Any] = []
    pending = np.eye(4)          # fixed transform carried into the next joint
    previous: Any = None
    joint_names: list[str] = []

    for joint in chain:
        pending = pending @ _origin_matrix(joint)
        if not joint.moves:
            continue
        if frozen is not None and joint.name in frozen:
            pending = pending @ _joint_matrix(joint, frozen[joint.name])
            continue
        align = _align_z_to(joint.axis or (0.0, 0.0, 1.0))
        ets = rtb.ET.SE3(SE3(pending @ align, check=False))
        # Prismatic joints translate along the axis; everything else
        # rotates about it. Both are now the local +z.
        ets = ets * (rtb.ET.tz() if joint.type == "prismatic"
                     else rtb.ET.Rz())
        link = rtb.Link(ets, name=joint.child, parent=previous)
        if joint.limits is not None:
            link.qlim = list(joint.limits)
        links.append(link)
        previous = link
        joint_names.append(joint.name)
        pending = align.T @ np.eye(4)      # undo the alignment downstream

    # Put the remaining fixed geometry into the chain itself. A constructor
    # tool offset is ignored by some backend paths; removing it from the
    # target also changes the meaning of position-only masks. A fixed link
    # makes FK, IK and the Jacobian all refer to the declared physical tip.
    links.append(rtb.Link(rtb.ET.SE3(SE3(pending, check=False)),
                          name=f"{name}_fixed_tip", parent=previous))
    robot = rtb.Robot(links, name=name)
    robot.ringfour_joint_names = tuple(joint_names)
    return robot


@lru_cache(maxsize=8)
def _model_for(sha: str, urdf: str, joints: tuple[str, ...],
               tip: str, frozen: tuple[tuple[str, float], ...] = ()) -> Any:
    """Cached chain for one (description, group) pair.

    Keyed by the description sha, so a `model.changed` that rebuilds the
    robot produces a different key and the stale chain is never reused.

    The chain runs from the description's ROOT to the tip, not from the
    group's first joint. `subchain` answers "which span does this group
    own" and is right for `links`; it is wrong here, because everything
    this module returns is in the base frame and the transforms
    above the first actuator (the deck, the arm mount) are exactly what
    puts the arm on the robot. Slicing them off silently moved base101's
    flange 0.42 m.
    """
    description = RobotDescription.parse(urdf)
    chain = description.chain_to(tip)
    if any(description.joints[n].type in {"floating", "planar"} for n in chain):
        raise CapabilityNotSupported("floating/planar chains require a dedicated solver")
    moving = tuple(name for name in chain if description.joints[name].moves)
    selected = tuple(name for name in moving if name in joints)
    if selected != joints or len(set(joints)) != len(joints):
        raise ValueError("the group's joints do not match the chain order to "
                         f"{tip!r}: selected {selected}, declared {joints}")
    if set(dict(frozen)) != set(moving) - set(joints):
        raise ValueError("missing frozen joint positions for the chain")
    return _build_chain([description.joints[name] for name in chain],
                        name=tip, frozen=dict(frozen))



# -- group plumbing ---------------------------------------------------------------

def _resolve(group: Any) -> tuple["GroupInfo", RobotDescription, Any]:
    """(info, description, robot) for a group handle.

    Accepts the client handles (`robot.arm`, `robot.groups[name]`). They
    carry a back-reference to the connection, which is how the static
    shape and the description are reached without a round trip.
    """
    robot = getattr(group, "_owner", None)
    if robot is None:
        raise TypeError(
            f"{type(group).__name__} {getattr(group, 'name', group)!r} is "
            f"not a group handle from a connected Robot. Pass "
            f"robot.arm or robot.groups[name].")
    info = robot.group_info(group.name)
    actual=getattr(robot,"_robot",robot)
    description = getattr(actual,"_group_models",{}).get(group.name) or robot.description
    welcome=getattr(actual,'_welcome',None)
    expected=welcome.group_description_shas.get(group.name) if welcome is not None else None
    if expected is not None and (description is None or expected!=description.sha):
        raise ModelChanged('refresh this group description explicitly before planning or configuring commands')
    if description is None:
        raise CapabilityNotSupported(
            "no robot description available, so no chain can be built. "
            "The topic is latched - this means robot_state_publisher is "
            "not running.")
    return info, description, robot


def _require_tip(info: "GroupInfo", group_name: str,
                 override: str | None = None) -> str:
    tip = override or info.tip
    if not tip:
        raise CapabilityNotSupported(
            f"group {group_name!r} declares no tip, so it has no end "
            f"effector to solve for. Add `tip:` to the profile, or pass "
            f"tip= if you know which link you mean. fk/limits/links/reach "
            f"work without one.")
    return tip


def _positions(group: Any, info: "GroupInfo", description: RobotDescription,
               state: Mapping[str, float] | None = None) -> dict[str, float]:
    external = {j.name for j in description.joints.values()
                if j.moves and j.name not in info.joints}
    if state is None:
        state = group.get_kinematic_positions() if external else {}
    positions = dict(state)
    missing = external - positions.keys()
    if missing:
        raise ValueError(f"missing measured joint positions: {sorted(missing)}")
    if any(not math.isfinite(float(v)) for v in positions.values()):
        raise ValueError("joint positions must be finite")
    return positions


def _chain_of(group: Any, tip: str | None = None,
              state: Mapping[str, float] | None = None):
    info, description, robot = _resolve(group)
    resolved_tip = _require_tip(info, group.name, tip)
    positions = _positions(group, info, description, state)
    frozen = tuple((name, float(positions[name]))
                   for name in description.chain_to(resolved_tip)
                   if description.joints[name].moves and name not in info.joints)
    model = _model_for(description.sha, description.urdf, info.joints,
                       resolved_tip, frozen)
    return info, description, robot, model, resolved_tip


def _q_of(group: Any, info: "GroupInfo", q: Sequence[float] | None
          ) -> list[float]:
    """``q`` as given, or the group's live joint positions.

    Group joint read fallback when no reusable snapshot was supplied.
    Ordering is `info.joints`, which is the controller's command order.
    """
    if q is not None:
        from .models.execution import JointConfiguration
        if isinstance(q,JointConfiguration):
            from .configuration import validate_configuration
            return validate_configuration(group,q)
        values = [float(v) for v in q]
        if len(values) != len(info.joints):
            raise ValueError(
                f"q has {len(values)} values, group {group.name!r} has "
                f"{len(info.joints)} joints ({', '.join(info.joints)})")
        return values
    positions = group.get_joint_positions()
    if hasattr(positions, "__await__"):
        # An aio handle: reading live state would need an await, and these
        # are plain functions by design (spec 26.1 - they are meant to be
        # called in a loop). Everything else works on an aio handle.
        raise TypeError(
            "reading live joint positions needs a sync Robot handle; from "
            "async code pass q= (or seed=) explicitly")
    try:
        return [float(positions[name]) for name in info.joints]
    except KeyError as exc:
        raise ValueError(
            f"no live position for joint {exc.args[0]!r}; pass q= "
            f"explicitly") from None


def _pose_from(matrix: np.ndarray, frame: str) -> Pose:
    """4x4 -> Pose, orientation included.

    Via the full quaternion, NOT via ``yaw=``. `Pose.yaw` is a
    convenience accessor for the navigation case; building a Pose from it
    silently discards roll and pitch, which for an arm means asking IK to
    hit a pose that FK never produced.
    """
    x, y, z = (float(v) for v in matrix[:3, 3])
    return Pose(x=x, y=y, z=z, q=_quaternion_from(matrix[:3, :3]),
                frame=frame)


def _quaternion_from(R: np.ndarray) -> Quaternion:
    """3x3 rotation -> unit quaternion (Shepperd's method).

    Branching on the largest diagonal term keeps it conditioned; the naive
    w-first formula loses precision as w approaches zero.
    """
    trace = float(R[0, 0] + R[1, 1] + R[2, 2])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return Quaternion(x=float(x), y=float(y), z=float(z), w=float(w))


def _se3_of(pose: Pose):
    """Pose -> SE3, orientation included. The inverse of `_pose_from`."""
    from spatialmath import SE3

    q = pose.q
    xx, yy, zz = q.x * q.x, q.y * q.y, q.z * q.z
    R = np.array([
        [1 - 2 * (yy + zz), 2 * (q.x * q.y - q.z * q.w),
         2 * (q.x * q.z + q.y * q.w)],
        [2 * (q.x * q.y + q.z * q.w), 1 - 2 * (xx + zz),
         2 * (q.y * q.z - q.x * q.w)],
        [2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w),
         1 - 2 * (xx + yy)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = (pose.x, pose.y, pose.z or 0.0)
    return SE3(T, check=False)


# -- the public surface -----------------------------------------------------------

@lru_cache(maxsize=8)
def _snapshot_for(sha: str) -> Any:
    """The recorded snapshot for this description, or None.

    Looked up by sha alone, so a snapshot of a different build of the same
    robot is never substituted. Cached because the lookup touches the
    filesystem and `collisions()` is called in loops.
    """
    from .snapshot import Snapshot

    try:
        return Snapshot.find(sha)
    except (OSError, ValueError):           # pragma: no cover
        return None


@lru_cache(maxsize=8)
def _collision_for(sha: str, urdf: str, joints: tuple[str, ...]) -> Any:
    """Cached collision model, keyed by description sha like the chain.

    Prefers the snapshot's geometry when one has been recorded for this
    exact description. That is what makes MESH links visible: parsed
    straight from XML a `<mesh>` carries no extent, and on base101 every
    mesh is a jaw — so without a snapshot the gripper is not checked at
    all, and `model.unmodelled` says so.
    """
    from .collision import CollisionModel

    description = RobotDescription.parse(urdf)
    snapshot = _snapshot_for(sha)
    if snapshot is not None:
        description = replace(description, collisions=snapshot.collisions)
    # Sample all robot joints when deriving structural exclusions. Sampling
    # only this group at zero external state can hide a collision pair that
    # becomes relevant after an independent lift or the other arm moves.
    return CollisionModel.build(description)


def collision_model(group: Any) -> Any:
    """The group's `CollisionModel`, built once per description.

    Worth reaching for directly to see what it can and cannot check:
    ``model.unmodelled`` lists links whose geometry this build cannot see
    — empty once a snapshot has been recorded — and ``model.always_touching``
    lists pairs disabled because they overlap in every configuration.
    """
    info, description, _robot = _resolve(group)
    return _collision_for(description.sha, description.urdf,
                          tuple(info.joints))


def snapshot(group: Any) -> Any:
    """The recorded snapshot backing this robot, or None if never onboarded.

    ``None`` is not an error — everything works without one, just with the
    mesh links unchecked and no reach map. Run
    ``ros2 run ringfour_agent onboard`` on the robot to record one.
    """
    _info, description, _robot = _resolve(group)
    return _snapshot_for(description.sha)


def reach_map(group: Any) -> Any:
    """The precomputed reachability map, or None if absent/state-dependent.

    A lookup, not a solve: `map.can_reach(point)` is an array index, where
    `reachable()` is most of a second when the answer is no. Use the map
    to filter candidates and `ik` to confirm the one you pick — the map is
    a lower bound (an empty cell means "not seen", not "impossible") and
    it ignores orientation.
    """
    info, description, _robot = _resolve(group)
    if info.tip and any(description.joints[n].moves and n not in info.joints
                        for n in description.chain_to(info.tip)):
        # Old maps contain no external-state metadata. A zero-lift map must
        # not be presented as valid at the currently measured lift height.
        return None
    recorded = snapshot(group)
    return None if recorded is None else recorded.reach


def collisions(group: Any, q: Sequence[float] | None = None, *,
               state: Mapping[str, float] | None = None) -> tuple:
    """Link pairs of this robot overlapping at ``q``, deepest first.

    External moving joints use measured positions, or the supplied state.

    Self-collision only — the robot against itself, including the parts it
    is bolted to. Nothing here knows about the world.
    """
    from .collision import link_poses

    info, description, _robot = _resolve(group)
    model = collision_model(group)
    values = _positions(group, info, description, state)
    selected = ([values[j] for j in info.joints]
                if q is None and all(j in values for j in info.joints)
                else _q_of(group, info, q))
    values.update(zip(info.joints, selected))
    poses = link_poses(description, values, model.links)
    return model.check(poses)


def in_collision(group: Any, q: Sequence[float] | None = None, *,
                 state: Mapping[str, float] | None = None) -> bool:
    """True if the robot overlaps itself at ``q``."""
    return bool(collisions(group, q, state=state))


def fk(group: Any, q: Sequence[float] | None = None,
       link: str | None = None, *,
       state: Mapping[str, float] | None = None) -> Pose:
    """Where ``link`` (default: the tip) would be at configuration ``q``.

    External moving joints use measured positions, or the supplied state.

    In the description's root frame. ``q`` defaults to the group's current
    joint positions, which makes ``fk(group)`` the hypothetical that
    happens to match reality — useful for checking the chain against TF,
    and not a substitute for it (spec 26.3).

    Computed here rather than in the backend. Forward kinematics is a
    product of 4x4s, not a solve: doing it natively means it works with no
    backend installed, and it means ``link=`` can name ANY link in the
    description — including one above the group's first joint, where a
    chain built for the group has nothing to say.
    """
    info, description, _robot = _resolve(group)
    target = link or _require_tip(info, group.name)
    if not description.has_link(target):
        raise ValueError(
            f"{target!r} is not a link in this robot's description")
    chain = description.chain_to(target)
    values = _positions(group, info, description, state)
    selected = ([values[j] for j in info.joints]
                if q is None and all(j in values for j in info.joints)
                else _q_of(group, info, q))
    values.update(zip(info.joints, selected))

    matrix = np.eye(4)
    for name in chain:
        joint = description.joints[name]
        if joint.type in {"floating", "planar"}:
            raise CapabilityNotSupported("floating/planar chains require a dedicated solver")
        matrix = matrix @ _origin_matrix(joint)
        if joint.moves:
            matrix = matrix @ _joint_matrix(joint, values[name])
    return _pose_from(matrix, description.root or "base_link")


def _joint_matrix(joint: JointSpec, value: float) -> np.ndarray:
    """The variable part of a joint's transform at ``value``."""
    axis = np.asarray(joint.axis or (0.0, 0.0, 1.0), dtype=float)
    axis = axis / np.linalg.norm(axis)
    T = np.eye(4)
    if joint.type == "prismatic":
        T[:3, 3] = axis * value
        return T
    # Rodrigues, about the joint's own axis.
    c, s = math.cos(value), math.sin(value)
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    T[:3, :3] = np.eye(3) + s * K + (1 - c) * (K @ K)
    return T


def ik(group: Any, pose: Pose, seed: Sequence[float] | JointConfiguration | None = None,
       tip: str | None = None,
       mask: Sequence[int] | None = None,
       avoid_collisions: bool = True, *,
       state: Mapping[str, float] | None = None) -> JointConfiguration:
    """A model-bound named configuration putting the tip at ``pose``. Raises `Unreachable`.

    Only the group's joints are solved. Other robot joints stay at their
    measured positions. Pass state=group.get_kinematic_positions() to reuse one
    snapshot across several solves; targets remain in the description root
    frame. Missing external feedback is an error. This snapshot does not
    lock other controllers: replan if a lift or torso moves before execution.

    Raising rather than returning None is deliberate: a membership test
    over a list of waypoints cannot say *which* pose failed, and that is
    the only thing the caller wants to know.

    ``mask`` says which Cartesian DOF to constrain, `[x, y, z, rx, ry,
    rz]`. It defaults to the profile's `dof_mask`, because a 5-DOF arm
    asked to hold a full 6-DOF pose has no solution and should say so
    rather than converge on a near miss.

    ``avoid_collisions`` (default on) rejects a solution that puts the
    robot through itself. It REJECTS rather than searches: the solver is
    deterministic; it does not search for a collision-free path. If the
    returned solution collides, `Unreachable` names the pair.

    Note this is a planning aid, not a guarantee. Nothing stops a mission
    commanding raw joint waypoints through the same configuration; a
    guarantee before motion is spec 8.7 and needs the real collision work.
    """
    info, description, _robot = _resolve(group)
    # A missing target is an interface error, independent of snapshot support.
    _require_tip(info, group.name, tip)
    state = _positions(group, info, description, state)
    info, _description, _robot, model, resolved = _chain_of(group, tip, state)
    start = (_q_of(group,info,seed) if seed is not None else
             [state[j] for j in info.joints] if all(j in state for j in info.joints)
             else _q_of(group, info, None))
    flags = list(mask) if mask is not None else (
        list(info.dof_mask) if info.dof_mask else [1, 1, 1, 1, 1, 1])
    if len(flags) != 6:
        raise ValueError(f"mask must have 6 entries, got {len(flags)}")

    target = _se3_of(pose)
    solution = model.ikine_LM(
        target, q0=np.asarray(start, dtype=float),
        mask=flags, tol=_TOL, ilimit=_MAX_ITER, seed=0, joint_limits=True,
    )
    if not solution.success:
        raise Unreachable(
            f"no IK solution for group {group.name!r} tip {resolved!r} at "
            f"({pose.x:.3f}, {pose.y:.3f}, {(pose.z or 0.0):.3f}) in "
            f"{pose.frame}: {solution.reason}. Constrained DOF: "
            f"{[a for a, on in zip(_AXES, flags) if on]}"
        )
    answer = [float(v) for v in solution.q]
    if avoid_collisions:
        hits = collisions(group, answer, state=state)
        if hits:
            raise Unreachable(
                f"the only IK solution for group {group.name!r} at "
                f"({pose.x:.3f}, {pose.y:.3f}, {(pose.z or 0.0):.3f}) puts "
                f"the robot through itself: {'; '.join(str(h) for h in hits)}"
                f". Pass avoid_collisions=False to get it anyway.")
    from .configuration import configuration
    return configuration(group,dict(zip(info.joints,answer)))


def reachable(group: Any, pose: Pose, **kwargs: Any) -> bool:
    """`ik` without the raise — including its collision check.

    So "reachable" means *the arm can actually get there*, not merely that
    the arithmetic closes. That is what makes it the right predicate for
    sampling a workspace.
    """
    try:
        ik(group, pose, **kwargs)
        return True
    except Unreachable:
        return False


def reach(group: Any, *, state: Mapping[str, float] | None = None) -> float:
    """Maximum base-to-tip extension, meters.

    The sum of the link translations — an upper bound on what the arm can
    touch, not a promise that every point inside it is reachable.
    """
    _info, _description, _robot, model, _tip = _chain_of(group, state=state)
    total = 0.0
    for link in model.links:
        total += float(np.linalg.norm(np.asarray(link.A(0.0).A)[:3, 3]))
    return total


def limits(group: Any) -> dict[str, tuple[float, float] | None]:
    """{joint: (lower, upper) | None} from the description.

    Works without a tip and without the backend — it is a lookup, not a
    computation.
    """
    info, description, _robot = _resolve(group)
    return description.limits_of(info.joints)


def links(group: Any) -> tuple[LinkInfo, ...]:
    """The group's chain, tip last. Fixed joints included: they carry the
    mounting offsets between actuators."""
    info, description, _robot = _resolve(group)
    tip = _require_tip(info, group.name)
    return tuple(
        LinkInfo(name=joint.child, joint=joint.name, type=joint.type,
                 moves=joint.moves, limits=joint.limits)
        for joint in description.subchain(info.joints, tip)
    )


def jacobian(group: Any, q: Sequence[float] | None = None, *,
             state: Mapping[str, float] | None = None) -> np.ndarray:
    """The 6xN base-frame Jacobian at ``q``."""
    info, description, _robot = _resolve(group)
    state = _positions(group, info, description, state)
    info, _description, _robot, model, _tip = _chain_of(group, state=state)
    values = ([state[j] for j in info.joints]
              if q is None and all(j in state for j in info.joints)
              else _q_of(group, info, q))
    return np.asarray(model.jacob0(values))


def manipulability(group: Any, q: Sequence[float] | None = None,
                   mask: Sequence[int] | None = None, *,
                   state: Mapping[str, float] | None = None) -> float:
    """Yoshikawa manipulability — 0 at a singularity.

    Computed over the CONSTRAINED rows of the Jacobian, defaulting to the
    group's `dof_mask`. The textbook form sqrt(det(J Jt)) assumes at least
    as many joints as task dimensions; base101's arm has five joints and
    six rows, so J Jt is singular by construction and the textbook form
    returns 0.0 everywhere — a number that looks like a permanent
    singularity and means nothing. Masking to the DOF the arm actually
    controls is both correct and what `dof_mask` is for.

    A mission streaming Cartesian velocities can watch this to see a
    singularity coming. It is not a guarantee: the server-side joint
    velocity clamps are (spec 21, 26.5).
    """
    info, _description, _robot = _resolve(group)
    J = jacobian(group, q, state=state)
    flags = list(mask) if mask is not None else (
        list(info.dof_mask) if info.dof_mask else [1, 1, 1, 1, 1, 1])
    rows = [i for i, on in enumerate(flags) if on]
    if len(rows) > J.shape[1]:
        raise ValueError(
            f"group {group.name!r} has {J.shape[1]} joints but "
            f"{len(rows)} constrained DOF ({[_AXES[i] for i in rows]}); "
            f"manipulability is undefined. Narrow dof_mask to what the "
            f"chain can actually control.")
    Jm = J[rows, :]
    return float(math.sqrt(max(float(np.linalg.det(Jm @ Jm.T)), 0.0)))


_BORESIGHT = {"x": 0, "y": 1, "z": 2}


def point_at(group: Any, point: Sequence[float],
             strict: bool = True, axis: str = "z", *,
             state: Mapping[str, float] | None = None) -> list[float]:
    """Pan/tilt joint values aiming a `GroupKind.POINTING` group at a point.

    ``axis`` is WHICH WAY THE FRAME LOOKS, and it has to be said rather
    than assumed because ROS uses two conventions that differ by 90
    degrees. An OPTICAL frame (REP-103) looks along **+Z** — the default,
    and what a camera's ``*_optical_frame`` means. A BODY frame looks
    along **+X**. Aim a group at its optical frame with the default; aim
    it at a body link with ``axis="x"``. Getting this wrong points the
    head a quarter turn off, which looks exactly like a broken extrinsic.

    ``strict`` (default True) raises `Unreachable` when the joint limits
    cannot get the boresight onto the target. Pass False for the best aim
    available instead — which is what a tracker wants: a head at its pan
    stop, watching something slide out of frame, should turn as far as it
    can rather than refuse and watch it leave. The best solution is
    computed either way; `strict` only decides whether it is returned or
    thrown away.

    Trigonometry, not IK: a head has two joints and one thing to do with
    them. The point is in the base frame, like everything else here.
    """
    info, _description, _robot = _resolve(group)
    if info.kind is not GroupKind.POINTING:
        raise CapabilityNotSupported(
            f"point_at needs a group of kind pointing; {group.name!r} is "
            f"kind {info.kind}. For an arm, solve a pose with ik().")
    if len(info.joints) != 2:
        raise CapabilityNotSupported(
            f"point_at expects a pan/tilt pair; group {group.name!r} has "
            f"{len(info.joints)} joints ({', '.join(info.joints)})")
    target = np.asarray([float(v) for v in point], dtype=float)
    frame = info.pointing_frame or _require_tip(info, group.name)
    bounds = [limits(group).get(j) for j in info.joints]

    # WHY THIS IS NOT TWO CALLS TO atan2. It used to be:
    #
    #     pan  = atan2(y, x);  tilt = atan2(z, hypot(x, y))
    #
    # which aims the ORIGIN of the base frame at the target, and a head is
    # not at the origin. On mod101 the pan/tilt sits on a tower 0.4 m up
    # and 0.14 m forward, so that formula asked for pan -87 deg where the
    # truth was -31 deg — measured against the real robot on 2026-09-01 by
    # closing the loop on the gripper. It also assumed the head's axes are
    # aligned with the base frame, and that positive tilt looks UP; this
    # robot's tilt looks DOWN, which is the aerospace sign and the more
    # common one on a camera head.
    #
    # So: solve it, against the chain the description actually describes.
    # Two joints, one direction to match, and `fk` already knows where a
    # named frame ends up for any q — including the mounting offset, the
    # joint axes and whichever way they point. Gauss-Newton on the two
    # angular residuals converges in a handful of iterations, and because
    # it starts from the group's current pose it stays deterministic and
    # in-branch, exactly like `ik`.
    # SEVERAL SEEDS, AND THE ANTIPODE IS THE REASON. Nulling the residual
    # below puts the target on the +/-Z axis, and BOTH satisfy it: one aims
    # at the target, the other exactly away from it. Newton converges to
    # whichever is nearer the seed, so a seed pointing the wrong way
    # returns a solution 180 degrees out that looks perfectly converged.
    # Solve from a spread of starts, discard the ones facing backwards,
    # keep the best of the rest.
    if axis not in _BORESIGHT:
        raise ValueError(f"axis must be x, y or z; got {axis!r}")
    forward = _BORESIGHT[axis]
    state = _positions(group, info, _description, state)
    current = ([state[j] for j in info.joints]
               if all(j in state for j in info.joints)
               else _q_of(group, info, None))
    starts = [list(current), [0.0, 0.0]]
    for pan in (-1.0, 1.0):
        for tilt in (-0.6, 0.6):
            starts.append([_clamp(pan, bounds[0]), _clamp(tilt, bounds[1])])

    best: tuple[float, list[float]] | None = None
    for start in starts:
        q = [_clamp(v, b) for v, b in zip(start, bounds)]
        for _ in range(_POINT_AT_ITERS):
            residual = _aim_error(group, q, frame, target, forward, state)
            if residual is None or np.linalg.norm(residual) < _POINT_AT_TOL:
                break
            jacobian = np.zeros((2, 2))
            singular = False
            for axis in range(2):
                nudged = list(q)
                nudged[axis] += _POINT_AT_STEP
                shifted = _aim_error(group, nudged, frame, target, forward, state)
                if shifted is None:
                    singular = True
                    break
                jacobian[:, axis] = (shifted - residual) / _POINT_AT_STEP
            if singular:
                break
            try:
                delta = np.linalg.solve(jacobian, -residual)
            except np.linalg.LinAlgError:
                break
            # Cap the step: near a singularity the solve can propose a
            # half-turn, and a head that whips round to reach the same
            # aim is not what anyone asked for.
            largest = float(np.max(np.abs(delta)))
            if largest > _POINT_AT_MAX_STEP:
                delta *= _POINT_AT_MAX_STEP / largest
            q = [a + float(b) for a, b in zip(q, delta)]
            q = [_clamp(v, b) for v, b in zip(q, bounds)]
        miss = _aim_miss(group, q, frame, target, forward, state)
        if miss is not None and (best is None or miss < best[0]):
            best = (miss, q)

    if best is None:
        raise Unreachable(
            f"group {group.name!r} cannot be aimed at "
            f"({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}): the "
            f"target is at the lens")
    miss, q = best
    if miss > _POINT_AT_MAX_MISS and not strict:
        return q
    if miss > _POINT_AT_MAX_MISS:
        # Within joint limits there is no aim that gets closer. Saying so
        # beats returning a clamped angle that points somewhere else and
        # letting the caller wonder why the camera is looking at a wall.
        raise Unreachable(
            f"group {group.name!r} cannot be aimed at "
            f"({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}): the "
            f"closest its joint limits allow is {math.degrees(miss):.0f} "
            f"degrees off. Joints: "
            + ", ".join(f"{j} in [{math.degrees(b[0]):.0f}, "
                        f"{math.degrees(b[1]):.0f}] deg"
                        for j, b in zip(info.joints, bounds) if b))
    return q


# Gauss-Newton settings for point_at. Small numbers on purpose: two joints
# and a smooth objective converge in three or four steps, and the cap stops
# a near-singular solve from proposing a half-turn.
_POINT_AT_ITERS = 24
_POINT_AT_TOL = 1e-4          # radians of residual
_POINT_AT_STEP = 1e-5         # finite-difference step
_POINT_AT_MAX_STEP = 0.5      # radians per iteration
# Beyond this the head simply cannot be pointed there within its limits.
_POINT_AT_MAX_MISS = math.radians(2.0)


def _aim_miss(group: Any, q: Sequence[float], frame: str,
              target: np.ndarray, forward: int = 2,
              state: Mapping[str, float] | None = None) -> float | None:
    """Angle between the pointing frame's +Z and the target, in radians.

    The unsigned truth, used to choose between solutions. Unlike the
    residual this DOES separate aiming at the target from aiming away
    from it, which is the whole reason it exists.
    """
    pose = fk(group, q=list(q), link=frame, state=state)
    origin = np.array([pose.x, pose.y, pose.z or 0.0])
    ray = target - origin
    distance = float(np.linalg.norm(ray))
    if distance < 1e-6:
        return None
    boresight = _rotation_of(pose)[:, forward]
    return float(np.arccos(np.clip((ray / distance) @ boresight, -1.0, 1.0)))


def _aim_error(group: Any, q: Sequence[float], frame: str,
               target: np.ndarray, forward: int = 2,
               state: Mapping[str, float] | None = None) -> np.ndarray | None:
    """How far the pointing frame's +Z misses `target`, as two angles.

    The residual is expressed in the pointing frame's OWN x and y axes,
    which is what makes it two independent numbers a 2-DOF head can null:
    x is "aim further left/right", y is "aim further up/down", both in the
    head's terms rather than the world's.
    """
    pose = fk(group, q=list(q), link=frame, state=state)
    origin = np.array([pose.x, pose.y, pose.z or 0.0])
    rotation = _rotation_of(pose)
    ray = target - origin
    distance = float(np.linalg.norm(ray))
    if distance < 1e-6:
        return None                     # target is at the lens
    ray /= distance
    # OFF-AXIS ANGLES, not dot products — and the difference is the
    # antipode. Nulling the two perpendicular COMPONENTS is satisfied
    # both when the frame looks at the target and when it looks exactly
    # away, so Newton converges happily to the wrong root: aiming at
    # something directly behind returned "already aimed", 180 degrees
    # out, from every seed. Scoring the seeds cannot rescue that when
    # they all fall into the same hole.
    #
    # atan2 against the boresight component makes the two cases different
    # numbers: dead on is (0, 0), 13 degrees off is (0.23, 0), directly
    # behind is (pi, pi) — the largest residual there is, which drives the
    # solver out of it rather than into it.
    other = [i for i in range(3) if i != forward]
    along = float(ray @ rotation[:, forward])
    return np.array([math.atan2(float(ray @ rotation[:, other[0]]), along),
                     math.atan2(float(ray @ rotation[:, other[1]]), along)])


def _rotation_of(pose: Any) -> np.ndarray:
    """3x3 rotation of a Pose's quaternion."""
    q = pose.q
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _clamp(value: float, bounds: tuple[float, float] | None) -> float:
    if bounds is None:
        return value
    return max(bounds[0], min(bounds[1], value))
