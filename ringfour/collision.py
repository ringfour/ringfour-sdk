"""Self-collision, derived from the URDF and nothing else.

**This is a planning aid, not a safety clamp** (Cristi, 2026-08-25). It
exists so `kin.ik` and `kin.reachable` know where the robot cannot go and
decline to put it there — not to stop a mission that commands raw joint
vectors. Guarantees before motion are spec 8.7's job and need the real
collision work; this refuses to pretend otherwise.

**It must work for any robot from its URDF.** No per-robot file, nothing
to check in, nothing to regenerate. That rules out MoveIt's approach,
where the allowed-collision matrix is authored by a setup assistant and
baked into the SRDF — the one you have to rebuild whenever the robot
changes. The algorithm behind it is mechanical, so it is computed here at
build time from the live description instead. Since `model.changed`
already fires when `robot_state_publisher` republishes (6.3), the model is
rebuilt when the robot changes and cannot drift from it.

**What is checked.** Every link's `<collision>` volumes, reduced to
oriented bounding boxes, tested pairwise with the separating-axis theorem.
Cylinders and spheres become the box that contains them: conservative, and
it keeps the whole thing to one intersection test. Refine per-primitive
only if a real robot loses reachable configurations to it.

**What is NOT checked, and says so.** A link whose collision geometry is a
mesh has no extent here — the file lives on the robot and a client holding
a description it received over a socket cannot open `package://...`. Those
links are reported by `unmodelled`, never silently treated as empty. On
base101 that is all three jaw links, which is exactly the part most likely
to hit something: `CollisionModel.unmodelled` is how a mission finds out.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Iterator

import numpy as np

from .description import CollisionShape, RobotDescription

__all__ = ["CollisionModel", "CollisionPair"]

# Two boxes that overlap by less than this are treated as touching, not
# colliding. Adjacent parts in a CAD-exported URDF routinely share a face
# to a few microns; without a margin every such pair reads as a permanent
# collision and the ACM disables it, which is worse - it would hide a real
# one later.
_TOLERANCE = 1e-4

# Configurations sampled when deriving the allowed-collision matrix. Pairs
# that collide in ALL of them are structural (they are built touching);
# pairs that collide in none are unreachable and can be skipped for speed.
# 64 is enough to separate those two populations on a 5-DOF arm and costs
# a fraction of a second.
_ACM_SAMPLES = 64


@dataclass(frozen=True)
class CollisionPair:
    """Two links found overlapping, and by how much."""

    a: str
    b: str
    depth: float          # smallest overlap along any separating axis, m

    def __str__(self) -> str:
        return f"{self.a} <-> {self.b} ({self.depth * 1000:.1f} mm)"


class _Box:
    """An oriented box: centre, half-extents, and a rotation, in link frame."""

    __slots__ = ("link", "half", "offset", "rotation", "radius")

    def __init__(self, shape: CollisionShape) -> None:
        self.link = shape.link
        self.half = np.asarray(shape.half, dtype=float)
        self.offset = np.asarray(shape.origin_xyz, dtype=float)
        self.rotation = _rpy(shape.origin_rpy)
        # Bounding sphere about the box's own centre, for the broadphase.
        # Two boxes whose centres are further apart than the sum of these
        # cannot touch, and that is one subtraction instead of fifteen
        # projections. Most pairs on a robot are far apart most of the
        # time, so this is where the time goes.
        self.radius = float(np.linalg.norm(self.half))

    def placed(self, link_pose: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(centre, 3x3 axes) in the base frame, given the link's pose."""
        rotation = link_pose[:3, :3] @ self.rotation
        centre = link_pose[:3, :3] @ self.offset + link_pose[:3, 3]
        return centre, rotation


def _rpy(rpy: tuple[float, float, float]) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def _overlap(centre_a: np.ndarray, axes_a: np.ndarray, half_a: np.ndarray,
             centre_b: np.ndarray, axes_b: np.ndarray, half_b: np.ndarray
             ) -> float:
    """Penetration depth of two OBBs, or 0.0 if they are apart.

    Separating-axis theorem: two convex boxes are disjoint if and only if
    some axis separates them, and only 15 axes need testing — the three
    face normals of each box plus the nine cross products of their edges.
    Returning the smallest overlap rather than a bool costs nothing and
    makes the tolerance meaningful.
    """
    translation = centre_b - centre_a
    # Everything in A's frame.
    t = axes_a.T @ translation
    rotation = axes_a.T @ axes_b
    absolute = np.abs(rotation)

    smallest = math.inf
    for i in range(3):                       # A's face normals
        reach = half_a[i] + float(absolute[i] @ half_b)
        gap = reach - abs(float(t[i]))
        if gap <= 0.0:
            return 0.0
        smallest = min(smallest, gap)
    for i in range(3):                       # B's face normals
        reach = float(absolute[:, i] @ half_a) + half_b[i]
        gap = reach - abs(float(rotation[:, i] @ t))
        if gap <= 0.0:
            return 0.0
        smallest = min(smallest, gap)
    for i in range(3):                       # edge-edge cross products
        for j in range(3):
            # The axis is a_i x b_j, whose length is sin of the angle
            # between them. When the two edges are PARALLEL that length is
            # zero and the axis does not exist: the formula below still
            # evaluates, to a reach and a distance both near zero, and
            # reports a gap of ~1e-10. Taking that as the penetration
            # depth made every axis-aligned overlap look like a 0.1 nm
            # touch, which the tolerance then discarded — so two boxes at
            # the SAME CENTRE read as clear. The face-normal tests above
            # already cover the parallel case; skip it here.
            cross = math.sqrt(max(0.0, 1.0 - float(rotation[i, j]) ** 2))
            if cross < 1e-6:
                continue
            i1, i2 = (i + 1) % 3, (i + 2) % 3
            j1, j2 = (j + 1) % 3, (j + 2) % 3
            reach = (half_a[i1] * absolute[i2, j] + half_a[i2] * absolute[i1, j]
                     + half_b[j1] * absolute[i, j2]
                     + half_b[j2] * absolute[i, j1])
            distance = abs(float(t[i2] * rotation[i1, j]
                                 - t[i1] * rotation[i2, j]))
            gap = (reach - distance) / cross
            if gap <= 0.0:
                return 0.0
            smallest = min(smallest, gap)
    return float(smallest)


class CollisionModel:
    """Which link pairs may collide, and whether they do at a pose.

    Build once per description (it is cached by sha in `kinematics`), then
    call `check` with link poses from FK.
    """

    def __init__(self, description: RobotDescription,
                 boxes: dict[str, list[_Box]],
                 pairs: tuple[tuple[str, str], ...],
                 unmodelled: tuple[str, ...],
                 always: tuple[tuple[str, str], ...]) -> None:
        self._description = description
        self._boxes = boxes
        self.pairs = pairs
        # Links whose geometry this model cannot see (meshes). Exposed
        # rather than hidden: on base101 these are the jaws, and a mission
        # planning a grasp deserves to know they are not being checked.
        self.unmodelled = unmodelled
        # Pairs disabled because they collide in every sampled
        # configuration - they are built touching. Kept for inspection;
        # a surprising entry here usually means a bad URDF origin.
        self.always_touching = always

    # -- construction ---------------------------------------------------------

    @classmethod
    def build(cls, description: RobotDescription,
              joints: Iterable[str] = (),
              samples: int = _ACM_SAMPLES) -> "CollisionModel":
        """Derive the model, including its allowed-collision matrix.

        ``joints`` are the ones that move for the purposes of sampling
        (a group's, usually). With none given, every moving joint in the
        description is sampled.
        """
        boxes: dict[str, list[_Box]] = {}
        unmodelled: list[str] = []
        for link, shapes in description.collisions.items():
            usable = [_Box(shape) for shape in shapes if shape.modelled]
            if usable:
                boxes[link] = usable
            if any(not shape.modelled for shape in shapes):
                unmodelled.append(link)

        movable = tuple(joints) or tuple(
            name for name, joint in description.joints.items() if joint.moves)
        candidates = cls._candidates(description, boxes, movable)
        always = cls._structural(description, boxes, candidates, movable,
                                 samples)
        pairs = tuple(pair for pair in candidates if pair not in set(always))
        return cls(description, boxes, pairs, tuple(sorted(unmodelled)),
                   tuple(always))

    @staticmethod
    def _bodies(description: RobotDescription) -> dict[str, str]:
        """link -> rigid-body id, collapsing every chain of FIXED joints.

        A URDF exported from CAD splits one physical part into many links:
        a bracket, the servo bolted into it, the adapter on its face, each
        its own link joined by fixed joints. They cannot move relative to
        each other, and their collision boxes routinely interpenetrate by
        a couple of millimetres because that is how the parts actually
        fit. Treating them as one body is both physically right and what
        makes the pair list meaningful.
        """
        parent: dict[str, str] = {link: link for link in description.links}

        def find(link: str) -> str:
            while parent[link] != link:
                parent[link] = parent[parent[link]]
                link = parent[link]
            return link

        for joint in description.joints.values():
            if joint.moves:
                continue
            a, b = find(joint.parent), find(joint.child)
            if a != b:
                parent[a] = b
        return {link: find(link) for link in description.links}

    @classmethod
    def _candidates(cls, description: RobotDescription,
                    boxes: dict[str, list[_Box]],
                    movable: tuple[str, ...]) -> tuple[tuple[str, str], ...]:
        """Pairs worth testing at all.

        Three exclusions, all structural rather than heuristic:

        * **same rigid body** — links joined only by fixed joints cannot
          move relative to each other, so the answer never changes;
        * **bodies either side of one moving joint** — the two halves of a
          joint assembly are built touching, and no joint value separates
          them. This is the generalisation of MoveIt's "adjacent" rule to
          a URDF that splits parts across fixed joints, and it is what
          removes base101's two worst offenders: elbow_adapter vs
          elbow_link (72% of configurations, never deeper than 3 mm) and
          wrist_roll_adapter vs wrist_servo_adapter;
        * **neither link moves** — a constant relative pose, so a genuine
          overlap is a URDF modelling error, not something a mission can
          steer around.
        """
        bodies = cls._bodies(description)
        # Bodies connected by exactly one moving joint.
        neighbours = {
            frozenset((bodies[joint.parent], bodies[joint.child]))
            for joint in description.joints.values() if joint.moves
        }
        movable_set = set(movable)
        moves = {
            link: any(name in movable_set
                      for name in description.chain_to(link))
            for link in boxes
        }
        out: list[tuple[str, str]] = []
        names = sorted(boxes)
        for index, a in enumerate(names):
            for b in names[index + 1:]:
                if bodies[a] == bodies[b]:
                    continue
                if frozenset((bodies[a], bodies[b])) in neighbours:
                    continue
                if not moves[a] and not moves[b]:
                    continue
                out.append((a, b))
        return tuple(out)

    @classmethod
    def _structural(cls, description: RobotDescription,
                    boxes: dict[str, list[_Box]],
                    candidates: tuple[tuple[str, str], ...],
                    movable: tuple[str, ...],
                    samples: int) -> tuple[tuple[str, str], ...]:
        """Pairs that collide in EVERY sampled configuration.

        MoveIt's setup assistant calls these "always in collision" and
        writes them into the SRDF. They are geometry that overlaps by
        construction — a servo body modelled slightly inside its bracket,
        say. Reporting them would drown every real result, and no joint
        value fixes them.

        Sampling is deterministic (fixed seed) so the model built from one
        description is always the same model. That matters: `ik` consults
        this, and `ik`'s contract is determinism.
        """
        if not candidates:
            return ()
        limits = [description.joints[name].limits for name in movable]
        rng = np.random.default_rng(0)
        counts = {pair: 0 for pair in candidates}
        for _ in range(samples):
            q = {}
            for name, bound in zip(movable, limits):
                if bound is None:
                    q[name] = float(rng.uniform(-math.pi, math.pi))
                else:
                    q[name] = float(rng.uniform(bound[0], bound[1]))
            poses = link_poses(description, q, tuple(boxes))
            for pair in cls._hits(boxes, poses, candidates):
                counts[pair.a, pair.b] += 1
        return tuple(pair for pair, hits in counts.items() if hits == samples)

    # -- checking -------------------------------------------------------------

    @staticmethod
    def _hits(boxes: dict[str, list[_Box]], poses: dict[str, np.ndarray],
              pairs: Iterable[tuple[str, str]]) -> Iterator[CollisionPair]:
        for a, b in pairs:
            pose_a, pose_b = poses.get(a), poses.get(b)
            if pose_a is None or pose_b is None:
                continue
            for box_a in boxes[a]:
                centre_a, axes_a = box_a.placed(pose_a)
                for box_b in boxes[b]:
                    centre_b, axes_b = box_b.placed(pose_b)
                    apart = centre_b - centre_a
                    if (float(apart @ apart)
                            > (box_a.radius + box_b.radius) ** 2):
                        continue             # broadphase: cannot touch
                    depth = _overlap(centre_a, axes_a, box_a.half,
                                     centre_b, axes_b, box_b.half)
                    if depth > _TOLERANCE:
                        yield CollisionPair(a, b, depth)
                        break
                else:
                    continue
                break

    def check(self, poses: dict[str, np.ndarray]
              ) -> tuple[CollisionPair, ...]:
        """Every colliding pair at these link poses, deepest first."""
        found = list(self._hits(self._boxes, poses, self.pairs))
        found.sort(key=lambda pair: pair.depth, reverse=True)
        return tuple(found)

    @property
    def links(self) -> tuple[str, ...]:
        """Links this model can see."""
        return tuple(sorted(self._boxes))


def link_poses(description: RobotDescription, q: dict[str, float],
               links: Iterable[str]) -> dict[str, np.ndarray]:
    """4x4 base-frame pose of each named link at configuration ``q``.

    Memoised over the tree rather than walked from the root per link.
    Both give the same answer; the difference is that base101 has 51
    links whose chains are ~15 joints long, so the naive version does
    around 765 matrix products per call and this one does 62. The
    reachability sampler calls it once per sample, hundreds of thousands
    of times, which is what makes the distinction worth the cache.
    """
    from .kinematics import _joint_matrix, _origin_matrix

    cache: dict[str, np.ndarray] = {}

    def pose_of(link: str) -> np.ndarray | None:
        known = cache.get(link)
        if known is not None:
            return known
        step = description._parent_of.get(link)
        if step is None:                     # the root
            cache[link] = np.eye(4)
            return cache[link]
        joint = description.joints[step]
        parent = pose_of(joint.parent)
        if parent is None:                   # pragma: no cover
            return None
        matrix = parent @ _origin_matrix(joint)
        if joint.moves:
            matrix = matrix @ _joint_matrix(joint, q.get(step, 0.0))
        cache[link] = matrix
        return matrix

    out: dict[str, np.ndarray] = {}
    for link in links:
        if link not in description.links:
            continue
        placed = pose_of(link)
        if placed is not None:
            out[link] = placed
    return out
