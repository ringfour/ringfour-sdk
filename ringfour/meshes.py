"""Reading mesh files well enough to bound them.

Onboarding needs a volume for every `<collision><mesh>`, because a client
holding a description it received over a socket cannot open
`package://...` — and on base101 the mesh links are the jaws, the part
most likely to hit something.

Deliberately not a mesh library. It reads STL, the format ROS robots
actually ship, and returns the axis-aligned box containing the vertices.
That box is what the collision model uses, so a mesh becomes exactly as
capable as a `<box>` primitive and no more. Convex hulls would be tighter
and would need scipy; if a real robot loses reachable configurations to
the bounding box, that is the moment to add them, not before.
"""

from __future__ import annotations

import re
import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np

__all__ = ["Bounds", "bounds_of_stl", "resolve_mesh_path"]


@dataclass(frozen=True)
class Bounds:
    """An ORIENTED box around a mesh, in the mesh's own frame.

    Oriented rather than axis-aligned because the parts that need bounding
    are rarely boxy. base101's moving jaw is a thin curved finger: its
    axis-aligned box is 40% larger than the oriented one, and that slack
    was enough to make the jaw overlap the wrist at the neutral pose and
    report a collision that does not exist.
    """

    centre: tuple[float, float, float]
    half: tuple[float, float, float]
    triangles: int
    # Roll/pitch/yaw of the box relative to the mesh frame, composed with
    # the collision element's own origin by whoever places it.
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)

    @classmethod
    def around(cls, vertices: np.ndarray, triangles: int) -> "Bounds":
        """Fit by principal axes, keeping whichever box is smaller.

        The covariance eigenvectors give the directions the geometry
        actually extends along. For a genuinely boxy part they come out
        near the identity and the two fits agree; for a finger or a
        bracket the oriented one is much tighter. Falling back to the
        axis-aligned fit when it wins keeps the result from ever being
        worse than the simple answer.
        """
        low, high = vertices.min(axis=0), vertices.max(axis=0)
        aligned_half = (high - low) / 2.0
        aligned = cls(tuple(float(v) for v in (low + high) / 2.0),
                      tuple(float(v) for v in aligned_half), triangles)
        if len(vertices) < 4:
            return aligned

        mean = vertices.mean(axis=0)
        centred = vertices - mean
        try:
            _, axes = np.linalg.eigh(np.cov(centred.T))
        except np.linalg.LinAlgError:          # pragma: no cover
            return aligned
        if np.linalg.det(axes) < 0:            # keep it right-handed
            axes[:, 2] *= -1.0
        local = centred @ axes
        lo, hi = local.min(axis=0), local.max(axis=0)
        half = (hi - lo) / 2.0
        if float(np.prod(half)) >= float(np.prod(aligned_half)):
            return aligned
        centre = mean + axes @ ((lo + hi) / 2.0)
        return cls(tuple(float(v) for v in centre),
                   tuple(float(v) for v in half), triangles,
                   _rpy_of(axes))

    def scaled(self, scale: tuple[float, float, float]) -> "Bounds":
        """URDF `<mesh scale=>` applies to the geometry, so it applies
        here too. Negative scales mirror; the box is symmetric so only the
        magnitude matters.

        Note a non-uniform scale does not commute with a rotation. ROS
        meshes are uniformly scaled in practice (base101 uses 0.001 for
        millimetres), and a non-uniform one is rare enough that skewing
        the fit slightly is better than refusing to bound the part.
        """
        return Bounds(
            tuple(c * s for c, s in zip(self.centre, scale)),
            tuple(abs(h * s) for h, s in zip(self.half, scale)),
            self.triangles, self.rpy,
        )


def _rpy_of(matrix: np.ndarray) -> tuple[float, float, float]:
    """3x3 rotation -> URDF roll/pitch/yaw (fixed-axis XYZ)."""
    import math

    pitch = math.asin(max(-1.0, min(1.0, -float(matrix[2, 0]))))
    if abs(matrix[2, 0]) < 0.999999:
        roll = math.atan2(float(matrix[2, 1]), float(matrix[2, 2]))
        yaw = math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))
    else:                                     # gimbal lock
        roll = math.atan2(-float(matrix[1, 2]), float(matrix[1, 1]))
        yaw = 0.0
    return (roll, pitch, yaw)


def bounds_of_stl(path: str | Path) -> Bounds:
    """Bounding box of an STL, binary or ASCII.

    Raises ValueError if the file is neither. STL carries no units; ROS
    convention is metres and a `scale` attribute handles the rest.
    """
    data = Path(path).read_bytes()
    if not data:
        raise ValueError(f"{path} is empty")
    # An ASCII STL starts with "solid", but so do some binary files whose
    # header happens to. The reliable test is the length: a binary STL is
    # exactly 84 + 50 * count bytes.
    if len(data) >= 84:
        count = struct.unpack_from("<I", data, 80)[0]
        if len(data) == 84 + count * 50 and count > 0:
            return _binary(data, count)
    text = data.decode("utf-8", errors="replace")
    if "facet" in text or text.lstrip().startswith("solid"):
        return _ascii(text)
    raise ValueError(f"{path} is not an STL file")


def _binary(data: bytes, count: int) -> Bounds:
    # Each triangle is a 12-float record (normal + 3 vertices) plus a
    # 2-byte attribute. Read them all at once rather than in a loop:
    # a 100k-triangle mesh is common and onboarding reads several.
    dtype = np.dtype([
        ("normal", "<3f4"),
        ("vertices", "<3,3f4"),
        ("attribute", "<u2"),
    ])
    records = np.frombuffer(data, dtype=dtype, count=count, offset=84)
    vertices = records["vertices"].reshape(-1, 3).astype(np.float64)
    return Bounds.around(vertices, count)


_VERTEX = re.compile(
    r"vertex\s+(-?[\d.eE+-]+)\s+(-?[\d.eE+-]+)\s+(-?[\d.eE+-]+)")


def _ascii(text: str) -> Bounds:
    found = _VERTEX.findall(text)
    if not found:
        raise ValueError("ASCII STL contains no vertices")
    vertices = np.asarray(found, dtype=np.float64)
    return Bounds.around(vertices, len(found) // 3)


def resolve_mesh_path(uri: str, share: "dict[str, str] | None" = None
                      ) -> Path | None:
    """A URDF mesh filename as a real path, or None if it cannot be found.

    Handles the three spellings a URDF uses: `package://pkg/rest`,
    `file:///abs`, and a bare path. ``share`` maps package name to its
    share directory; onboarding fills it from the ament index, which is
    why this runs on the robot and not on a laptop.
    """
    if uri.startswith("file://"):
        return Path(uri[7:])
    if uri.startswith("package://"):
        rest = uri[len("package://"):]
        package, _, tail = rest.partition("/")
        root = (share or {}).get(package)
        if root is None:
            return None
        return Path(root) / tail
    path = Path(uri)
    return path if path.is_absolute() else None
