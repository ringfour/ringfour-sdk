"""Reusable geometry preparation; mesh resolution belongs to the environment adapter."""
from pathlib import Path
import logging
import math
import numpy as np
from .description import CollisionShape, RobotDescription
from .meshes import Bounds, bounds_of_stl, _rpy_of
log = logging.getLogger(__name__)

def _rotation(rpy: tuple[float, float, float]) -> np.ndarray:
    """URDF fixed-axis roll/pitch/yaw as a 3x3."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def measure_meshes(description: RobotDescription,
                   resolve) -> dict[str, tuple[CollisionShape, ...]]:
    """Bounding volumes for every `<collision><mesh>` in the description.

    A mesh that cannot be found or read is skipped and logged rather than
    guessed at — an invented volume is worse than a missing one, because
    the missing one is reported by `CollisionModel.unmodelled`.
    """
    hulls: dict[str, list[CollisionShape]] = {}
    for link, shapes in description.collisions.items():
        for shape in shapes:
            if shape.modelled or not shape.mesh:
                continue
            path = resolve(shape.mesh)
            if path is None or not path.exists():
                log.warning("%s: cannot resolve %s", link, shape.mesh)
                continue
            try:
                bounds: Bounds = bounds_of_stl(path).scaled(shape.scale)
            except (ValueError, OSError) as exc:
                log.warning("%s: cannot read %s (%s)", link, path, exc)
                continue
            # Two frames compose here and getting it wrong puts the box
            # somewhere the part is not. The <collision><origin> places
            # the MESH frame inside the link; the fitted box sits at its
            # own centre and orientation inside the MESH frame.
            outer = _rotation(shape.origin_rpy)
            centre = tuple(
                float(v) for v in
                np.asarray(shape.origin_xyz)
                + outer @ np.asarray(bounds.centre))
            hulls.setdefault(link, []).append(CollisionShape(
                link=link, kind="box", half=bounds.half,
                origin_xyz=centre,
                origin_rpy=_rpy_of(outer @ _rotation(bounds.rpy)),
            ))
            log.info("  %-26s %5d triangles -> half %s", link,
                     bounds.triangles,
                     tuple(round(v, 4) for v in bounds.half))
    return {link: tuple(shapes) for link, shapes in hulls.items()}

