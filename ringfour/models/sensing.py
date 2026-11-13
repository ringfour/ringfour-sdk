"""Sensing models and array descriptors shared by SDK and agent.

SHM maps local Linux memory. NET uses tagged binary WebSocket frames ordered
before each descriptor; array dtype, shape and byte count are validated by the
client. Commands remain JSON-RPC objects with either payload transport.
"""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict

from ..enums import Transport


class PayloadRef(BaseModel):
    """Array metadata: name is a SHM segment or connection-local NET payload ID.

    NET preserves exact raw dtype/shape values, including metric float depth;
    it does not apply image compression or infer completion of robot commands.
    """

    model_config = ConfigDict(frozen=True)

    transport: Transport = Transport.SHM
    name: str                  # shm segment name (file name under /dev/shm)
    dtype: str                 # numpy dtype string ("uint8", "float32", ...)
    shape: tuple[int, ...]     # array shape, C-contiguous
    size: int                  # total bytes, == prod(shape) * itemsize


class CameraIntrinsics(BaseModel):
    """Camera calibration (spec section 10), from the camera_info topic."""

    model_config = ConfigDict(frozen=True)

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    distortion_model: str = "plumb_bob"
    distortion_coeffs: tuple[float, ...] = ()
