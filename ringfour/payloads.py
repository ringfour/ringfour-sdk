"""Decode validated array descriptors through a connection-local payload seam.

SHM mmaps local Linux segments read-only. NET consumes tagged binary WebSocket
frames already ordered before their JSON descriptor. It preserves raw array
values and works across hosts and Docker VMs without filesystem access.
"""

from __future__ import annotations

import mmap
from typing import Callable

import numpy as np

from .enums import Transport
from .errors import FrameExpired, RingfourError
from .models.sensing import PayloadRef

SHM_DIR = "/dev/shm"


def _fetch_shm(ref: PayloadRef) -> np.ndarray:
    if "/" in ref.name or ref.name.startswith("."):
        raise RingfourError(f"suspicious shm payload name {ref.name!r}")
    path = f"{SHM_DIR}/{ref.name}"
    try:
        with open(path, "rb") as f:
            mapped = mmap.mmap(f.fileno(), 0, prot=mmap.PROT_READ)
    except FileNotFoundError:
        raise FrameExpired(
            f"payload {ref.name} is gone — the agent expired it before it "
            "was read (past frame_ttl, or evicted under memory pressure)"
        ) from None
    if mapped.size() < ref.size:
        raise RingfourError(
            f"payload {ref.name}: expected {ref.size} bytes, "
            f"segment has {mapped.size()}"
        )
    array = np.frombuffer(mapped, dtype=np.dtype(ref.dtype),
                          count=ref.size // np.dtype(ref.dtype).itemsize)
    return array.reshape(ref.shape)


# Local/custom synchronous fetchers. NET additionally needs its owning connection.
_FETCHERS: dict[Transport, Callable[[PayloadRef], np.ndarray]] = {
    Transport.SHM: _fetch_shm,
}


def fetch_array(ref: PayloadRef, *, connection=None) -> np.ndarray:
    """Materialize a payload descriptor as a (read-only) numpy array.

    Raises FrameExpired if the payload no longer exists, RingfourError
    for an unknown transport (e.g. a remote agent offering a transport
    this client version does not implement).
    """
    if ref.transport == Transport.NET and connection is not None:
        kind, meta, data = connection.take_payload(ref.name)
        if kind != "array/raw" or meta != {"dtype": ref.dtype, "shape": list(ref.shape), "size": ref.size}:
            raise RingfourError("network array header does not match its descriptor")
        return array_from_bytes(ref, data)
    fetcher = _FETCHERS.get(ref.transport)
    if fetcher is None:
        raise RingfourError(
            f"payload transport {ref.transport!r} is not supported by this "
            "client without its originating connection"
        )
    return fetcher(ref)


def register_transport(
    name: Transport, fetcher: Callable[[PayloadRef], np.ndarray]
) -> None:
    """Extension point: install a fetcher for a transport."""
    _FETCHERS[Transport(name)] = fetcher


def array_from_bytes(ref: PayloadRef, data: bytes) -> np.ndarray:
    """Validate raw network bytes before exposing an immutable numeric array."""
    import math
    try:
        dtype = np.dtype(ref.dtype)
        if dtype.kind not in "buif" or len(ref.shape) > 8 or any(n < 0 for n in ref.shape):
            raise ValueError("unsupported numeric layout")
        if math.prod(ref.shape) * dtype.itemsize != ref.size or len(data) != ref.size:
            raise ValueError("shape, dtype and byte count disagree")
        return np.frombuffer(data, dtype=dtype).reshape(ref.shape)
    except (TypeError, ValueError, OverflowError) as exc:
        raise RingfourError(f"invalid network array: {exc}") from exc
