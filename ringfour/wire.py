"""JSON-RPC 2.0 framing helpers, shared by client and agent.

Boring by design: every text frame is a single JSON-RPC 2.0 object readable
with websocat. Binary payloads (images, scans, grids) are announced by a
``payload.header`` notification and then sent as one binary WebSocket frame
whose first 8 bytes are the payload id (big-endian unsigned), followed by
the raw bytes (spec section 23).

This module is pure: no sockets, no ROS, no state.
"""

from __future__ import annotations

import json
import struct
from typing import Any

JSONRPC_VERSION = "2.0"

# Standard JSON-RPC 2.0 error codes.
ERR_PARSE = -32700
ERR_INVALID_REQUEST = -32600
ERR_METHOD_NOT_FOUND = -32601
ERR_INVALID_PARAMS = -32602
# Application errors: data["type"] names the ringfour exception class.
ERR_APPLICATION = -32000

# Binary frame layout: 8-byte big-endian payload id, then raw bytes.
_PAYLOAD_TAG = struct.Struct(">Q")
PAYLOAD_TAG_BYTES = _PAYLOAD_TAG.size

# Method name of the notification announcing a binary payload.
PAYLOAD_HEADER_METHOD = "payload.header"


def request(request_id: int, method: str, params: dict[str, Any] | None) -> str:
    """Encode a JSON-RPC request."""
    msg: dict[str, Any] = {
        "jsonrpc": JSONRPC_VERSION,
        "id": request_id,
        "method": method,
    }
    if params is not None:
        msg["params"] = params
    return json.dumps(msg)


def response(request_id: int | str | None, result: Any) -> str:
    """Encode a successful JSON-RPC response."""
    return json.dumps(
        {"jsonrpc": JSONRPC_VERSION, "id": request_id, "result": result}
    )


def error_response(
    request_id: int | str | None,
    code: int,
    message: str,
    data: dict[str, Any] | None = None,
) -> str:
    """Encode a JSON-RPC error response."""
    error: dict[str, Any] = {"code": code, "message": message}
    if data is not None:
        error["data"] = data
    return json.dumps(
        {"jsonrpc": JSONRPC_VERSION, "id": request_id, "error": error}
    )


def notification(method: str, params: dict[str, Any] | None = None) -> str:
    """Encode a server-pushed JSON-RPC notification (no id, no reply)."""
    msg: dict[str, Any] = {"jsonrpc": JSONRPC_VERSION, "method": method}
    if params is not None:
        msg["params"] = params
    return json.dumps(msg)


def payload_header(payload_id: int, kind: str, meta: dict[str, Any]) -> str:
    """Encode the notification that announces a binary payload.

    ``kind`` says what the bytes are ("image/jpeg", "depth/f32", ...);
    ``meta`` carries kind-specific fields (camera, stamp, shape, ...).
    """
    return notification(
        PAYLOAD_HEADER_METHOD,
        {"payload_id": payload_id, "kind": kind, "meta": meta},
    )


def encode_payload_frame(payload_id: int, data: bytes) -> bytes:
    """Prefix raw payload bytes with their 8-byte payload id tag."""
    return _PAYLOAD_TAG.pack(payload_id) + data


def decode_payload_frame(frame: bytes) -> tuple[int, bytes]:
    """Split a binary frame into (payload_id, data).

    Raises ValueError if the frame is shorter than the id tag.
    """
    if len(frame) < PAYLOAD_TAG_BYTES:
        raise ValueError(
            f"binary frame too short: {len(frame)} < {PAYLOAD_TAG_BYTES} bytes"
        )
    (payload_id,) = _PAYLOAD_TAG.unpack_from(frame)
    return payload_id, frame[PAYLOAD_TAG_BYTES:]


def parse_message(text: str) -> dict[str, Any]:
    """Parse one incoming text frame into a dict.

    Raises ValueError if it is not a JSON object with jsonrpc == "2.0".
    Callers decide whether it is a request, response, or notification by
    inspecting the "id" / "method" / "result" / "error" keys.
    """
    msg = json.loads(text)
    if not isinstance(msg, dict) or msg.get("jsonrpc") != JSONRPC_VERSION:
        raise ValueError("not a JSON-RPC 2.0 message")
    return msg
