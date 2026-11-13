"""ringfour exception hierarchy (spec section 20).

Every error carries ``robot`` (robot name, if known), ``call`` (the wire
method that failed, if any) and ``request_id`` (the JSON-RPC id) so failures
are traceable in the agent's audit log.

Errors raised by the agent travel over the wire as JSON-RPC error objects
whose ``data["type"]`` names the exception class; ``error_from_wire``
rebuilds the right Python exception on the client side.
"""

from __future__ import annotations

from typing import Any


class RingfourError(Exception):
    """Base class for every ringfour error."""

    def __init__(
        self,
        message: str,
        *,
        robot: str | None = None,
        call: str | None = None,
        request_id: int | str | None = None,
        details: dict[str, Any] | None = None,
    ) -> None:
        super().__init__(message)
        self.robot = robot
        self.call = call
        self.request_id = request_id
        self.details = dict(details or {})
        self.wire_type = self.details.get("type", type(self).__name__)


class ConnectionFailed(RingfourError):
    """Could not establish a connection to the agent."""


class ConnectionLost(RingfourError):
    """The connection to the agent dropped while in use."""


class ProtocolMismatch(RingfourError):
    """Client and agent speak incompatible protocol versions."""


class CapabilityNotSupported(RingfourError):
    """The robot's profile does not provide this capability."""


class AmbiguousCapability(RingfourError):
    """Singular accessor used where several instances exist (e.g. robot.arm
    on a two-armed robot)."""


class SafetyViolation(RingfourError):
    """Rejected by the agent's server-side safety layer.

    ``reason`` is a short machine-readable cause (e.g. "geofence",
    "teleop active"); ``limit`` is the violated limit value, when numeric.
    """

    def __init__(self, message: str, *, reason: str | None = None,
                 limit: float | None = None, **kwargs: Any) -> None:
        super().__init__(message, **kwargs)
        self.reason = reason
        self.limit = limit


class FrameMismatch(RingfourError):
    """Spatial math attempted across two different reference frames."""


class FrameExpired(RingfourError):
    """Deprojection requested with a stale frame handle (past frame_ttl)."""


class TimeoutError(RingfourError):  # noqa: A001 - name mandated by the spec
    """A blocking call exceeded its timeout."""


class NavigationFailed(RingfourError):
    """Navigation ended without reaching the goal.

    ``reason`` is one of "no_path" | "aborted" | "stuck".
    """

    def __init__(self, message: str, *, reason: str | None = None,
                 **kwargs: Any) -> None:
        super().__init__(message, **kwargs)
        self.reason = reason


class Unreachable(RingfourError):
    """No IK solution for a pose (spec 26.1).

    Raised, never returned as None: a membership test over a list of
    waypoints cannot say WHICH pose failed, and that is the only thing
    the caller needs to know. The message names the group, the tip, the
    pose and the constrained DOF.
    """




class ExecutionFailed(RingfourError):
    """Controller-level failure mid-trajectory."""


class PermissionDenied(RingfourError):
    """The connection principal lacks the command's required permission."""


class CleanupFailed(RingfourError):
    """Session cleanup failed; details retain the report and quarantine."""


class StopUnverified(RingfourError):
    """Measured stop criteria could not be established within their budget."""


class CompletionUnverified(ExecutionFailed):
    """Required fresh measured evidence did not establish completion."""


class ResourceBusy(SafetyViolation):
    """The requested physical resource already has accepted work."""




class UnknownPlace(RingfourError):
    """A named place that does not exist on the active map (spec
    section 17): either never saved, or saved on a different map."""


class StateUnavailable(RingfourError):
    """Required measured state/provenance is unavailable."""

class StateStale(RingfourError):
    """Measured source or receive age exceeds the requested budget."""

class ModelChanged(RingfourError):
    """Command or session belongs to a different robot-model revision."""

class OperationCancelled(RingfourError):
    """Operation ended through verified cancellation; no success result exists."""

class WaitTimeout(TimeoutError):
    """An observation/result wait expired; execution continues."""

class OperationTimeout(TimeoutError):
    """A blocking action timed out and attempted bounded stopping/cleanup."""

class SubmissionUnknown(ConnectionLost):
    """A mutating reply was lost; reconcile command_id before any retry."""

class CommandRequired(RingfourError):
    """A mutating submission needs a valid client-generated command ID."""

class CommandConflict(RingfourError):
    """The command ID is already bound to a different request."""

class CommandExpired(RingfourError):
    """The command ID cannot execute or is outside query retention."""

class AgentEpochChanged(RingfourError):
    """An earlier boot's command cannot execute in the current epoch."""

_ERRORS_BY_NAME: dict[str, type[RingfourError]] = {
    cls.__name__: cls
    for cls in (
        ConnectionFailed, ConnectionLost, ProtocolMismatch,
        SubmissionUnknown, CommandRequired, CommandConflict, CommandExpired, AgentEpochChanged,
        CapabilityNotSupported, AmbiguousCapability, SafetyViolation,
        FrameMismatch, FrameExpired, TimeoutError, NavigationFailed,
        ExecutionFailed, UnknownPlace,
        CleanupFailed, CompletionUnverified, ResourceBusy, PermissionDenied, StopUnverified,
        ModelChanged, OperationCancelled, WaitTimeout, OperationTimeout, StateUnavailable, StateStale,
    )
}


def error_from_wire(message: str, data: dict[str, Any] | None) -> RingfourError:
    """Rebuild a ringfour exception from a JSON-RPC error object.

    ``data["type"]`` selects the class (RingfourError if absent or unknown).
    Known context keys (robot, call, request_id, reason, limit) are passed
    through to the exception; everything else is ignored.
    """
    data = data or {}
    cls = _ERRORS_BY_NAME.get(data.get("type", ""), RingfourError)
    kwargs: dict[str, Any] = {
        "robot": data.get("robot"),
        "call": data.get("call"),
        "request_id": data.get("request_id"),
        "details": data,
    }
    if cls in (SafetyViolation, NavigationFailed):
        kwargs["reason"] = data.get("reason")
    if cls is SafetyViolation:
        kwargs["limit"] = data.get("limit")
    return cls(message, **kwargs)
