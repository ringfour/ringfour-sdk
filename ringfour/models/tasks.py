"""Task lifecycle model (spec sections 5 and 23).

Long-running calls return a task_id immediately; progress arrives as
``task.update`` notifications carrying TaskUpdate. Terminal states are
"succeeded", "failed", "cancelled".
"""

from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, ConfigDict

TASK_STATES = ("pending", "running", "succeeded", "failed", "cancelled")
TERMINAL_STATES = ("succeeded", "failed", "cancelled")


class DispatchReceipt(BaseModel):
    """A vendor command was dispatched; physical completion is unverified."""

    model_config = ConfigDict(frozen=True)
    action: str
    dispatched: bool
    completion: Literal["unverified"] = "unverified"


class TaskError(BaseModel):
    """Why a task failed. ``type`` names a ringfour exception class;
    ``data`` carries its extras (reason, limit, ...)."""

    model_config = ConfigDict(frozen=True)

    type: str
    message: str
    data: dict[str, Any] = {}


class TaskUpdate(BaseModel):
    """One ``task.update`` notification."""

    model_config = ConfigDict(frozen=True)

    task_id: str
    sequence: int = 0
    report_age: float = 0.0  # agent-computed age of the retained report
    cleanup: dict[str, Any] | None = None
    state: str  # one of TASK_STATES
    progress: float | None = None  # 0..1 when the task can estimate it
    message: str | None = None
    # Task-kind-specific live feedback (navigation: distance_remaining,
    # pose, ...). The client builds typed status objects from it.
    data: dict[str, Any] | None = None
    result: dict[str, Any] | None = None  # set on "succeeded"
    error: TaskError | None = None        # set on "failed"
