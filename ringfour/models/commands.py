"""Reconciliation records retain evidence without resuming interrupted work."""
from typing import Any, Literal
from pydantic import BaseModel, ConfigDict
from .tasks import TaskUpdate


class CommandRecord(BaseModel):
    """Durable acceptance reply, or explicit unknown outcome after a crash."""
    model_config = ConfigDict(frozen=True, extra='forbid')
    command_id: str
    epoch: str
    method: str
    state: Literal['pending', 'completed', 'unknown']
    result: Any = None
    error: dict[str, Any] | None = None
    operation_id: str | None = None
    expires_at: float
    retained_until: float


class OperationRecord(BaseModel):
    """Queryable terminal evidence; active work at a crash remains unknown."""
    model_config = ConfigDict(frozen=True, extra='forbid')
    operation_id: str
    epoch: str
    state: Literal['active', 'terminal', 'unknown']
    snapshot: TaskUpdate
    retained_until: float | None = None
    physical_outcome: Literal['recorded', 'unknown']
