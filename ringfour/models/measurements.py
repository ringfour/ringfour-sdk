"""Immutable observations and conditions over discovered numeric signals."""
from typing import Generic, Literal, TypeVar
import math
from pydantic import BaseModel, ConfigDict, field_validator

T = TypeVar('T')
class Measurement(BaseModel, Generic[T]):
    """A value and agent-computed freshness; unknown source age stays explicit."""
    model_config = ConfigDict(frozen=True, extra='forbid', allow_inf_nan=False)
    value: T | None
    robot_id: str
    model_revision: str | None
    source_stamp: float | None = None
    clock_domain: str | None = None
    received_at: float | None = None
    source_age: float | None = None
    receive_age: float | None = None
    validity: Literal['valid', 'unavailable', 'stale', 'invalid']
    reason: str | None = None
    frame: str | None = None

class SignalCondition(BaseModel):
    """A validated signal threshold with a required source/receipt age budget."""
    model_config = ConfigDict(frozen=True, extra='forbid', allow_inf_nan=False)
    robot_id: str
    path: str
    comparison: Literal['below', 'above']
    threshold: float
    max_age: float = .5
    frame: str | None = None
    @field_validator('threshold', 'max_age', mode='before')
    @classmethod
    def finite(cls, value):
        if type(value) not in (int,float) or not math.isfinite(value):
            raise ValueError('condition values must be finite numbers')
        return value
    @field_validator('max_age')
    @classmethod
    def positive(cls, value):
        if value <= 0: raise ValueError('max_age must be positive')
        return value

class GuardStatus(BaseModel):
    """Guard state is independent of operator e-stop and cleanup quarantine."""
    model_config = ConfigDict(frozen=True, extra='forbid', allow_inf_nan=False)
    guard_id: str
    condition: SignalCondition
    tripped: bool
    reason: str | None = None
    observation: Measurement[float]
    cleanup_confirmed: bool | None = None
    evaluation_age: float = 0.0
