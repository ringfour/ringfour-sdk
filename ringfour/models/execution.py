"""Immutable execution values: identity, timing and native-unit evidence."""
from __future__ import annotations
import math
from enum import Enum
from types import MappingProxyType
from typing import Generic, Literal, TypeVar
from pydantic import BaseModel, ConfigDict, field_validator, model_validator
from .spatial import Pose


class _Value(BaseModel):
    model_config = ConfigDict(frozen=True, extra='forbid', allow_inf_nan=False)


class JointValue(_Value):
    """One model-bound joint value in radians or metres, with model bounds."""
    name: str
    position: float
    unit: Literal['rad', 'm']
    lower: float | None = None
    upper: float | None = None

    @field_validator('position', 'lower', 'upper', mode='before')
    @classmethod
    def numeric(cls, value):
        if value is None:return value
        if type(value) not in (int,float) or not math.isfinite(value):raise ValueError('joint values must be finite numbers')
        return value

    @model_validator(mode='after')
    def bounded(self):
        if not self.name:raise ValueError('joint name is required')
        if (self.lower is None)!=(self.upper is None):raise ValueError('both joint bounds are required')
        if self.lower is not None and not self.lower<=self.position<=self.upper:raise ValueError(f'{self.name}: value exceeds model bounds')
        return self


class JointConfiguration(_Value):
    """Exact named joint coverage tied to one robot, group and model revision."""
    robot: str
    group: str
    model_revision: str
    joints: tuple[JointValue,...]

    @model_validator(mode='after')
    def identity(self):
        if not all((self.robot,self.group,self.model_revision,self.joints)):raise ValueError('configuration needs identity and joint values')
        if len({j.name for j in self.joints})!=len(self.joints):raise ValueError('duplicate joint names')
        return self

    @property
    def positions(self):return MappingProxyType({j.name:j.position for j in self.joints})

    def vector(self, names):
        if set(names)!=set(self.positions):raise ValueError('configuration has different joint coverage')
        return [self.positions[n] for n in names]


class JointTrajectory(_Value):
    """Model-bound configurations at strictly increasing finite times from start.

    Endpoint validation does not establish interpolation speed/acceleration safety.
    """
    configurations: tuple[JointConfiguration,...]
    times: tuple[float,...]

    @field_validator('times', mode='before')
    @classmethod
    def finite_times(cls, values):
        if not values or any(type(v) not in (int,float) or not math.isfinite(v) for v in values):raise ValueError('trajectory times must be finite numbers')
        return values

    @model_validator(mode='after')
    def coherent(self):
        if not self.configurations or len(self.configurations)!=len(self.times):raise ValueError('one time per configuration is required')
        if self.times[0]<=0 or any(b<=a for a,b in zip(self.times,self.times[1:])):raise ValueError('trajectory times must strictly increase from zero')
        first=self.configurations[0]
        signature=lambda q:(q.robot,q.group,q.model_revision,sorted((j.name,j.unit,j.lower,j.upper) for j in q.joints))
        if any(signature(q)!=signature(first) for q in self.configurations[1:]):raise ValueError('trajectory configurations have different identities or joint models')
        return self

    @classmethod
    def from_waypoints(cls, configurations, *, step_duration=2.):
        if type(step_duration) not in (int,float) or not math.isfinite(step_duration) or step_duration<=0:raise ValueError('step_duration must be a positive finite number')
        configurations=tuple(configurations)
        return cls(configurations=configurations,times=tuple(step_duration*(i+1) for i in range(len(configurations))))


from ..enums import OperationState

class CancelReceipt(_Value):
    """Cancellation request acknowledgement, never evidence of measured rest."""
    operation_id:str
    accepted:bool
    already_terminal:bool


class StopReport(_Value):
    """Measured software stopping criterion; hardware commissioning is separate."""
    resource:str
    verified:bool
    criterion:str
    physical_stop:Literal['unverified','uncommissioned']
    elapsed:float | None=None
    dwell:float | None=None


class JointResidual(_Value):
    """One measured endpoint residual with its own unit and acceptance tolerance."""
    name:str
    error:float
    unit:Literal['rad','m']
    tolerance:float


class NavigationResult(_Value):
    """Verified arrival at a measured framed pose, not controller acknowledgement."""
    operation_id:str
    final_pose:Pose
    position_error:float
    orientation_error:float
    position_tolerance:float
    orientation_tolerance:float
    stop:StopReport

    @model_validator(mode='after')
    def verified_arrival(self):
        if min(self.position_error,self.orientation_error,self.position_tolerance,self.orientation_tolerance)<0:
            raise ValueError('arrival errors/tolerances must be nonnegative')
        if self.position_error>self.position_tolerance or self.orientation_error>self.orientation_tolerance:
            raise ValueError('arrival errors exceed declared tolerances')
        if not self.stop.verified:raise ValueError('arrival requires verified stopping')
        return self


class TrajectoryResult(_Value):
    """Complete measured joint endpoint residuals and stopping evidence."""
    operation_id:str
    configuration:JointConfiguration
    residuals:tuple[JointResidual,...]
    stop:StopReport


class NavigationProgress(_Value):
    """Available navigation feedback; distance is an estimate, not completion."""
    pose:Pose | None=None
    remaining_distance:float | None=None


class TrajectoryProgress(_Value):
    """Measured joint positions when the controller provides complete feedback."""
    positions:tuple[JointValue,...] | None=None
    elapsed_fraction:float | None=None
    fraction_basis:Literal['elapsed_time'] | None=None


TProgress=TypeVar('TProgress')
class OperationSnapshot(_Value,Generic[TProgress]):
    """Immutable cached state; receipt age and sequence make staleness explicit."""
    operation_id:str
    state:OperationState
    sequence:int
    observed_age:float
    progress:TProgress | None=None

class RouteResult(_Value):
    """Verified arrival evidence for every waypoint in a finite route."""
    operation_id:str
    arrivals:tuple[NavigationResult,...]

class RouteProgress(_Value):
    """Route index and available navigation feedback; no inferred arrival."""
    waypoint_index:int
    waypoint_count:int
    navigation:NavigationProgress | None=None

class ToolResult(_Value):
    """Measured positioning evidence. This does not prove an object is held."""
    operation_id:str
    group:str
    joint:str
    position:float
    unit:Literal['rad','m']
    target:float
    residual:float
    tolerance:float
    stop:StopReport

class ToolProgress(_Value):
    position:float | None=None
    unit:Literal['rad','m']
