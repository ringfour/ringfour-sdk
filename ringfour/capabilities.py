"""Capability gating and singular-sugar resolution (spec section 4).

Shared by the sync and aio clients so both raise identical errors.
"""

from __future__ import annotations

from typing import Mapping, TypeVar

from .errors import AmbiguousCapability, CapabilityNotSupported

T = TypeVar("T")


def require(capabilities: frozenset[str], capability: str,
            robot: str, profile: str) -> None:
    """Raise CapabilityNotSupported (never AttributeError) if the robot's
    profile does not provide ``capability``."""
    if capability not in capabilities:
        raise CapabilityNotSupported(
            f"robot {robot!r} (profile {profile!r}) has no "
            f"{capability!r} capability",
            robot=robot,
        )


def single(instances: Mapping[str, T], plural: str, robot: str) -> T:
    """Resolve singular sugar: the only instance, or AmbiguousCapability
    telling the user which names exist."""
    if len(instances) == 1:
        return next(iter(instances.values()))
    if not instances:
        # Reachable now that a capability can exist with none of a given
        # instance: a profile may declare `groups:` with no group of kind
        # arm. "use robot.arms[]" would be nonsense advice.
        raise AmbiguousCapability(
            f"robot {robot!r} has no {plural}", robot=robot)
    names = " | ".join(f"'{name}'" for name in instances)
    raise AmbiguousCapability(
        f"robot {robot!r} has {len(instances)} {plural}; "
        f"use robot.{plural}[{names}]",
        robot=robot,
    )
