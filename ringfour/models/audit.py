"""Audit log event model (spec section 18)."""

from __future__ import annotations

from typing import Any

from pydantic import BaseModel, ConfigDict, Field

from ..enums import AuditKind, Outcome


class AuditEvent(BaseModel):
    """One audited occurrence on the agent.

    ``stamp`` is ISO 8601 UTC. ``client`` is the connection id that caused
    the event (None for agent-internal events). ``call`` is the wire method
    plus its params with binary payloads elided. ``outcome`` carries the
    reason in ``detail``.
    """

    model_config = ConfigDict(frozen=True)

    stamp: str
    kind: AuditKind
    client: int | None = None
    call: str | None = None
    outcome: Outcome | None = None
    task_id: str | None = None
    detail: dict[str, Any] = Field(default_factory=dict)
