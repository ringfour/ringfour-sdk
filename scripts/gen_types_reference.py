#!/usr/bin/env python3
"""Generate docs/types-reference.md from the source of truth (plan 6.2).

Run from anywhere:  python3 ringfour-sdk/scripts/gen_types_reference.py

This replaces the hand-written ``ringfour-types.md`` (now in archive/).
That document was cited from nine source docstrings, had to be maintained
by hand, and drifted anyway — Phase 6 invalidated a third of it before
anyone noticed. A generated reference cannot drift: it walks the enums,
the pydantic models, the array-bearing dataclasses and the profile schema
and prints exactly what is there.

Regenerate in the same pass as ``protocol.json``; both are committed so a
type change shows up as a documentation diff in review.
"""

from __future__ import annotations

import dataclasses
import enum
import inspect
import re
import sys
import typing
from pathlib import Path

ENGINE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ENGINE_DIR))

from pydantic import BaseModel  # noqa: E402
from pydantic_core import PydanticUndefined  # noqa: E402

from ringfour import data, enums, models  # noqa: E402
from ringfour_agent.models import profile as profile_models  # noqa: E402
from ringfour.version import CLIENT_VERSION, PROTOCOL_VERSION  # noqa: E402

OUT = ENGINE_DIR / "docs" / "types-reference.md"

HEADER = f"""# ringfour type reference

**Generated — do not edit.** Produced by `ringfour-sdk/scripts/gen_types_reference.py`
from the shipped source; regenerate with:

```
python3 ringfour-sdk/scripts/gen_protocol.py
python3 ringfour-sdk/scripts/gen_types_reference.py
```

Client {CLIENT_VERSION}, protocol {PROTOCOL_VERSION}.

Everything here is importable from the `ringfour` package root
(`from ringfour import Pose, Phase, ImageFrame`). The wire schema for the same
types — including JSON-schema `enum` constraints — is `engine/protocol.json`.

> The RGB image type is `ImageFrame`; `Frame` is the enum of frame
> *aliases* (`@map`/`@odom`/`@base`).
"""


def summary(obj: object) -> str:
    """A docstring reflowed to one markdown paragraph per source block."""
    doc = inspect.getdoc(obj)
    if not doc:
        return ""
    blocks = [" ".join(block.split()) for block in doc.split("\n\n")]
    return "\n\n".join(block for block in blocks if block)


def render_type(annotation: object) -> str:
    """A readable, non-`typing`-noisy spelling of an annotation."""
    if annotation is type(None):
        return "None"
    if annotation is Ellipsis:
        return "..."
    if isinstance(annotation, str):
        return annotation
    origin = typing.get_origin(annotation)
    args = typing.get_args(annotation)
    if origin is not None and args:
        if origin is typing.Union or str(origin) == "<class 'types.UnionType'>":
            return " | ".join(render_type(arg) for arg in args)
        if origin is typing.Literal:
            return " | ".join(repr(arg) for arg in args)
        name = getattr(origin, "__name__", str(origin))
        return f"{name}[{', '.join(render_type(arg) for arg in args)}]"
    return getattr(annotation, "__name__", str(annotation)).replace(
        "numpy.ndarray", "ndarray")


def render_default(default: object) -> str:
    if default is PydanticUndefined or default is dataclasses.MISSING:
        return "*required*"
    if isinstance(default, enum.Enum):
        return f"`{default.value!r}`"
    if isinstance(default, (dict, list)) and not default:
        return f"`{default!r}`"
    return f"`{default!r}`"


_COMMENT = re.compile(r"^\s*(\w+)\s*[:=][^#]*#\s*(.+?)\s*$")


def inline_comments(cls: type) -> dict[str, str]:
    """Trailing ``# (H, W, 3) uint8``-style comments, by field name.

    This codebase documents units and array shapes in inline comments
    rather than ``Field(description=)``. Dropping them would make the
    generated reference strictly worse than the hand-written one it
    replaces, so we recover them from the source.
    """
    try:
        source = inspect.getsource(cls)
    except (OSError, TypeError):
        return {}
    found = {}
    for line in source.splitlines():
        match = _COMMENT.match(line)
        if match and match.group(1) not in found:
            found[match.group(1)] = match.group(2)
    return found


def field_table(rows: list[tuple[str, str, str, str]]) -> list[str]:
    if not rows:
        return ["*(no fields)*", ""]
    notes = any(row[3] for row in rows)
    head = "| Field | Type | Default | Notes |" if notes else \
           "| Field | Type | Default |"
    out = [head, "|---|---|---|---|" if notes else "|---|---|---|"]
    for name, type_, default, note in rows:
        cells = f"| `{name}` | `{type_}` | {default} |"
        out.append(f"{cells} {note} |" if notes else cells)
    out.append("")
    return out


def model_rows(model: type[BaseModel]) -> list[tuple[str, str, str, str]]:
    hints = typing.get_type_hints(model)
    comments = inline_comments(model)
    rows = []
    for name, field in model.model_fields.items():
        default = (render_default(field.default)
                   if field.default_factory is None
                   else f"`{field.default_factory()!r}`")
        if field.is_required():
            default = "*required*"
        rows.append((name, render_type(hints.get(name, field.annotation)),
                     default, field.description or comments.get(name, "")))
    return rows


def dataclass_rows(cls: type) -> list[tuple[str, str, str, str]]:
    hints = typing.get_type_hints(cls)
    comments = inline_comments(cls)
    rows = []
    for field in dataclasses.fields(cls):
        if field.default is not dataclasses.MISSING:
            default = render_default(field.default)
        elif field.default_factory is not dataclasses.MISSING:  # type: ignore[misc]
            default = f"`{field.default_factory()!r}`"          # type: ignore[misc]
        else:
            default = "*required*"
        rows.append((field.name, render_type(hints.get(field.name,
                                                       field.type)),
                     default, comments.get(field.name, "")))
    return rows


def methods_of(cls: type) -> list[str]:
    """Public methods and properties, with their one-line summaries."""
    out = []
    for name, member in sorted(vars(cls).items()):
        if name.startswith("_"):
            continue
        if isinstance(member, property):
            doc = summary(member.fget).split("\n\n")[0]
            out.append(f"- `{name}` *(property)* — {doc}" if doc
                       else f"- `{name}` *(property)*")
        elif inspect.isfunction(member):
            signature = str(inspect.signature(member)).replace("self, ", "")
            signature = signature.replace("(self)", "()")
            # PEP 563 leaves every annotation as a quoted string
            signature = signature.replace("'", "").replace('"', "")
            doc = summary(member).split("\n\n")[0]
            out.append(f"- `{name}{signature}` — {doc}" if doc
                       else f"- `{name}{signature}`")
    return out


def section(title: str, level: int = 2) -> str:
    return f"\n{'#' * level} {title}\n"


def build() -> str:
    """The rendered reference. Split out so a test can assert the
    committed file still matches the source it was generated from."""
    lines = [HEADER]

    # -- enums ---------------------------------------------------------------
    lines.append(section("Enums"))
    lines.append(
        "Every member subclasses `str`, so the member **is** the wire value "
        "and the YAML value: `Controller.TRAJECTORY == \"trajectory\"` is "
        "True and bare strings are accepted anywhere a member is.\n")
    for name in enums.__all__:
        cls = getattr(enums, name)
        lines.append(section(f"`{name}`", 3))
        doc = summary(cls)
        if doc:
            lines.append(doc + "\n")
        lines.append("| Member | Value |")
        lines.append("|---|---|")
        for member in cls:
            lines.append(f"| `{name}.{member.name}` | `{member.value!r}` |")
        lines.append("")

    # -- wire models ---------------------------------------------------------
    lines.append(section("Wire models"))
    lines.append(
        "Pydantic models shared by the client and the agent. These are what "
        "`protocol.json` is generated from; they cross the JSON-RPC link "
        "verbatim.\n")
    for name in models.__all__:
        cls = getattr(models, name)
        lines.append(section(f"`{name}`", 3))
        doc = summary(cls)
        if doc:
            lines.append(doc + "\n")
        lines.extend(field_table(model_rows(cls)))
        extras = methods_of(cls)
        if extras:
            lines.append("\n".join(extras) + "\n")

    # -- array-bearing types -------------------------------------------------
    lines.append(section("Array-bearing types"))
    lines.append(summary(data) + "\n")
    for name in sorted(vars(data)):
        cls = getattr(data, name)
        if not (inspect.isclass(cls) and dataclasses.is_dataclass(cls)
                and cls.__module__ == data.__name__):
            continue
        lines.append(section(f"`{name}`", 3))
        doc = summary(cls)
        if doc:
            lines.append(doc + "\n")
        lines.extend(field_table(dataclass_rows(cls)))
        extras = methods_of(cls)
        if extras:
            lines.append("\n".join(extras) + "\n")

    # -- profile schema ------------------------------------------------------
    lines.append(section("Profile schema"))
    lines.append(
        "The YAML a robot ships to describe itself. `RobotProfile` is the "
        "top level; every other class here is one of its sections. A section "
        "being present is what declares the capability (see "
        "`KNOWN_CAPABILITIES`).\n")
    lines.append("**Capabilities:** " + ", ".join(
        f"`{name}`" for name in profile_models.KNOWN_CAPABILITIES) + "\n")
    for name in sorted(vars(profile_models)):
        cls = getattr(profile_models, name)
        if not (inspect.isclass(cls) and issubclass(cls, BaseModel)
                and cls.__module__ == profile_models.__name__):
            continue
        if name.startswith("_"):
            continue
        lines.append(section(f"`{name}`", 3))
        doc = summary(cls)
        if doc:
            lines.append(doc + "\n")
        lines.extend(field_table(model_rows(cls)))

    # Deliberately no generation timestamp: it would make the file churn
    # on every run and defeat the "regenerate and diff" review workflow.
    return "\n".join(lines)


def main() -> None:
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(build())
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
