"""Output helpers for command modules."""

from __future__ import annotations

import json
from collections import deque
from typing import Any


def dumps_json(payload: Any, *, compact: bool = False) -> str:
    """Serialize payload as pretty or compact JSON."""
    if compact:
        return json.dumps(payload, separators=(",", ":"))
    return json.dumps(payload, indent=2)


def emit(payload: Any, *, as_json: bool, compact_json: bool = False) -> None:
    """Print structured payload either as JSON or repr-like text."""
    if as_json:
        print(dumps_json(payload, compact=compact_json))
        return
    print(payload)


def bound_lines(
    lines: list[str],
    *,
    max_lines: int | None,
    tail: bool,
) -> tuple[list[str], dict[str, Any]]:
    """Bound a list of lines and return metadata about truncation."""
    total = len(lines)
    if max_lines is None or max_lines <= 0 or total <= max_lines:
        return lines, {"total_lines": total, "returned_lines": total, "truncated": False, "tail": tail}
    if tail:
        bounded = list(deque(lines, maxlen=max_lines))
    else:
        bounded = lines[:max_lines]
    return bounded, {"total_lines": total, "returned_lines": len(bounded), "truncated": True, "tail": tail}


def format_size_bytes(size: int) -> str:
    """Human-friendly byte formatting."""
    units = ["B", "KB", "MB", "GB", "TB"]
    value = float(size)
    for unit in units:
        if value < 1024 or unit == units[-1]:
            return f"{value:.2f} {unit}"
        value /= 1024.0
    return f"{size} B"
