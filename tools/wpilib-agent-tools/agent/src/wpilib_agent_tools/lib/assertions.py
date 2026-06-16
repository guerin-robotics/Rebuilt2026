"""Assertion helpers for simulation validation."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from wpilib_agent_tools.lib.log_reader import LogReader


def _as_numeric(value: Any) -> float | None:
    if isinstance(value, bool):
        return 1.0 if value else 0.0
    if isinstance(value, (int, float)):
        return float(value)
    return None


def evaluate_assertions(
    log_path: str | Path,
    *,
    assert_keys: list[str],
    assert_ranges: list[tuple[str, float, float]],
) -> dict[str, Any]:
    """Evaluate key-presence and numeric-range assertions against a log."""
    reader = LogReader(log_path)
    checks: list[dict[str, Any]] = []

    for key in assert_keys:
        points = reader.read_key_points(key)
        passed = len(points) > 0
        check: dict[str, Any] = {
            "type": "assert_key",
            "key": key,
            "passed": passed,
            "sample_count": len(points),
        }
        if not passed:
            check["reason"] = "missing_key_or_no_samples"
        checks.append(check)

    for key, min_value, max_value in assert_ranges:
        points = reader.read_key_points(key)
        numeric_points = [
            (timestamp, numeric)
            for timestamp, raw_value in points
            if (numeric := _as_numeric(raw_value)) is not None
        ]
        if not points:
            checks.append(
                {
                    "type": "assert_range",
                    "key": key,
                    "min": min_value,
                    "max": max_value,
                    "passed": False,
                    "sample_count": 0,
                    "numeric_sample_count": 0,
                    "reason": "missing_key_or_no_samples",
                }
            )
            continue
        if not numeric_points:
            checks.append(
                {
                    "type": "assert_range",
                    "key": key,
                    "min": min_value,
                    "max": max_value,
                    "passed": False,
                    "sample_count": len(points),
                    "numeric_sample_count": 0,
                    "reason": "no_numeric_samples",
                }
            )
            continue

        values = [value for _, value in numeric_points]
        observed_min = min(values)
        observed_max = max(values)
        first_violation: dict[str, Any] | None = None
        out_of_range = 0
        for timestamp, value in numeric_points:
            if min_value <= value <= max_value:
                continue
            out_of_range += 1
            if first_violation is None:
                first_violation = {"timestamp": timestamp, "value": value}

        passed = out_of_range == 0
        check = {
            "type": "assert_range",
            "key": key,
            "min": min_value,
            "max": max_value,
            "passed": passed,
            "sample_count": len(points),
            "numeric_sample_count": len(numeric_points),
            "observed_min": observed_min,
            "observed_max": observed_max,
            "out_of_range_samples": out_of_range,
        }
        if first_violation is not None:
            check["first_violation"] = first_violation
        if not passed:
            check["reason"] = "range_violation"
        checks.append(check)

    failed = sum(1 for check in checks if not check.get("passed"))
    passed = len(checks) - failed
    return {
        "log_path": str(reader.path),
        "passed": failed == 0,
        "summary": {"total": len(checks), "passed": passed, "failed": failed},
        "checks": checks,
    }
