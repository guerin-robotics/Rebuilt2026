from __future__ import annotations

from pathlib import Path
from typing import Any

from wpiutil import DataLogWriter

from wpilib_agent_tools.commands.sim import _parse_assert_ranges
from wpilib_agent_tools.lib.assertions import evaluate_assertions


def _write_log(path: Path, entries: dict[str, dict[str, object]]) -> Path:
    writer = DataLogWriter(str(path))
    try:
        for key, entry in entries.items():
            if not isinstance(entry, dict):
                continue
            type_str = str(entry.get("type", "raw"))
            data = entry.get("data", [])
            if not isinstance(data, list):
                continue
            entry_id = int(writer.start(key, type_str))
            for item in data:
                if not isinstance(item, list) or len(item) != 2:
                    continue
                timestamp, value = item
                if not isinstance(timestamp, (int, float)):
                    continue
                _append_value(
                    writer=writer,
                    entry_id=entry_id,
                    type_str=type_str,
                    value=value,
                    timestamp_us=max(1, int(float(timestamp) * 1_000_000)),
                )
        writer.flush()
    finally:
        writer.stop()
    return path


def _append_value(*, writer: Any, entry_id: int, type_str: str, value: object, timestamp_us: int) -> None:
    if type_str == "double":
        writer.appendDouble(entry_id, float(value), timestamp_us)
        return
    if type_str == "float":
        writer.appendFloat(entry_id, float(value), timestamp_us)
        return
    if type_str in {"int64", "integer", "int"}:
        writer.appendInteger(entry_id, int(value), timestamp_us)
        return
    if type_str == "boolean":
        writer.appendBoolean(entry_id, bool(value), timestamp_us)
        return
    if type_str == "string":
        writer.appendString(entry_id, str(value), timestamp_us)
        return
    writer.appendRaw(entry_id, str(value).encode("utf-8"), timestamp_us)


def test_parse_assert_ranges_validates_input() -> None:
    parsed = _parse_assert_ranges([["Shooter/Angle", "-450", "630"]])
    assert parsed == [("Shooter/Angle", -450.0, 630.0)]

    try:
        _parse_assert_ranges([["Shooter/Angle", "abc", "630"]])
        assert False, "Expected ValueError"
    except ValueError as exc:
        assert "Invalid --assert-range bounds" in str(exc)

    try:
        _parse_assert_ranges([["Shooter/Angle", "20", "10"]])
        assert False, "Expected ValueError"
    except ValueError as exc:
        assert "min must be <= max" in str(exc)


def test_evaluate_assertions_reports_pass_and_failure(tmp_path: Path) -> None:
    log_path = _write_log(
        tmp_path / "assertions.wpilog",
        {
            "Shooter/turretResolvedSetpointDeg": {
                "type": "double",
                "data": [[0.0, -180.0], [1.0, 200.0], [2.0, 700.0]],
            },
            "Shooter/turretUsedUnwindFallback": {
                "type": "boolean",
                "data": [[0.0, False], [1.0, True]],
            },
        },
    )
    result = evaluate_assertions(
        log_path,
        assert_keys=["Shooter/turretUsedUnwindFallback"],
        assert_ranges=[("Shooter/turretResolvedSetpointDeg", -450.0, 630.0)],
    )
    assert result["passed"] is False
    assert result["summary"]["total"] == 2
    assert result["summary"]["failed"] == 1
    range_check = next(check for check in result["checks"] if check["type"] == "assert_range")
    assert range_check["passed"] is False
    assert range_check["reason"] == "range_violation"
    assert range_check["first_violation"]["value"] == 700.0


def test_evaluate_assertions_missing_key_fails(tmp_path: Path) -> None:
    log_path = _write_log(
        tmp_path / "missing.wpilog",
        {
            "Shooter/Velocity": {
                "type": "double",
                "data": [[0.0, 1000.0], [1.0, 1200.0]],
            }
        },
    )
    result = evaluate_assertions(
        log_path,
        assert_keys=["Shooter/NotPresent"],
        assert_ranges=[("Shooter/NotPresent", 0.0, 1.0)],
    )
    assert result["passed"] is False
    assert result["summary"]["failed"] == 2
