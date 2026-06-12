from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

from wpiutil import DataLogWriter

from wpilib_agent_tools.commands.query import _run_single_query
from wpilib_agent_tools.lib.log_reader import LogReader


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


def _ds_args(start: float | None = None) -> argparse.Namespace:
    return argparse.Namespace(
        file=None,
        key=None,
        mode="ds",
        start=start,
        end=None,
        limit=None,
        json=True,
    )


def test_ds_mode_resolves_prefixed_and_slash_keys(tmp_path: Path) -> None:
    log_path = _write_log(
        tmp_path / "prefixed.wpilog",
        {
            "/DriverStation/Enabled": {"type": "boolean", "data": [[0.0, False], [1.0, True]]},
            "/AdvantageKit/DriverStation/Autonomous": {"type": "boolean", "data": [[0.0, True]]},
            "/AdvantageKit/DriverStation/Test": {"type": "boolean", "data": [[0.0, False]]},
            "/DriverStation/AllianceStation": {"type": "string", "data": [[0.0, "Blue1"]]},
            "/AdvantageKit/DriverStation/MatchTime": {"type": "double", "data": [[0.0, 15.0], [1.0, 14.0]]},
        },
    )

    result = _run_single_query(LogReader(log_path), _ds_args(start=1.0))

    assert result["state"]["DriverStation/Enabled"] is True
    assert result["state"]["DriverStation/Autonomous"] is True
    assert result["state"]["DriverStation/Test"] is False
    assert result["state"]["DriverStation/AllianceStation"] == "Blue1"
    assert result["state"]["DriverStation/MatchTime"] == 14.0
    assert result["resolved_keys"]["DriverStation/Autonomous"] == "/AdvantageKit/DriverStation/Autonomous"


def test_ds_mode_keeps_bare_driverstation_keys(tmp_path: Path) -> None:
    log_path = _write_log(
        tmp_path / "bare.wpilog",
        {
            "DriverStation/Enabled": {"type": "boolean", "data": [[0.0, True]]},
            "DriverStation/Autonomous": {"type": "boolean", "data": [[0.0, False]]},
            "DriverStation/Test": {"type": "boolean", "data": [[0.0, False]]},
            "DriverStation/AllianceStation": {"type": "string", "data": [[0.0, "Red2"]]},
            "DriverStation/MatchTime": {"type": "double", "data": [[0.0, 12.5]]},
        },
    )

    result = _run_single_query(LogReader(log_path), _ds_args())

    assert result["state"]["DriverStation/Enabled"] is True
    assert result["state"]["DriverStation/Autonomous"] is False
    assert result["state"]["DriverStation/Test"] is False
    assert result["state"]["DriverStation/AllianceStation"] == "Red2"
    assert result["state"]["DriverStation/MatchTime"] == 12.5
    assert result["resolved_keys"]["DriverStation/Enabled"] == "DriverStation/Enabled"
