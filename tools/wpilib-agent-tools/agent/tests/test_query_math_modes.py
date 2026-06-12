from __future__ import annotations

import argparse
import math
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


def _args(**overrides: object) -> argparse.Namespace:
    base: dict[str, object] = {
        "file": None,
        "key": "Shooter/Velocity",
        "mode": "stats",
        "start": None,
        "end": None,
        "limit": None,
        "window": 3,
        "expr": None,
        "above": None,
        "below": None,
        "min_duration": 0.0,
        "top": 5,
        "setpoint": None,
        "setpoint_key": None,
        "tolerance": 0.02,
        "json": True,
    }
    base.update(overrides)
    return argparse.Namespace(**base)


def test_query_new_modes_stats_smooth_threshold_rms_expr(tmp_path: Path) -> None:
    log_path = _write_log(
        tmp_path / "modes.wpilog",
        {
            "Shooter/Velocity": {
                "type": "double",
                "data": [[0.0, 0.0], [1.0, 1000.0], [2.0, 1500.0], [3.0, 1200.0], [4.0, 1500.0]],
            },
            "Shooter/Setpoint": {
                "type": "double",
                "data": [[0.0, 1000.0], [1.0, 1000.0], [2.0, 1500.0], [3.0, 1500.0], [4.0, 1500.0]],
            },
            "PDH/Current": {
                "type": "double",
                "data": [[0.0, 20.0], [1.0, 45.0], [2.0, 46.0], [3.0, 25.0], [4.0, 50.0], [5.0, 51.0]],
            },
        },
    )
    reader = LogReader(log_path)

    stats = _run_single_query(reader, _args(mode="stats"))
    assert stats["stats"] is not None
    assert stats["stats"]["count"] == 5
    assert stats["stats"]["p95"] >= stats["stats"]["p50"]

    smooth = _run_single_query(reader, _args(mode="smooth", window=2))
    assert len(smooth["values"]) == 5
    assert smooth["values"][1][1] == 500.0

    threshold = _run_single_query(
        reader,
        _args(mode="threshold", key="PDH/Current", above=40.0, min_duration=0.5),
    )
    assert len(threshold["events"]) == 2
    assert threshold["events"][0]["peak_value"] == 46.0

    rms = _run_single_query(reader, _args(mode="rms"))
    assert isinstance(rms["rms"], float)
    assert rms["rms"] > 0.0

    expr = _run_single_query(
        reader,
        _args(
            mode="expr",
            expr="{Shooter/Setpoint} - {Shooter/Velocity}",
            key=None,
        ),
    )
    assert expr["count"] == 5
    assert expr["values"][2][1] == 0.0


def test_query_fft_and_settle_modes(tmp_path: Path) -> None:
    wave_samples: list[list[float]] = []
    dt = 0.02
    for idx in range(200):
        timestamp = idx * dt
        value = math.sin(2.0 * math.pi * 2.0 * timestamp)
        wave_samples.append([timestamp, value])

    log_path = _write_log(
        tmp_path / "fft_settle.wpilog",
        {
            "Oscillation": {"type": "double", "data": wave_samples},
            "Arm/Position": {
                "type": "double",
                "data": [[0.0, 0.0], [1.0, 200.0], [2.0, 1300.0], [3.0, 1490.0], [4.0, 1505.0], [5.0, 1498.0]],
            },
            "Arm/Setpoint": {
                "type": "double",
                "data": [[0.0, 0.0], [1.0, 0.0], [2.0, 1500.0], [3.0, 1500.0], [4.0, 1500.0], [5.0, 1500.0]],
            },
        },
    )
    reader = LogReader(log_path)

    fft = _run_single_query(reader, _args(mode="fft", key="Oscillation", top=3))
    assert fft["components"]
    assert abs(fft["components"][0]["frequency_hz"] - 2.0) < 0.25

    settle = _run_single_query(
        reader,
        _args(
            mode="settle",
            key="Arm/Position",
            setpoint_key="Arm/Setpoint",
            tolerance=0.02,
        ),
    )
    metrics = settle["settle"]
    assert metrics is not None
    assert metrics["rise_time"] is not None
    assert metrics["settling_time"] is not None
    assert metrics["steady_state_error"] < 5.0
