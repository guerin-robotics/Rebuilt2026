from __future__ import annotations

import struct
from pathlib import Path

import pytest

from wpilib_agent_tools.lib.log_reader import LogReader

DataLogWriter = pytest.importorskip("wpiutil").DataLogWriter


def _write_raw_struct_log(path: Path) -> None:
    log = DataLogWriter(str(path))
    schema_entry = int(log.start("/AdvantageKit/.schema/struct:ChassisSpeeds", "raw"))
    value_entry = int(log.start("/AdvantageKit/RealOutputs/RobotState/fieldRelativeSpeeds", "raw"))
    log.appendRaw(schema_entry, b"schema", 10)
    log.appendRaw(value_entry, struct.pack("<ddd", 0.1, -0.2, 0.3), 1_000_000)
    log.appendRaw(value_entry, struct.pack("<ddd", 0.5, 1.0, -0.4), 3_000_000)
    log.flush()
    log.stop()


def _write_raw_pose3d_log(path: Path) -> None:
    log = DataLogWriter(str(path))
    schema_entry = int(log.start("/AdvantageKit/.schema/struct:Pose3d", "raw"))
    value_entry = int(log.start("/AdvantageKit/RealOutputs/RobotState/pose3d", "raw"))
    log.appendRaw(schema_entry, b"schema", 10)
    log.appendRaw(value_entry, struct.pack("<ddddddd", 1.0, -2.0, 0.5, 0.9, 0.1, 0.2, 0.3), 1_500_000)
    log.flush()
    log.stop()


def test_log_reader_decodes_raw_chassis_speed_using_schema_hint(tmp_path: Path) -> None:
    log_path = tmp_path / "raw_struct.wpilog"
    _write_raw_struct_log(log_path)

    points = LogReader(log_path).read_key_points("/AdvantageKit/RealOutputs/RobotState/fieldRelativeSpeeds")
    assert len(points) == 2
    assert points[0][1]["type"] == "ChassisSpeeds"
    assert points[0][1]["vx_mps"] == pytest.approx(0.1)
    assert points[0][1]["vy_mps"] == pytest.approx(-0.2)
    assert points[0][1]["omega_radps"] == pytest.approx(0.3)


def test_log_reader_decodes_raw_pose3d_using_schema_hint(tmp_path: Path) -> None:
    log_path = tmp_path / "raw_pose3d.wpilog"
    _write_raw_pose3d_log(log_path)

    points = LogReader(log_path).read_key_points("/AdvantageKit/RealOutputs/RobotState/pose3d")
    assert len(points) == 1
    value = points[0][1]
    assert value["type"] == "Pose3d"
    assert value["translation"]["x_meters"] == pytest.approx(1.0)
    assert value["translation"]["y_meters"] == pytest.approx(-2.0)
    assert value["translation"]["z_meters"] == pytest.approx(0.5)
    assert value["rotation"]["quaternion"]["w"] == pytest.approx(0.9)
    assert value["rotation"]["quaternion"]["x"] == pytest.approx(0.1)
    assert value["rotation"]["quaternion"]["y"] == pytest.approx(0.2)
    assert value["rotation"]["quaternion"]["z"] == pytest.approx(0.3)


def test_log_summary_uses_data_sample_window(tmp_path: Path) -> None:
    log_path = tmp_path / "summary_window.wpilog"
    _write_raw_struct_log(log_path)

    summary = LogReader(log_path).get_summary()
    assert summary.sample_count == 3
    assert summary.duration_sec == pytest.approx(2.99999, abs=0.01)
