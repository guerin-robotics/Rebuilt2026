from __future__ import annotations

from wpilib_agent_tools.cli import build_parser


def test_sim_parser_defaults_to_concise_mode() -> None:
    parser = build_parser()
    args = parser.parse_args(["sim", "--direct-workspace"])
    assert args.command == "sim"
    assert args.verbose is False
    assert args.max_lines == 120
    assert args.tail is True
    assert args.record is True
    assert args.record_address == "localhost"
    assert args.record_delay == 2.0
    assert args.record_output is None


def test_sim_parser_accepts_assertions() -> None:
    parser = build_parser()
    args = parser.parse_args(
        [
            "sim",
            "--direct-workspace",
            "--assert-key",
            "Shooter/turretUsedUnwindFallback",
            "--assert-range",
            "Shooter/turretResolvedSetpointDeg",
            "-450",
            "630",
        ]
    )
    assert args.assert_key == ["Shooter/turretUsedUnwindFallback"]
    assert args.assert_range == [["Shooter/turretResolvedSetpointDeg", "-450", "630"]]


def test_sim_parser_accepts_recording_flags() -> None:
    parser = build_parser()
    args = parser.parse_args(
        [
            "sim",
            "--direct-workspace",
            "--no-record",
            "--record-address",
            "10.0.0.2",
            "--record-delay",
            "1.25",
            "--record-output",
            "sim_capture.wpilog",
        ]
    )
    assert args.record is False
    assert args.record_address == "10.0.0.2"
    assert args.record_delay == 1.25
    assert args.record_output == "sim_capture.wpilog"


def test_sandbox_run_parser_accepts_output_controls() -> None:
    parser = build_parser()
    args = parser.parse_args(
        [
            "sandbox",
            "run",
            "--name",
            "test",
            "--max-lines",
            "10",
            "--no-tail",
            "--include",
            "WARN|ERROR",
            "--",
            "sim",
            "--duration",
            "5",
        ]
    )
    assert args.sandbox_command == "run"
    assert args.command == ["--", "sim", "--duration", "5"]
    assert args.max_lines == 10
    assert args.tail is False
    assert args.include == "WARN|ERROR"
