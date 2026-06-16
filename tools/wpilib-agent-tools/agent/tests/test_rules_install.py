from __future__ import annotations

import argparse
import json
from pathlib import Path

import pytest

from wpilib_agent_tools.cli import build_parser
from wpilib_agent_tools.commands import rules


def _args(**overrides: object) -> argparse.Namespace:
    base: dict[str, object] = {
        "mode": "core",
        "target": "custom",
        "output_dir": None,
        "force": False,
        "json": True,
    }
    base.update(overrides)
    return argparse.Namespace(**base)


def test_rules_cli_parser_registers_install_command() -> None:
    parser = build_parser()
    args = parser.parse_args(
        [
            "rules",
            "install",
            "--mode",
            "core",
            "--target",
            "custom",
            "--output-dir",
            "/tmp/rules",
        ]
    )
    assert args.command == "rules"
    assert args.rules_command == "install"
    assert args.mode == "core"


def test_rules_install_custom_target_installs_core_template(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    output_dir = tmp_path / "rules"
    exit_code = rules.handle_install(_args(mode="core", output_dir=str(output_dir), json=True))

    assert exit_code == 0
    payload = json.loads(capsys.readouterr().out)
    installed_names = sorted(Path(path).name for path in payload["installed"])
    assert installed_names == ["wpilib-agent-tools-core.mdc"]
    assert payload["overwritten"] == []
    assert payload["skipped"] == []

    core_text = (output_dir / "wpilib-agent-tools-core.mdc").read_text(encoding="utf-8")
    assert "WPILib Agent Tools" in core_text


def test_rules_install_skips_existing_without_force(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    output_dir = tmp_path / "rules"
    output_dir.mkdir(parents=True, exist_ok=True)
    core_path = output_dir / "wpilib-agent-tools-core.mdc"
    core_path.write_text("manual-content", encoding="utf-8")

    exit_code = rules.handle_install(_args(mode="core", output_dir=str(output_dir), force=False, json=True))

    assert exit_code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["installed"] == []
    assert payload["overwritten"] == []
    assert len(payload["skipped"]) == 1
    assert payload["skipped"][0]["path"] == str(core_path)
    assert "use --force to overwrite" in payload["skipped"][0]["reason"]
    assert core_path.read_text(encoding="utf-8") == "manual-content"


def test_rules_install_overwrites_existing_with_force(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    output_dir = tmp_path / "rules"
    output_dir.mkdir(parents=True, exist_ok=True)
    core_path = output_dir / "wpilib-agent-tools-core.mdc"
    core_path.write_text("manual-content", encoding="utf-8")

    exit_code = rules.handle_install(_args(mode="core", output_dir=str(output_dir), force=True, json=True))

    assert exit_code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["installed"] == []
    assert payload["skipped"] == []
    assert payload["overwritten"] == [str(core_path)]
    assert "WPILib Agent Tools" in core_path.read_text(encoding="utf-8")


def test_rules_install_custom_target_requires_output_dir(capsys: pytest.CaptureFixture[str]) -> None:
    exit_code = rules.handle_install(_args(target="custom", output_dir=None, json=False))
    assert exit_code == 2
    assert "--output-dir is required when --target custom is used" in capsys.readouterr().out


def test_rule_template_keeps_critical_safety_guidance() -> None:
    core_text = rules._read_template(rules.CORE_TEMPLATE_FILE)

    # Sandbox workflow
    assert "sandbox create --name <id> --source workspace" in core_text
    assert "explicit user approval" in core_text

    # Execution safety
    assert "--json" in core_text
    assert "bounded" in core_text

    # Evidence collection
    assert "acceptance criteria" in core_text
    assert "minimum run set" in core_text

    # Pre-sim gate (mode + DS state)
    assert "DriverStationSim.setAutonomous(true);" in core_text
    assert "DriverStationSim.setTest(true);" in core_text
    assert "DriverStationSim.setEnabled(true);" in core_text
    assert "DriverStationSim.notifyNewData();" in core_text
    assert "Constants.java" in core_text
    assert "MUST" in core_text
    assert "blocker" in core_text
    assert "do not proceed" in core_text

    # Self-discovery
    assert "wpilib-agent-tools --help" in core_text
