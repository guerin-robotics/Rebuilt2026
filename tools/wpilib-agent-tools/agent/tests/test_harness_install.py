from __future__ import annotations

import argparse
import json
from pathlib import Path

import pytest

from wpilib_agent_tools.cli import build_parser
from wpilib_agent_tools.commands import harness


def _args(**overrides: object) -> argparse.Namespace:
    base: dict[str, object] = {
        "workspace": None,
        "harnesses": "all",
        "cursor_mode": "core",
        "fallback_runner": None,
        "force": False,
        "json": True,
    }
    base.update(overrides)
    return argparse.Namespace(**base)


def test_harness_cli_parser_registers_install_command() -> None:
    parser = build_parser()
    args = parser.parse_args(["harness", "install", "--workspace", "/tmp/workspace"])
    assert args.command == "harness"
    assert args.harness_command == "install"
    assert args.harnesses == "all"


def test_harness_install_all_creates_codex_claude_and_cursor_assets(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    workspace = tmp_path / "robot"
    workspace.mkdir()

    exit_code = harness.handle_install(
        _args(workspace=str(workspace), fallback_runner="/tmp/fallback-runner")
    )

    assert exit_code == 0
    payload = json.loads(capsys.readouterr().out)
    installed_names = {Path(path).name for path in payload["installed"]}
    assert "run_cli.sh" in installed_names
    assert "AGENTS.md" in installed_names
    assert "CLAUDE.md" in installed_names
    assert "wpilib-agent-tools-validate.md" in installed_names
    assert "wpilib-agent-tools-core.mdc" in installed_names

    runner_path = workspace / ".wpilib-agent-tools" / "run_cli.sh"
    runner_text = runner_path.read_text(encoding="utf-8")
    assert "wpilib-agent-tools" in runner_text
    assert "/tmp/fallback-runner" in runner_text

    agents_text = (workspace / "AGENTS.md").read_text(encoding="utf-8")
    assert "wpilib-agent-tools:codex:start" in agents_text
    assert "./.wpilib-agent-tools/run_cli.sh" in agents_text

    claude_text = (workspace / "CLAUDE.md").read_text(encoding="utf-8")
    assert "wpilib-agent-tools:claude:start" in claude_text
    assert "./.wpilib-agent-tools/run_cli.sh" in claude_text

    command_text = (workspace / ".claude" / "commands" / "wpilib-agent-tools-validate.md").read_text(
        encoding="utf-8"
    )
    assert "./.wpilib-agent-tools/run_cli.sh" in command_text

    cursor_rule = workspace / ".cursor" / "rules" / "wpilib-agent-tools-core.mdc"
    assert cursor_rule.exists()


def test_harness_install_updates_managed_blocks_without_force(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    workspace = tmp_path / "robot"
    workspace.mkdir()
    agents_path = workspace / "AGENTS.md"
    agents_path.write_text("# Existing\n\nLocal notes\n", encoding="utf-8")
    claude_path = workspace / "CLAUDE.md"
    claude_path.write_text("# Existing Claude\n", encoding="utf-8")

    exit_code = harness.handle_install(_args(workspace=str(workspace), harnesses="codex,claude"))

    assert exit_code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["skipped"] == []
    assert "Local notes" in agents_path.read_text(encoding="utf-8")
    assert "wpilib-agent-tools:codex:start" in agents_path.read_text(encoding="utf-8")
    assert "wpilib-agent-tools:claude:start" in claude_path.read_text(encoding="utf-8")


def test_harness_install_skips_existing_generated_files_without_force(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    workspace = tmp_path / "robot"
    (workspace / ".wpilib-agent-tools").mkdir(parents=True)
    (workspace / ".wpilib-agent-tools" / "run_cli.sh").write_text("manual", encoding="utf-8")
    (workspace / ".claude" / "commands").mkdir(parents=True)
    command_path = workspace / ".claude" / "commands" / "wpilib-agent-tools-validate.md"
    command_path.write_text("manual", encoding="utf-8")
    (workspace / ".cursor" / "rules").mkdir(parents=True)
    rule_path = workspace / ".cursor" / "rules" / "wpilib-agent-tools-core.mdc"
    rule_path.write_text("manual", encoding="utf-8")

    exit_code = harness.handle_install(_args(workspace=str(workspace), force=False))

    assert exit_code == 0
    payload = json.loads(capsys.readouterr().out)
    skipped_paths = {item["path"] for item in payload["skipped"]}
    assert str(command_path) in skipped_paths
    assert str(rule_path) in skipped_paths
    assert str(workspace / ".wpilib-agent-tools" / "run_cli.sh") in skipped_paths
    assert command_path.read_text(encoding="utf-8") == "manual"
    assert rule_path.read_text(encoding="utf-8") == "manual"


def test_harness_install_rejects_unknown_harness(capsys: pytest.CaptureFixture[str]) -> None:
    exit_code = harness.handle_install(_args(workspace="/tmp", harnesses="codex,unknown", json=False))
    assert exit_code == 2
    assert "Unknown harnesses" in capsys.readouterr().out
