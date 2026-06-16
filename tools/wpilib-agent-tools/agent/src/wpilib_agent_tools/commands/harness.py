"""Shared harness installation commands for Codex, Claude Code, and Cursor."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from wpilib_agent_tools.commands import rules
from wpilib_agent_tools.integrations import (
    claude_command_template,
    claude_workspace_block,
    codex_workspace_block,
)

RUNNER_DIR_NAME = ".wpilib-agent-tools"
RUNNER_FILE_NAME = "run_cli.sh"
CODEX_BLOCK_START = "<!-- wpilib-agent-tools:codex:start -->"
CODEX_BLOCK_END = "<!-- wpilib-agent-tools:codex:end -->"
CLAUDE_BLOCK_START = "<!-- wpilib-agent-tools:claude:start -->"
CLAUDE_BLOCK_END = "<!-- wpilib-agent-tools:claude:end -->"
CLAUDE_COMMAND_FILE = "wpilib-agent-tools-validate.md"


def _read_template(template_name: str) -> str:
    if template_name == "codex":
        path = codex_workspace_block()
    elif template_name == "claude":
        path = claude_workspace_block()
    elif template_name == "claude-command":
        path = claude_command_template()
    else:
        raise FileNotFoundError(f"Unknown template: {template_name}")
    if not path.exists():
        raise FileNotFoundError(f"Template not found: {path}")
    return path.read_text(encoding="utf-8")


def _replace_placeholder(text: str, runner_path: str) -> str:
    return text.replace("{{RUNNER_PATH}}", runner_path)


def _parse_harnesses(raw_value: str) -> list[str]:
    value = raw_value.strip().lower()
    if value == "all":
        return ["codex", "claude", "cursor"]
    harnesses = [item.strip().lower() for item in value.split(",") if item.strip()]
    allowed = {"codex", "claude", "cursor"}
    unknown = [item for item in harnesses if item not in allowed]
    if unknown:
        raise ValueError(f"Unknown harnesses: {', '.join(sorted(set(unknown)))}")
    if not harnesses:
        raise ValueError("At least one harness must be selected")
    deduped: list[str] = []
    for harness in harnesses:
        if harness not in deduped:
            deduped.append(harness)
    return deduped


def _runner_script_content(fallback_runner: str | None) -> str:
    fallback_literal = fallback_runner or ""
    return """#!/usr/bin/env bash
set -euo pipefail

if command -v wpilib-agent-tools >/dev/null 2>&1; then
  exec wpilib-agent-tools \"$@\"
fi

FALLBACK_RUNNER={fallback!r}
if [[ -n \"${{FALLBACK_RUNNER}}\" && -x \"${{FALLBACK_RUNNER}}\" ]]; then
  exec \"${{FALLBACK_RUNNER}}\" \"$@\"
fi

echo \"wpilib-agent-tools CLI not found. Install the CLI first.\" >&2
exit 127
""".format(fallback=fallback_literal)


def _write_runner(
    workspace: Path,
    *,
    fallback_runner: str | None,
    force: bool,
    installed: list[str],
    overwritten: list[str],
    skipped: list[dict[str, str]],
) -> str:
    runner_dir = workspace / RUNNER_DIR_NAME
    runner_dir.mkdir(parents=True, exist_ok=True)
    runner_path = runner_dir / RUNNER_FILE_NAME
    runner_str = str(runner_path)

    if runner_path.exists() and not force:
        skipped.append({"path": runner_str, "reason": "exists (use --force to overwrite)"})
    else:
        content = _runner_script_content(fallback_runner)
        if runner_path.exists() and force:
            overwritten.append(runner_str)
        else:
            installed.append(runner_str)
        runner_path.write_text(content, encoding="utf-8")
        runner_path.chmod(runner_path.stat().st_mode | 0o111)

    return f"./{RUNNER_DIR_NAME}/{RUNNER_FILE_NAME}"


def _merge_managed_block(
    file_path: Path,
    *,
    title: str,
    start_marker: str,
    end_marker: str,
    body: str,
) -> None:
    managed_block = f"{start_marker}\n{body.rstrip()}\n{end_marker}\n"
    if not file_path.exists():
        file_path.parent.mkdir(parents=True, exist_ok=True)
        file_path.write_text(f"# {title}\n\n{managed_block}", encoding="utf-8")
        return

    current = file_path.read_text(encoding="utf-8")
    if start_marker in current and end_marker in current:
        start = current.index(start_marker)
        end = current.index(end_marker) + len(end_marker)
        replacement = managed_block.rstrip("\n")
        updated = current[:start].rstrip() + "\n\n" + replacement + current[end:]
        file_path.write_text(updated.rstrip() + "\n", encoding="utf-8")
        return

    updated = current.rstrip() + "\n\n" + managed_block
    file_path.write_text(updated, encoding="utf-8")


def _record_write(path: Path, *, installed: list[str], overwritten: list[str]) -> None:
    path_str = str(path)
    if path.exists():
        overwritten.append(path_str)
    else:
        installed.append(path_str)


def _install_codex(
    workspace: Path,
    *,
    runner_path: str,
    installed: list[str],
    overwritten: list[str],
) -> None:
    body = _replace_placeholder(_read_template("codex"), runner_path)
    agents_path = workspace / "AGENTS.md"
    _record_write(agents_path, installed=installed, overwritten=overwritten)
    _merge_managed_block(
        agents_path,
        title="Agent Instructions",
        start_marker=CODEX_BLOCK_START,
        end_marker=CODEX_BLOCK_END,
        body=body,
    )


def _install_claude(
    workspace: Path,
    *,
    runner_path: str,
    force: bool,
    installed: list[str],
    overwritten: list[str],
    skipped: list[dict[str, str]],
) -> None:
    claude_path = workspace / "CLAUDE.md"
    body = _replace_placeholder(_read_template("claude"), runner_path)
    _record_write(claude_path, installed=installed, overwritten=overwritten)
    _merge_managed_block(
        claude_path,
        title="Claude Code Instructions",
        start_marker=CLAUDE_BLOCK_START,
        end_marker=CLAUDE_BLOCK_END,
        body=body,
    )

    command_dir = workspace / ".claude" / "commands"
    command_dir.mkdir(parents=True, exist_ok=True)
    command_path = command_dir / CLAUDE_COMMAND_FILE
    command_str = str(command_path)
    if command_path.exists() and not force:
        skipped.append({"path": command_str, "reason": "exists (use --force to overwrite)"})
    else:
        content = _replace_placeholder(_read_template("claude-command"), runner_path)
        if command_path.exists() and force:
            overwritten.append(command_str)
        else:
            installed.append(command_str)
        command_path.write_text(content, encoding="utf-8")


def _install_cursor(
    workspace: Path,
    *,
    cursor_mode: str,
    force: bool,
    installed: list[str],
    overwritten: list[str],
    skipped: list[dict[str, str]],
) -> None:
    rules_dir = workspace / ".cursor" / "rules"
    rules_dir.mkdir(parents=True, exist_ok=True)
    selected_templates = rules.MODE_TEMPLATES[cursor_mode]
    for template_name in selected_templates:
        output_name = rules.OUTPUT_FILE_NAMES[template_name]
        output_path = rules_dir / output_name
        output_str = str(output_path)
        content = rules._read_template(template_name)
        if output_path.exists() and not force:
            skipped.append({"path": output_str, "reason": "exists (use --force to overwrite)"})
            continue
        if output_path.exists() and force:
            overwritten.append(output_str)
        else:
            installed.append(output_str)
        output_path.write_text(content, encoding="utf-8")


def register_subparser(subparsers: argparse._SubParsersAction) -> None:
    parser = subparsers.add_parser(
        "harness",
        help="Install workspace support for Codex, Claude Code, and Cursor.",
    )
    harness_subparsers = parser.add_subparsers(dest="harness_command", required=True)

    install_parser = harness_subparsers.add_parser(
        "install", help="Install shared harness support into a workspace."
    )
    install_parser.add_argument("--workspace", required=True, help="Target workspace/repo path.")
    install_parser.add_argument(
        "--harnesses",
        default="all",
        help="Comma-separated harness list (codex,claude,cursor) or 'all'.",
    )
    install_parser.add_argument(
        "--cursor-mode",
        choices=["core", "all"],
        default="core",
        help="Cursor rule install mode.",
    )
    install_parser.add_argument(
        "--fallback-runner",
        help="Optional absolute fallback path to a run_cli.sh wrapper when wpilib-agent-tools is not on PATH.",
    )
    install_parser.add_argument("--force", action="store_true", help="Overwrite generated files.")
    install_parser.add_argument("--json", action="store_true", help="Emit machine-readable JSON output.")
    install_parser.set_defaults(handler=handle_install)


def _emit(payload: dict[str, object], *, as_json: bool) -> None:
    if as_json:
        print(json.dumps(payload, indent=2))
        return

    print(f"Workspace: {payload['workspace']}")
    print(f"Harnesses: {', '.join(payload['harnesses'])}")
    print(f"Runner: {payload['runner_path']}")

    for label in ("installed", "overwritten"):
        items = payload[label]
        if items:
            print(f"{label.capitalize()}:")
            for item in items:  # type: ignore[assignment]
                print(f"  - {item}")
    skipped = payload["skipped"]
    if skipped:
        print("Skipped:")
        for item in skipped:  # type: ignore[assignment]
            print(f"  - {item['path']} ({item['reason']})")


def handle_install(args: argparse.Namespace) -> int:
    try:
        harnesses = _parse_harnesses(args.harnesses)
    except ValueError as exc:
        print(str(exc))
        return 2

    workspace = Path(args.workspace).expanduser().resolve()
    if not workspace.exists() or not workspace.is_dir():
        print(f"Workspace not found: {workspace}")
        return 2

    fallback_runner = None
    if args.fallback_runner:
        fallback_runner = str(Path(args.fallback_runner).expanduser().resolve())

    installed: list[str] = []
    overwritten: list[str] = []
    skipped: list[dict[str, str]] = []
    runner_path = _write_runner(
        workspace,
        fallback_runner=fallback_runner,
        force=args.force,
        installed=installed,
        overwritten=overwritten,
        skipped=skipped,
    )

    if "codex" in harnesses:
        _install_codex(workspace, runner_path=runner_path, installed=installed, overwritten=overwritten)
    if "claude" in harnesses:
        _install_claude(
            workspace,
            runner_path=runner_path,
            force=args.force,
            installed=installed,
            overwritten=overwritten,
            skipped=skipped,
        )
    if "cursor" in harnesses:
        _install_cursor(
            workspace,
            cursor_mode=args.cursor_mode,
            force=args.force,
            installed=installed,
            overwritten=overwritten,
            skipped=skipped,
        )

    payload: dict[str, object] = {
        "workspace": str(workspace),
        "harnesses": harnesses,
        "runner_path": runner_path,
        "installed": installed,
        "overwritten": overwritten,
        "skipped": skipped,
    }
    _emit(payload, as_json=args.json)
    return 0
