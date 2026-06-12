"""Canonical package-centered integration asset paths."""

from __future__ import annotations

from pathlib import Path


def _base_dir() -> Path:
    return Path(__file__).resolve().parent


def codex_dir() -> Path:
    return _base_dir() / "codex"


def codex_skill_bundle_dir() -> Path:
    return codex_dir() / "skill_bundle"


def codex_workspace_block() -> Path:
    return codex_dir() / "workspace" / "AGENTS.block.md"


def claude_dir() -> Path:
    return _base_dir() / "claude"


def claude_workspace_block() -> Path:
    return claude_dir() / "workspace" / "CLAUDE.block.md"


def claude_command_template() -> Path:
    return claude_dir() / "commands" / "wpilib-agent-tools-validate.md"


def cursor_dir() -> Path:
    return _base_dir() / "cursor"


def cursor_rules_dir() -> Path:
    return cursor_dir() / "rules"
