from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import pytest

from wpilib_agent_tools.commands.sandbox import _prepare_command
from wpilib_agent_tools.lib import sandbox_manager
from wpilib_agent_tools.lib.sandbox_manager import SandboxError, SandboxManager


def test_prepare_command_maps_rules_to_internal_cli() -> None:
    prepared = _prepare_command(["--", "rules", "install", "--mode", "core"])
    assert prepared[:4] == [sys.executable, "-m", "wpilib_agent_tools", "rules"]
    assert prepared[4:] == ["install", "--mode", "core"]


def test_prepare_command_keeps_external_commands() -> None:
    prepared = _prepare_command(["python3", "-V"])
    assert prepared == ["python3", "-V"]


def test_manager_run_wraps_missing_command_as_sandbox_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    workspace = tmp_path / "workspace"
    workspace.mkdir(parents=True)
    (workspace / "README.md").write_text("demo\n", encoding="utf-8")
    monkeypatch.setenv("HOME", str(tmp_path))
    manager = SandboxManager(workspace_root=workspace)
    manager.create(name="demo", source_spec="workspace", force=True)

    def fake_popen(*_args: Any, **_kwargs: Any) -> Any:
        raise FileNotFoundError("missing")

    monkeypatch.setattr(subprocess, "Popen", fake_popen)
    with pytest.raises(SandboxError, match="Command not found in sandbox"):
        manager.run("demo", ["rules", "install"])


def test_stale_lock_is_reaped_when_owner_is_not_running(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("HOME", str(tmp_path))
    manager = SandboxManager(workspace_root=tmp_path)
    lock_file = manager._lock_path("stale")
    lock_file.write_text("999999 0\n", encoding="utf-8")
    monkeypatch.setattr(sandbox_manager, "_is_process_running", lambda _pid: False)

    with manager._lock("stale"):
        assert lock_file.exists()

    assert not lock_file.exists()


def test_live_lock_is_not_reaped(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("HOME", str(tmp_path))
    manager = SandboxManager(workspace_root=tmp_path)
    lock_file = manager._lock_path("busy")
    lock_file.write_text(f"{12345} {time.time()}\n", encoding="utf-8")
    monkeypatch.setattr(sandbox_manager, "_is_process_running", lambda _pid: True)

    with pytest.raises(SandboxError, match="currently locked"):
        with manager._lock("busy"):
            pass
