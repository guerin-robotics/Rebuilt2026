from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from wpilib_agent_tools.cli import build_parser
from wpilib_agent_tools.commands import sim


class _FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += max(0.0, seconds)


class _FakeProcess:
    def __init__(
        self,
        *,
        pid: int,
        clock: _FakeClock,
        done_at: float,
        return_code: int = 0,
        stderr_text: str = "",
    ) -> None:
        self.pid = pid
        self._clock = clock
        self._done_at = done_at
        self._return_code_target = return_code
        self._stderr_text = stderr_text
        self.returncode: int | None = None

    def poll(self) -> int | None:
        if self.returncode is not None:
            return self.returncode
        if self._clock.now >= self._done_at:
            self.returncode = self._return_code_target
            return self.returncode
        return None

    def wait(self, timeout: float | None = None) -> int:
        _ = timeout
        if self.returncode is None:
            self.returncode = self._return_code_target
        return self.returncode

    def communicate(self, timeout: float | None = None) -> tuple[str, str]:
        _ = timeout
        if self.returncode is None:
            self.returncode = self._return_code_target
        return ("", self._stderr_text)


def _sim_args(*extra: str) -> Any:
    parser = build_parser()
    return parser.parse_args(
        [
            "sim",
            "--direct-workspace",
            "--duration",
            "4",
            "--record-delay",
            "0.2",
            "--no-analyze",
            "--json",
            *extra,
        ]
    )


def _install_runtime_fakes(
    monkeypatch: Any,
    tmp_path: Path,
    *,
    generated_log: Path | None,
    record_exit_code: int,
    sim_return_code: int = 0,
    sim_done_at: float = 2.6,
    record_done_at: float = 100.0,
) -> tuple[list[list[str]], list[tuple[int, Any]]]:
    logs_dir = tmp_path / "agent" / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(sim, "_logs_dir", lambda: logs_dir)
    monkeypatch.setattr(sim, "_snapshot_log_state", lambda _path: {})
    monkeypatch.setattr(sim, "_resolve_latest_generated_log", lambda _path, _state: generated_log)
    monkeypatch.setattr(sim, "_kill_prior_instance", lambda _pid_file: {"killed": False})
    monkeypatch.setattr(sim, "_sim_pid_file", lambda _scope: tmp_path / "sim.pid")
    monkeypatch.setattr(sim, "_sim_scope_id", lambda: "test-scope")
    monkeypatch.setattr(sim.os, "getpgid", lambda pid: pid)
    kill_calls: list[tuple[int, Any]] = []
    monkeypatch.setattr(sim.os, "killpg", lambda pgid, sig: kill_calls.append((pgid, sig)))

    clock = _FakeClock()
    monkeypatch.setattr(sim.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(sim.time, "sleep", clock.sleep)

    commands: list[list[str]] = []
    pid_counter = {"next": 1000}

    def fake_popen(command: list[str], **kwargs: Any) -> _FakeProcess:
        _ = kwargs
        commands.append(command)
        pid_counter["next"] += 1
        if command and command[0] == "./gradlew":
            return _FakeProcess(
                pid=pid_counter["next"],
                clock=clock,
                done_at=sim_done_at,
                return_code=sim_return_code,
            )
        return _FakeProcess(
            pid=pid_counter["next"],
            clock=clock,
            done_at=record_done_at,
            return_code=record_exit_code,
            stderr_text="record-failed" if record_exit_code != 0 else "",
        )

    monkeypatch.setattr(sim.subprocess, "Popen", fake_popen)
    return commands, kill_calls


def test_sim_auto_record_enabled_by_default_and_invoked(
    monkeypatch: Any, tmp_path: Path, capsys: Any
) -> None:
    generated_log = tmp_path / "agent" / "logs" / "generated.wpilog"
    commands, _kill_calls = _install_runtime_fakes(
        monkeypatch,
        tmp_path,
        generated_log=generated_log,
        record_exit_code=0,
    )
    args = _sim_args()

    exit_code = sim.handle_sim(args)

    assert exit_code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["recording"]["enabled"] is True
    assert payload["recording"]["started"] is True
    assert payload["recording"]["exit_code"] == 0
    assert payload["log_generation"]["passed"] is True
    assert "log_summary" not in payload
    assert commands[0] == ["./gradlew", "simulateJava"]
    assert any(cmd[:4] == [sim.sys.executable, "-m", "wpilib_agent_tools", "record"] for cmd in commands)


def test_sim_fallback_to_wpilog_when_record_fails(
    monkeypatch: Any, tmp_path: Path, capsys: Any
) -> None:
    generated_log = tmp_path / "agent" / "logs" / "generated.wpilog"
    _commands, _kill_calls = _install_runtime_fakes(
        monkeypatch,
        tmp_path,
        generated_log=generated_log,
        record_exit_code=1,
    )
    args = _sim_args()

    exit_code = sim.handle_sim(args)

    assert exit_code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["recording"]["exit_code"] == 1
    assert payload["log_generation"]["passed"] is True
    assert payload["log_generation"]["path"].endswith("generated.wpilog")


def test_sim_fails_when_no_generated_log(
    monkeypatch: Any, tmp_path: Path, capsys: Any
) -> None:
    _commands, _kill_calls = _install_runtime_fakes(
        monkeypatch,
        tmp_path,
        generated_log=None,
        record_exit_code=1,
    )
    args = _sim_args()

    exit_code = sim.handle_sim(args)

    assert exit_code == 3
    payload = json.loads(capsys.readouterr().out)
    assert payload["log_generation"]["passed"] is False
    assert payload["log_generation"]["reason"] == "no_log_file_found"


def test_sim_waits_for_recorder_flush_before_terminate(
    monkeypatch: Any, tmp_path: Path, capsys: Any
) -> None:
    generated_log = tmp_path / "agent" / "logs" / "generated.wpilog"
    _commands, kill_calls = _install_runtime_fakes(
        monkeypatch,
        tmp_path,
        generated_log=generated_log,
        record_exit_code=0,
        record_done_at=2.8,
    )
    args = _sim_args()

    exit_code = sim.handle_sim(args)

    assert exit_code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["recording"]["started"] is True
    # Regression check: recorder should be allowed to exit naturally.
    assert kill_calls == []


def test_sim_normalizes_bounded_termination_exit_code(
    monkeypatch: Any, tmp_path: Path, capsys: Any
) -> None:
    generated_log = tmp_path / "agent" / "logs" / "generated.wpilog"
    _commands, _kill_calls = _install_runtime_fakes(
        monkeypatch,
        tmp_path,
        generated_log=generated_log,
        record_exit_code=0,
        sim_return_code=143,
        sim_done_at=100.0,
    )
    args = _sim_args()

    exit_code = sim.handle_sim(args)

    assert exit_code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["duration_reached"] is True
    assert payload["exit_code"] == 0
    assert payload["exit_code_raw"] == 143
