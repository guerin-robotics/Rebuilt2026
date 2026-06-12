"""Simulation command."""

from __future__ import annotations

import argparse
import hashlib
import os
import re
import signal
import subprocess
import sys
import time
from collections import deque
from io import TextIOWrapper
from pathlib import Path
from typing import Any

from wpilib_agent_tools.lib.assertions import evaluate_assertions
from wpilib_agent_tools.lib.log_reader import LogReader
from wpilib_agent_tools.lib.output import emit


def _runtime_dir() -> Path:
    path = Path.home() / ".wpilib-agent-tools" / "runtime"
    path.mkdir(parents=True, exist_ok=True)
    return path


def _sim_scope_id() -> str:
    sandbox_name = os.environ.get("WPILIB_AGENT_TOOLS_SANDBOX_NAME")
    if sandbox_name:
        return f"sandbox-{sandbox_name}"
    cwd_digest = hashlib.sha1(str(Path.cwd().resolve()).encode("utf-8")).hexdigest()[:12]
    return f"cwd-{cwd_digest}"


def _sim_pid_file(scope_id: str) -> Path:
    safe_scope = re.sub(r"[^a-zA-Z0-9._-]", "_", scope_id)
    return _runtime_dir() / f"sim-{safe_scope}.pid"


def _logs_dir() -> Path:
    path = Path("agent/logs")
    path.mkdir(parents=True, exist_ok=True)
    return path


def _snapshot_log_state(log_dir: Path) -> dict[str, int]:
    state: dict[str, int] = {}
    for path in LogReader.list_log_files(log_dir):
        try:
            state[str(path.resolve())] = path.stat().st_mtime_ns
        except OSError:
            continue
    return state


def _resolve_latest_generated_log(log_dir: Path, previous_state: dict[str, int]) -> Path | None:
    for path in LogReader.list_log_files(log_dir):
        try:
            current_mtime = path.stat().st_mtime_ns
        except OSError:
            continue
        key = str(path.resolve())
        previous_mtime = previous_state.get(key)
        if previous_mtime is None or current_mtime > previous_mtime:
            return path
    return None


def _resolve_record_output_path(log_dir: Path, output_arg: str | None) -> Path:
    if output_arg:
        path = Path(output_arg)
        if not path.is_absolute():
            path = log_dir / path
        path.parent.mkdir(parents=True, exist_ok=True)
        return path.resolve()
    path = log_dir / f"sim-nt4-{int(time.time() * 1000)}.wpilog"
    path.parent.mkdir(parents=True, exist_ok=True)
    return path.resolve()


def _record_grace_timeout(
    *,
    started_at: float | None,
    duration_sec: float,
    interrupted: bool,
) -> float:
    """How long to wait for recorder to flush before forcing termination."""
    if interrupted:
        return 0.5
    if started_at is None:
        return 2.0
    expected_end = started_at + max(0.0, duration_sec)
    # Include connection setup overhead (up to 5s) plus buffer for final flush.
    remaining = max(0.0, expected_end - time.monotonic()) + 6.0
    return max(2.0, min(10.0, remaining))


def _is_running(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def _kill_prior_instance(pid_file: Path) -> dict[str, Any]:
    """Kill previous sim instance to enforce single active run."""
    if not pid_file.exists():
        return {"killed": False}
    try:
        pid = int(pid_file.read_text(encoding="utf-8").strip())
    except Exception:
        pid_file.unlink(missing_ok=True)
        return {"killed": False}

    if not _is_running(pid):
        pid_file.unlink(missing_ok=True)
        return {"killed": False}

    try:
        pgid = os.getpgid(pid)
        os.killpg(pgid, signal.SIGTERM)
        deadline = time.time() + 5.0
        while time.time() < deadline and _is_running(pid):
            time.sleep(0.1)
        if _is_running(pid):
            os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    pid_file.unlink(missing_ok=True)
    return {"killed": True, "pid": pid}


def register_subparser(subparsers: argparse._SubParsersAction) -> None:
    parser = subparsers.add_parser(
        "sim",
        help="Run headless simulation for a fixed duration.",
    )
    parser.add_argument("--duration", type=float, default=15.0, help="Simulation duration in seconds.")
    parser.add_argument("--gradle-task", default="simulateJava", help="Gradle task to invoke.")
    parser.add_argument("--no-analyze", action="store_true", help="Skip log summary after simulation.")
    parser.add_argument("--verbose", action="store_true", help="Stream Gradle output directly.")
    parser.add_argument(
        "--max-lines",
        type=int,
        default=120,
        help="Maximum output lines returned when not verbose (<=0 means unlimited).",
    )
    parser.add_argument(
        "--tail",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="When truncating output, keep trailing lines (default: true).",
    )
    parser.add_argument("--include", help="Optional regex filter for returned output lines.")
    parser.add_argument("--assert-key", action="append", help="Require at least one sample for this key.")
    parser.add_argument(
        "--assert-range",
        action="append",
        nargs=3,
        metavar=("KEY", "MIN", "MAX"),
        help="Require all numeric samples for KEY to stay within [MIN, MAX].",
    )
    parser.add_argument(
        "--direct-workspace",
        action="store_true",
        help="Allow running outside sandbox (disabled by default).",
    )
    parser.add_argument(
        "--record",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Automatically record NT4 data during sim (default: true).",
    )
    parser.add_argument(
        "--record-address",
        default="localhost",
        help="NT4 server address to use when auto-recording.",
    )
    parser.add_argument(
        "--record-delay",
        type=float,
        default=2.0,
        help="Delay before starting auto-recording (seconds).",
    )
    parser.add_argument(
        "--record-output",
        help="Output file path for auto-recorded NT4 WPILOG (default: agent/logs/sim-nt4-<timestamp>.wpilog).",
    )
    parser.add_argument("--json", action="store_true", help="Emit JSON output.")
    parser.add_argument("--json-compact", action="store_true", help="Emit compact JSON output.")
    parser.set_defaults(handler=handle_sim)


def _emit(payload: dict[str, Any], as_json: bool, *, compact_json: bool = False) -> None:
    if as_json:
        emit(payload, as_json=True, compact_json=compact_json)
        return
    if payload.get("killed_previous"):
        print(f"Stopped previous sim instance (pid={payload.get('previous_pid')}).")
    print(
        f"Completed sim pid={payload['pid']} "
        f"(duration={payload['duration_sec']}s, exit_code={payload['exit_code']})"
    )
    if payload.get("duration_reached"):
        print("Duration reached: terminated running sim process at end of bounded run.")
    output_summary = payload.get("output_summary")
    if isinstance(output_summary, dict):
        print(
            "Output lines: "
            f"included={output_summary.get('included_lines', 0)} "
            f"returned={output_summary.get('returned_lines', 0)} "
            f"warnings={output_summary.get('warning_lines', 0)} "
            f"errors={output_summary.get('error_lines', 0)}"
        )
        excerpt = payload.get("output_excerpt") or []
        if excerpt:
            print("Output excerpt:")
            for line in excerpt:
                print(f"  {line}")
        if output_summary.get("artifact"):
            print(f"Full output saved to: {output_summary['artifact']}")
    if payload.get("verbose"):
        print("Verbose mode enabled: full Gradle output was streamed directly.")
    if payload.get("exit_code_raw") != payload.get("exit_code"):
        print(f"Exit code: {payload['exit_code']} (raw={payload.get('exit_code_raw')})")
    else:
        print(f"Exit code: {payload['exit_code']}")
    summary = payload.get("log_summary")
    if summary:
        print(
            "Latest log: "
            f"{summary['path']} (keys={summary['key_count']}, duration={summary['duration_sec']})"
        )
    log_generation = payload.get("log_generation")
    if isinstance(log_generation, dict):
        if log_generation.get("passed"):
            print(f"Generated log: {log_generation.get('path')}")
        else:
            print(f"Generated log: none ({log_generation.get('reason')})")
    recording = payload.get("recording")
    if isinstance(recording, dict) and recording.get("enabled"):
        print(
            "Auto-record: "
            f"started={recording.get('started')} "
            f"exit_code={recording.get('exit_code')} "
            f"output={recording.get('output_path')}"
        )
    assertions = payload.get("assertions")
    if isinstance(assertions, dict):
        result = "PASS" if assertions.get("passed") else "FAIL"
        stats = assertions.get("summary", {})
        print(
            f"Assertions: {result} "
            f"(passed={stats.get('passed', 0)}, failed={stats.get('failed', 0)})"
        )
        for check in assertions.get("checks", []):
            if not check.get("passed"):
                reason = check.get("reason")
                key = check.get("key")
                check_type = check.get("type")
                print(f"  failed {check_type} key={key} reason={reason}")


def _parse_assert_ranges(raw_ranges: list[list[str]] | None) -> list[tuple[str, float, float]]:
    parsed: list[tuple[str, float, float]] = []
    for item in raw_ranges or []:
        key, min_text, max_text = item
        try:
            min_value = float(min_text)
            max_value = float(max_text)
        except ValueError as exc:
            raise ValueError(
                f"Invalid --assert-range bounds for '{key}': min='{min_text}' max='{max_text}'"
            ) from exc
        if min_value > max_value:
            raise ValueError(f"Invalid --assert-range for '{key}': min must be <= max")
        parsed.append((key, min_value, max_value))
    return parsed


def _summarize_output_file(
    artifact_path: Path,
    *,
    max_lines: int | None,
    tail: bool,
    include: str | None,
) -> tuple[list[str], dict[str, Any]]:
    include_re: re.Pattern[str] | None = None
    if include:
        try:
            include_re = re.compile(include)
        except re.error as exc:
            raise ValueError(f"Invalid --include regex: {exc}") from exc

    selected_head: list[str] = []
    selected_tail: deque[str] = deque(maxlen=max_lines if max_lines and max_lines > 0 else None)
    total_lines = 0
    included_lines = 0
    warning_lines = 0
    error_lines = 0

    with artifact_path.open("r", encoding="utf-8", errors="replace") as artifact:
        for raw_line in artifact:
            line = raw_line.rstrip("\n")
            lowered = line.lower()
            total_lines += 1
            if "warning" in lowered:
                warning_lines += 1
            if "error" in lowered:
                error_lines += 1
            if include_re and not include_re.search(line):
                continue
            included_lines += 1
            if max_lines is None or max_lines <= 0:
                selected_head.append(line)
                continue
            if tail:
                selected_tail.append(line)
            elif len(selected_head) < max_lines:
                selected_head.append(line)
    if max_lines is None or max_lines <= 0:
        excerpt = selected_head
    elif tail:
        excerpt = list(selected_tail)
    else:
        excerpt = selected_head

    return excerpt, {
        "total_lines": total_lines,
        "included_lines": included_lines,
        "returned_lines": len(excerpt),
        "truncated": included_lines > len(excerpt),
        "tail": tail,
        "max_lines": max_lines,
        "include": include,
        "warning_lines": warning_lines,
        "error_lines": error_lines,
        "artifact": str(artifact_path),
    }


def handle_sim(args: argparse.Namespace) -> int:
    inside_sandbox = os.environ.get("WPILIB_AGENT_TOOLS_SANDBOX") == "1"
    if not inside_sandbox and not args.direct_workspace:
        print(
            "Refusing to run sim directly in workspace. "
            "Use `wpilib-agent-tools sandbox run --name <sandbox> -- sim ...` "
            "or pass --direct-workspace."
        )
        return 2

    try:
        assertion_ranges = _parse_assert_ranges(args.assert_range)
    except ValueError as exc:
        print(str(exc))
        return 2
    if args.include:
        try:
            re.compile(args.include)
        except re.error as exc:
            print(f"Invalid --include regex: {exc}")
            return 2
    if args.record_delay < 0:
        print("--record-delay must be >= 0")
        return 2

    sim_scope = _sim_scope_id()
    sim_pid_file = _sim_pid_file(sim_scope)
    logs_dir = _logs_dir()
    pre_log_state = _snapshot_log_state(logs_dir)
    kill_result = _kill_prior_instance(sim_pid_file)
    command = ["./gradlew", args.gradle_task]
    output_artifact_path: Path | None = None
    output_file_handle: TextIOWrapper | None = None
    record_process: subprocess.Popen[str] | None = None
    record_output_path: Path | None = None
    record_started = False
    record_attempted = False
    record_started_at: float | None = None
    record_duration = 0.0
    record_delay = 0.0
    record_stderr = ""
    popen_kwargs: dict[str, Any] = {"preexec_fn": os.setsid}
    if not args.verbose:
        reports_dir = Path("agent/reports")
        reports_dir.mkdir(parents=True, exist_ok=True)
        output_artifact_path = reports_dir / f"sim-run-{int(time.time() * 1000)}.log"
        output_file_handle = output_artifact_path.open("w", encoding="utf-8")
        popen_kwargs.update({"stdout": output_file_handle, "stderr": subprocess.STDOUT})
    try:
        process = subprocess.Popen(command, **popen_kwargs)
    except OSError as exc:
        if output_file_handle is not None:
            output_file_handle.close()
        print(f"Failed to launch gradle sim command '{command[0]}': {exc}")
        return 1
    sim_pid_file.write_text(str(process.pid), encoding="utf-8")

    start = time.monotonic()
    interrupted = False
    duration_reached = False
    output_excerpt: list[str] = []
    output_summary: dict[str, Any] | None = None
    if args.record:
        record_delay = max(0.0, min(args.record_delay, max(0.0, args.duration - 0.5)))
        record_duration = max(0.5, args.duration - record_delay)
        record_output_path = _resolve_record_output_path(logs_dir, args.record_output)
    try:
        while True:
            elapsed = time.monotonic() - start
            process_status = process.poll()
            if args.record and not record_attempted and process_status is None and elapsed >= record_delay:
                record_attempted = True
                record_command = [
                    sys.executable,
                    "-m",
                    "wpilib_agent_tools",
                    "record",
                    "--address",
                    args.record_address,
                    "--duration",
                    str(record_duration),
                    "--output",
                    str(record_output_path),
                    "--json",
                ]
                try:
                    record_process = subprocess.Popen(
                        record_command,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True,
                        preexec_fn=os.setsid,
                    )
                    record_started = True
                    record_started_at = time.monotonic()
                except OSError as exc:
                    record_stderr = f"failed_to_start_recorder: {exc}"
            if process_status is not None:
                break
            if elapsed >= args.duration:
                duration_reached = True
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        if process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=5.0)
        if record_process is not None:
            if record_process.poll() is None:
                wait_timeout = _record_grace_timeout(
                    started_at=record_started_at,
                    duration_sec=record_duration,
                    interrupted=interrupted,
                )
                try:
                    record_process.wait(timeout=wait_timeout)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(os.getpgid(record_process.pid), signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    try:
                        record_process.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        try:
                            os.killpg(os.getpgid(record_process.pid), signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                        record_process.wait(timeout=5.0)
            try:
                _, record_stderr = record_process.communicate(timeout=0.1)
            except subprocess.TimeoutExpired:
                record_stderr = ""
        sim_pid_file.unlink(missing_ok=True)
        if output_file_handle is not None:
            output_file_handle.close()

    if not args.verbose:
        try:
            if output_artifact_path is None:
                raise ValueError("Internal error: missing output artifact path")
            output_excerpt, output_summary = _summarize_output_file(
                output_artifact_path,
                max_lines=args.max_lines,
                tail=args.tail,
                include=args.include,
            )
        except ValueError as exc:
            print(str(exc))
            return 2

    exit_code_raw = process.returncode if process.returncode is not None else 0
    exit_code = exit_code_raw
    if duration_reached and not interrupted and exit_code_raw != 0:
        # Bounded runs intentionally terminate long-running gradle sim tasks.
        exit_code = 0
    assertion_keys = args.assert_key or []

    payload: dict[str, Any] = {
        "pid": process.pid,
        "scope": sim_scope,
        "duration_sec": args.duration,
        "duration_reached": duration_reached,
        "interrupted": interrupted,
        "exit_code": exit_code,
        "exit_code_raw": exit_code_raw,
        "killed_previous": bool(kill_result.get("killed")),
        "previous_pid": kill_result.get("pid"),
        "verbose": args.verbose,
        "output_excerpt": output_excerpt,
        "output_summary": output_summary,
        "recording": {
            "enabled": args.record,
            "started": record_started,
            "address": args.record_address if args.record else None,
            "delay_sec": record_delay if args.record else None,
            "duration_sec": record_duration if args.record else None,
            "output_path": str(record_output_path) if record_output_path is not None else None,
            "exit_code": record_process.returncode if record_process is not None else None,
            "stderr": record_stderr.strip() if record_stderr else None,
        },
    }

    latest = _resolve_latest_generated_log(logs_dir, pre_log_state)
    if latest is None and record_output_path is not None and record_output_path.exists():
        latest = record_output_path
    payload["log_generation"] = {
        "passed": latest is not None,
        "reason": None if latest is not None else "no_log_file_found",
        "path": str(latest) if latest is not None else None,
        "log_dir": str(logs_dir),
    }
    if not args.no_analyze:
        if latest is not None:
            summary = LogReader(latest).get_summary()
            payload["log_summary"] = {
                "path": summary.path,
                "format": summary.format,
                "duration_sec": summary.duration_sec,
                "key_count": summary.key_count,
                "sample_count": summary.sample_count,
            }

    if assertion_keys or assertion_ranges:
        if latest is None:
            payload["assertions"] = {
                "passed": False,
                "summary": {"total": 0, "passed": 0, "failed": 1},
                "checks": [
                    {
                        "type": "assertion_runtime",
                        "key": None,
                        "passed": False,
                        "reason": "no_log_file_found",
                    }
                ],
            }
        else:
            payload["assertions"] = evaluate_assertions(
                latest,
                assert_keys=assertion_keys,
                assert_ranges=assertion_ranges,
            )

    _emit(payload, args.json, compact_json=args.json_compact)
    if exit_code != 0:
        return exit_code
    log_generation = payload.get("log_generation")
    if isinstance(log_generation, dict) and not log_generation.get("passed", True):
        return 3
    assertions = payload.get("assertions")
    if isinstance(assertions, dict) and not assertions.get("passed", True):
        return 3
    return 0
