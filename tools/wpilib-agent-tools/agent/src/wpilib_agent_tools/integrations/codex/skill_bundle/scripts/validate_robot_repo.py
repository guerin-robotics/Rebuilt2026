#!/usr/bin/env python3
"""Validate WPILib simulation behavior against a target robot repository."""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


EXPECTED_SIM_EXIT_CODES = {0, 143}


def _run(cmd: list[str], *, cwd: Path | None = None, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    merged_env = os.environ.copy()
    if env:
        merged_env.update(env)
    return subprocess.run(
        cmd,
        cwd=str(cwd) if cwd else None,
        text=True,
        capture_output=True,
        check=False,
        env=merged_env,
    )


def _run_cli(cli_runner: Path, args: list[str], *, cwd: Path | None = None, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    return _run([str(cli_runner), *args], cwd=cwd, env=env)


def _parse_json(text: str, *, context: str) -> dict[str, Any]:
    raw = text.strip()
    if not raw:
        raise RuntimeError(f"{context}: expected JSON output, got empty stdout")
    try:
        parsed = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"{context}: failed to parse JSON output: {exc}\n---\n{raw}") from exc
    if not isinstance(parsed, dict):
        raise RuntimeError(f"{context}: expected JSON object, got {type(parsed).__name__}")
    return parsed


def _git_current_branch(repo: Path) -> str:
    result = _run(["git", "-C", str(repo), "branch", "--show-current"])
    if result.returncode != 0:
        raise RuntimeError(f"failed to detect git branch:\n{result.stderr.strip()}")
    return result.stdout.strip()


def _replace_method_body(source: str, signature: str, body_lines: list[str]) -> str:
    signature_index = source.find(signature)
    if signature_index < 0:
        raise RuntimeError(f"method signature not found: {signature}")

    open_brace_index = source.find("{", signature_index)
    if open_brace_index < 0:
        raise RuntimeError(f"could not find opening brace for method: {signature}")

    depth = 0
    close_brace_index = -1
    for index in range(open_brace_index, len(source)):
        char = source[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                close_brace_index = index
                break

    if close_brace_index < 0:
        raise RuntimeError(f"could not find closing brace for method: {signature}")

    line_start = source.rfind("\n", 0, signature_index) + 1
    indent_match = re.match(r"\s*", source[line_start:])
    method_indent = indent_match.group(0) if indent_match else ""

    body_text = "\n".join(f"{method_indent}    {line}" for line in body_lines)
    return (
        source[: open_brace_index + 1]
        + "\n"
        + body_text
        + "\n"
        + method_indent
        + source[close_brace_index:]
    )


def _strip_single_comment_prefix(line: str) -> str:
    stripped = line.lstrip()
    if stripped.startswith("//"):
        return stripped[2:]
    return line


def _comment_bline_include_block(settings_text: str) -> tuple[str, str]:
    lines = settings_text.splitlines(keepends=True)
    block_start = -1
    block_end = -1
    for index, line in enumerate(lines):
        normalized = _strip_single_comment_prefix(line).strip()
        if "if (file(" not in normalized:
            continue
        if "BLine-Lib" not in normalized or "exists())" not in normalized:
            continue
        block_start = index
        depth = 0
        for cursor in range(index, len(lines)):
            scan_line = _strip_single_comment_prefix(lines[cursor])
            depth += scan_line.count("{")
            depth -= scan_line.count("}")
            if depth <= 0 and cursor > index:
                block_end = cursor
                break
        break

    if block_start < 0 or block_end < block_start:
        return settings_text, "settings.gradle local BLine-Lib block not found; left unchanged"

    block_lines = lines[block_start : block_end + 1]
    if all((not line.strip()) or line.lstrip().startswith("//") for line in block_lines):
        return settings_text, "settings.gradle local BLine-Lib include block already commented"

    for cursor in range(block_start, block_end + 1):
        line = lines[cursor]
        if not line.strip():
            continue
        indent = line[: len(line) - len(line.lstrip())]
        content = line[len(indent) :]
        lines[cursor] = f"{indent}// {content}"
    return "".join(lines), "commented local BLine-Lib include block in settings.gradle"


def _patch_2026_profile(sandbox_path: Path, auto_path: str) -> list[str]:
    notes: list[str] = []

    robot_container = sandbox_path / "src/main/java/frc/robot/RobotContainer.java"
    robot_java = sandbox_path / "src/main/java/frc/robot/Robot.java"
    constants_java = sandbox_path / "src/main/java/frc/robot/constants/Constants.java"
    settings_gradle = sandbox_path / "settings.gradle"

    required_files = [robot_container, robot_java, constants_java, settings_gradle]
    missing = [str(path) for path in required_files if not path.exists()]
    if missing:
        raise RuntimeError(f"2026 profile requires files that were not found: {missing}")

    container_text = robot_container.read_text(encoding="utf-8")
    container_text = _replace_method_body(
        container_text,
        "public Command getAutonomousCommand()",
        [f"return Autos.followPath(\"{auto_path}\", true);"],
    )
    robot_container.write_text(container_text, encoding="utf-8")
    notes.append("set autonomous command to Autos.followPath(auto_path, true)")

    constants_text = constants_java.read_text(encoding="utf-8")
    constants_text, count = re.subn(
        r"(public\s+static\s+final\s+Mode\s+currentMode\s*=\s*Mode\.)[A-Z_]+(;)",
        r"\1SIM\2",
        constants_text,
        count=1,
    )
    if count == 0:
        raise RuntimeError("could not patch Constants.currentMode")
    constants_java.write_text(constants_text, encoding="utf-8")
    notes.append("forced Constants.currentMode to Mode.SIM")

    robot_text = robot_java.read_text(encoding="utf-8")
    if "import edu.wpi.first.wpilibj.simulation.DriverStationSim;" not in robot_text:
        robot_text, import_count = re.subn(
            r"(import edu\.wpi\.first\.wpilibj\.Filesystem;\n)",
            r"\1import edu.wpi.first.wpilibj.simulation.DriverStationSim;\n",
            robot_text,
            count=1,
        )
        if import_count == 0:
            raise RuntimeError("could not add DriverStationSim import to Robot.java")

    if "DriverStationSim.setAutonomous(true);" not in robot_text:
        marker = "        m_robotContainer = RobotContainer.getInstance();\n"
        insertion = (
            marker
            + "        if (Constants.currentMode == Constants.Mode.SIM) {\n"
            + "            DriverStationSim.setAutonomous(true);\n"
            + "            DriverStationSim.setEnabled(true);\n"
            + "            DriverStationSim.notifyNewData();\n"
            + "        }\n"
        )
        if marker not in robot_text:
            raise RuntimeError("could not insert DriverStationSim block into Robot.robotInit")
        robot_text = robot_text.replace(marker, insertion, 1)

    robot_java.write_text(robot_text, encoding="utf-8")
    notes.append("added DriverStationSim auto-enable block in Robot.robotInit")

    settings_text = settings_gradle.read_text(encoding="utf-8")
    settings_text, settings_note = _comment_bline_include_block(settings_text)
    notes.append(settings_note)
    settings_gradle.write_text(settings_text, encoding="utf-8")
    return notes


def _contains_subsequence(values: list[str], expected: list[str]) -> bool:
    if not expected:
        return True
    position = 0
    for value in values:
        if value == expected[position]:
            position += 1
            if position == len(expected):
                return True
    return False


def _normalize_profile_defaults(profile: str) -> dict[str, Any]:
    if profile == "2026-robot-code":
        return {
            "duration": 30.0,
            "record_delay": 3.0,
            "auto_path": "straight",
            "state_key": "/AdvantageKit/RealOutputs/SwerveDrive/currentSystemState",
            "expected_states": ["FOLLOW_PATH", "IDLE"],
            "check_ds": True,
            "apply_profile_patch": True,
        }

    return {
        "duration": 30.0,
        "record_delay": 3.0,
        "auto_path": None,
        "state_key": None,
        "expected_states": [],
        "check_ds": False,
        "apply_profile_patch": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Run repeatable wpilib-agent-tools validation against a robot repo")
    parser.add_argument("--repo", required=True, help="Path to target robot repository")
    parser.add_argument("--branch", help="Expected branch name")
    parser.add_argument("--profile", choices=["generic", "2026-robot-code"], default="generic")
    parser.add_argument("--duration", type=float, help="Sim duration seconds")
    parser.add_argument("--record-delay", type=float, help="Recorder start delay seconds")
    parser.add_argument("--auto-path", help="Autonomous path name for profile-specific patching")
    parser.add_argument("--state-key", help="State key to query for expected sequence checks")
    parser.add_argument("--expected-states", help="Comma-separated ordered states to require")
    parser.add_argument("--check-ds", dest="check_ds", action="store_true", help="Require DS enabled+autonomous checks")
    parser.add_argument("--no-check-ds", dest="check_ds", action="store_false", help="Disable DS checks")
    parser.set_defaults(check_ds=None)
    parser.add_argument("--sandbox-name", help="Optional explicit sandbox name")
    parser.add_argument("--keep-sandbox", action="store_true", help="Always keep sandbox after run")
    parser.add_argument("--keep-sandbox-on-fail", action="store_true", help="Keep sandbox when checks fail")
    args = parser.parse_args()

    repo_path = Path(args.repo).expanduser().resolve()
    script_dir = Path(__file__).resolve().parent
    cli_runner = script_dir / "run_cli.sh"

    if not repo_path.exists() or not repo_path.is_dir():
        print(json.dumps({"passed": False, "error": f"repo path not found: {repo_path}"}, indent=2))
        return 1

    if not cli_runner.exists():
        print(json.dumps({"passed": False, "error": f"CLI runner not found: {cli_runner}"}, indent=2))
        return 1

    defaults = _normalize_profile_defaults(args.profile)
    duration = args.duration if args.duration is not None else defaults["duration"]
    record_delay = args.record_delay if args.record_delay is not None else defaults["record_delay"]
    auto_path = args.auto_path if args.auto_path is not None else defaults["auto_path"]
    state_key = args.state_key if args.state_key is not None else defaults["state_key"]
    check_ds = args.check_ds if args.check_ds is not None else defaults["check_ds"]

    expected_states: list[str]
    if args.expected_states is not None:
        expected_states = [item.strip() for item in args.expected_states.split(",") if item.strip()]
    else:
        expected_states = list(defaults["expected_states"])

    report: dict[str, Any] = {
        "repo": str(repo_path),
        "profile": args.profile,
        "inputs": {
            "duration": duration,
            "record_delay": record_delay,
            "auto_path": auto_path,
            "state_key": state_key,
            "expected_states": expected_states,
            "check_ds": check_ds,
            "branch": args.branch,
        },
        "checks": {},
        "sandbox": {},
        "patch_notes": [],
    }

    branch = _git_current_branch(repo_path)
    report["branch"] = branch
    if args.branch and branch != args.branch:
        report["passed"] = False
        report["error"] = f"expected branch '{args.branch}' but repo is on '{branch}'"
        print(json.dumps(report, indent=2))
        return 1

    sandbox_name = args.sandbox_name or f"validate-{int(time.time())}"
    report["sandbox"]["name"] = sandbox_name

    create_proc = _run_cli(
        cli_runner,
        ["sandbox", "create", "--name", sandbox_name, "--source", "workspace", "--json"],
        cwd=repo_path,
    )
    if create_proc.returncode != 0:
        report["passed"] = False
        report["error"] = f"sandbox create failed: {create_proc.stderr.strip() or create_proc.stdout.strip()}"
        print(json.dumps(report, indent=2))
        return 1

    create_payload = _parse_json(create_proc.stdout, context="sandbox create")
    sandbox_path = Path(str(create_payload.get("path", "")))
    if not sandbox_path.exists():
        report["passed"] = False
        report["error"] = "sandbox path missing after create"
        print(json.dumps(report, indent=2))
        return 1

    report["sandbox"]["path"] = str(sandbox_path)

    passed = False
    sim_payload: dict[str, Any] = {}

    try:
        if defaults["apply_profile_patch"]:
            if not auto_path:
                raise RuntimeError("profile patch requested but no auto_path resolved")
            report["patch_notes"] = _patch_2026_profile(sandbox_path, auto_path)

        sim_proc = _run_cli(
            cli_runner,
            [
                "sim",
                "--duration",
                f"{duration:g}",
                "--record-delay",
                f"{record_delay:g}",
                "--json",
            ],
            cwd=sandbox_path,
            env={"WPILIB_AGENT_TOOLS_SANDBOX": "1"},
        )
        sim_payload = _parse_json(sim_proc.stdout, context="sim run")

        log_generation = sim_payload.get("log_generation", {})
        generated_path = log_generation.get("path")
        if not generated_path:
            generated_log_path: Path | None = None
        else:
            generated_log_path = Path(str(generated_path))
            if not generated_log_path.is_absolute():
                generated_log_path = sandbox_path / generated_log_path

        report["sim"] = {
            "process_returncode": sim_proc.returncode,
            "sim_exit_code": sim_payload.get("exit_code"),
            "log_generation": log_generation,
            "recording": sim_payload.get("recording"),
            "output_summary": sim_payload.get("output_summary"),
        }
        report["log_path"] = str(generated_log_path) if generated_log_path else None

        checks: dict[str, bool] = {}
        checks["sim_exit_expected"] = sim_proc.returncode in EXPECTED_SIM_EXIT_CODES
        checks["log_generated"] = bool(log_generation.get("passed")) and generated_log_path is not None

        ds_result: dict[str, Any] | None = None
        if checks["log_generated"] and generated_log_path is not None:
            if check_ds:
                ds_proc = _run_cli(
                    cli_runner,
                    ["query", "--mode", "ds", "--file", str(generated_log_path), "--json"],
                    cwd=repo_path,
                )
                if ds_proc.returncode == 0:
                    ds_payload = _parse_json(ds_proc.stdout, context="query ds")
                    results = ds_payload.get("results") or []
                    if results and isinstance(results[0], dict):
                        ds_result = results[0]
                checks["ds_enabled_true"] = bool(
                    ds_result
                    and isinstance(ds_result.get("state"), dict)
                    and ds_result["state"].get("DriverStation/Enabled") is True
                )
                checks["ds_autonomous_true"] = bool(
                    ds_result
                    and isinstance(ds_result.get("state"), dict)
                    and ds_result["state"].get("DriverStation/Autonomous") is True
                )

            if state_key and expected_states:
                values_proc = _run_cli(
                    cli_runner,
                    [
                        "query",
                        "--mode",
                        "values",
                        "--file",
                        str(generated_log_path),
                        "--key",
                        state_key,
                        "--limit",
                        "20000",
                        "--json",
                    ],
                    cwd=repo_path,
                )
                state_values: list[str] = []
                if values_proc.returncode == 0:
                    values_payload = _parse_json(values_proc.stdout, context="query values")
                    values_results = values_payload.get("results") or []
                    if values_results and isinstance(values_results[0], dict):
                        rows = values_results[0].get("values") or []
                        for row in rows:
                            if isinstance(row, list) and len(row) == 2:
                                state_values.append(str(row[1]))
                report["state_values"] = state_values
                checks["expected_state_sequence"] = _contains_subsequence(state_values, expected_states)

        report["ds_result"] = ds_result
        report["checks"] = checks
        passed = all(checks.values()) if checks else False
        report["passed"] = passed

    except Exception as exc:
        report["passed"] = False
        report["error"] = str(exc)

    keep_sandbox = args.keep_sandbox or ((not report.get("passed", False)) and args.keep_sandbox_on_fail)
    report["sandbox"]["kept"] = keep_sandbox

    if not keep_sandbox:
        clean_proc = _run_cli(
            cli_runner,
            ["sandbox", "clean", "--name", sandbox_name],
            cwd=repo_path,
        )
        if clean_proc.returncode != 0:
            report["cleanup_warning"] = clean_proc.stderr.strip() or clean_proc.stdout.strip()

    print(json.dumps(report, indent=2))
    return 0 if report.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
