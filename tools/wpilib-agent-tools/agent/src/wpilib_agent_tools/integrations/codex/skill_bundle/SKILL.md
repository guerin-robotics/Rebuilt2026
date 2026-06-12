---
name: wpilib-agent-tools
description: Sandbox-first WPILib simulation, NT4 recording, and WPILOG analysis orchestration with the wpilib-agent-tools CLI. Use when tasks involve running simulation, validating autonomous or subsystem behavior, querying/graphing log keys, reconstructing DriverStation state, generating evidence from .wpilog files, or iterating robot-code changes safely in isolated sandboxes.
---

# WPILib Agent Tools

Use this skill to run and validate WPILib robot workflows through `wpilib-agent-tools` instead of ad-hoc shell/python glue.

## Quick Start

- Verify CLI availability:
  - `scripts/run_cli.sh --version`
- For quick local validation in a target repo:
  - `scripts/validate_robot_repo.sh --repo /path/to/robot-repo --profile generic`
- For the author-specific 2026 profile example:
  - `scripts/validate_robot_repo.sh --repo ~/FRC/2026-Robot-Code --branch comp-dev --profile 2026-robot-code`

## Operation Modes

- Use direct CLI commands (`logs`, `keys`, `query`, `graph`, `record`, `math`, `sim`) when no code edits are needed and only analysis/evidence is required.
- Use sandbox workflow when changing code and validating behavior:
  1. `sandbox create`
  2. run `sim` and analysis commands
  3. `sandbox patch`
  4. clean sandbox
- Use `scripts/validate_robot_repo.sh` for repeatable end-to-end checks and machine-readable pass/fail output.

## CLI Fluency Rule

Do not assume flags or modes from memory. For unfamiliar contexts, self-discover first:

1. `scripts/run_cli.sh --help`
2. `scripts/run_cli.sh <subcommand> --help`
3. Prefer `--json` for parseable outputs.
4. Use bounded output (`--summary`, `--limit`, `--max-lines`) before verbose dumps.

## Pre-Sim Gate

Before `sim`, confirm:

1. The target repo is in simulation mode (for example via `Constants.java`).
2. DriverStation is auto-enabled for the target mode, or the run will not execute meaningful autonomous logic.

## Validation Expectations

Default evidence set for behavior validation (path or non-path):

1. A new log is generated.
2. DriverStation state shows enabled/autonomous when that mode is expected.
3. State telemetry or assertions show the expected behavior sequence/range.

## Failure Semantics

- Do not treat bounded-run termination as an automatic failure if telemetry checks pass (`exit_code_raw` may be `143` while normalized `exit_code` is `0`).
- Treat telemetry and explicit assertions as source of truth, not process exit alone.
- Recorder issues are triaged by evidence:
  - If a new analyzable log exists and checks pass, run can still be acceptable.
  - If DS/state evidence is missing, tune timing and retry.

## Known Timing Tip

Startup overhead can consume early behavior windows. A reliable starting point for many repos is:

- `--duration 30`
- `--record-delay 3`

Practical timing expectations:

- allow roughly 5 seconds for sim and NT startup
- allow at least 10 seconds for behavior execution (many auto paths run ~15-20 seconds)

If early transitions are missing:

1. Lower `--record-delay`.
2. Increase `--duration`.
3. Add a startup `WaitCommand` (or equivalent gate) before the behavior under test so recording and sim initialization are stable before the critical action starts.

## References

- Workflow patterns: `references/workflows.md`
- Validation flow: `references/validation.md`
- Timing and recorder tuning: `references/tuning-tips.md`
- Full CLI capability map: `references/cli-capabilities.md`
- Author-specific repo profile example: `references/profiles/2026-robot-code.md`
