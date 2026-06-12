# WPILib Agent Tools

`wpilib-agent-tools` is a sandbox-first CLI for WPILib robot iteration, simulation runs, NetworkTables recording, and post-run log analysis.

This file is the **CLI/package manual**.

For the project overview, supported harnesses, shared installer story, and release/share guidance, start with the root [`README.md`](../README.md).

This repository also ships workspace support for Codex, Claude Code, and Cursor, plus a package-centered Codex skill bundle for agent-facing workflow guidance and reusable validation helpers.

The design goal is safe iteration:

- do work in disposable sandboxes
- keep the workspace clean until review
- produce explicit patches before applying final changes

## Table of Contents

- [What This Tool Does](#what-this-tool-does)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Command Overview](#command-overview)
- [Detailed Command Reference](#detailed-command-reference)
  - [logs](#logs)
  - [keys](#keys)
  - [query](#query)
  - [graph](#graph)
  - [record](#record)
  - [view](#view)
  - [math](#math)
  - [sim](#sim)
  - [sandbox](#sandbox)
  - [harness](#harness)
  - [rules](#rules)
- [Data Formats and Struct Decoding](#data-formats-and-struct-decoding)
- [Filesystem Locations](#filesystem-locations)
- [Automation Scripts](#automation-scripts)
- [Typical End-to-End Workflow](#typical-end-to-end-workflow)
- [Troubleshooting](#troubleshooting)
- [Development](#development)

## What This Tool Does

`wpilib-agent-tools` provides:

- **Sandbox lifecycle tooling**: create, run, inspect, stop, clean, and patch isolated work areas.
- **Simulation orchestration**: run `./gradlew` tasks for fixed durations with single-instance enforcement.
- **Log analysis**: inspect keys, compute metrics, derive series, reconstruct DriverStation state, and more.
- **Graph generation**: output PNG plots for values, derivatives, and integrals.
- **NetworkTables recording**: capture live NT4 streams into WPILOG files.
- **Shared harness installation**: install workspace support for Codex, Claude Code, and Cursor from one command.
- **Cursor rule installation (compatibility path)**: install packaged `.mdc` templates directly when you only need Cursor rules.

## Installation

### Requirements

- Python `>=3.10`
- Recommended environment: virtualenv or similar
- Optional runtime dependencies:
  - `robotpy-wpiutil` for `.wpilog` read/write support
  - `pyntcore` for `record`
  - `matplotlib` for `graph`
  - `sympy` for `math`

These dependencies are included by default when you install the package.

### Install from this repository

```bash
cd agent
python -m pip install -e .
```

### Install from PyPI (when published)

```bash
python -m pip install wpilib-agent-tools
```

### Verify install

```bash
wpilib-agent-tools --version
```

You can also invoke via module:

```bash
python -m wpilib_agent_tools --version
```

## Quick Start

Create a sandbox from current workspace state, run simulation and analysis there, export a patch, then clean up:

```bash
wpilib-agent-tools sandbox create --name tune_shooter --source workspace
wpilib-agent-tools sandbox run --name tune_shooter -- sim --duration 15
wpilib-agent-tools sandbox run --name tune_shooter -- query --mode avg --key "Shooter/Velocity"
wpilib-agent-tools sandbox patch --name tune_shooter --output tune_shooter.diff
wpilib-agent-tools sandbox clean --name tune_shooter
```

## Command Overview

| Command | Purpose |
| --- | --- |
| `logs` | List log files and metadata |
| `keys` | List keys in a selected log |
| `query` | Run point/metric analysis over log data |
| `graph` | Generate PNG graphs from key series |
| `record` | Record live NT4 data to WPILOG |
| `view` | Open a log in AdvantageScope/system opener |
| `math` | Symbolic/numeric math operations |
| `sim` | Run a bounded simulation task |
| `sandbox` | Full sandbox lifecycle management |
| `harness` | Install shared workspace support for Codex, Claude Code, and Cursor |
| `rules` | Install Cursor rule templates directly (compatibility path) |

Most commands support `--json` for machine-readable output (and many support `--json-compact` for lower-overhead output). `view` is intentionally human-oriented and does not expose JSON mode.

## Detailed Command Reference

### logs

List log files (newest-first) and metadata.

```bash
wpilib-agent-tools logs --dir agent/logs
wpilib-agent-tools logs --dir agent/logs --json
wpilib-agent-tools logs --summary
wpilib-agent-tools logs --max-lines 20 --tail
```

- Supports `.wpilog` files
- Shows modified time, size, key count, and duration

### keys

List keys from a selected log.

```bash
wpilib-agent-tools keys --file agent/logs/run.wpilog
wpilib-agent-tools keys --filter shooter
wpilib-agent-tools keys --json
wpilib-agent-tools keys --summary
wpilib-agent-tools keys --max-lines 50 --tail
```

- If `--file` is omitted, uses the latest log in `agent/logs`
- `--filter` is case-insensitive substring matching

### query

Query and analyze key data in logs.

```bash
wpilib-agent-tools query --mode values --key "Shooter/Velocity" --limit 20
wpilib-agent-tools query --mode avg --key "Shooter/Velocity"
wpilib-agent-tools query --mode stats --key "Shooter/Velocity" --json
wpilib-agent-tools query --mode values --key "Shooter/Velocity" --summary
wpilib-agent-tools query --mode threshold --key "PDH/Current" --above 40 --max-lines 25
```

Common options:

- `--file`: path or glob (for example `agent/logs/*.wpilog`)
- `--start`, `--end`: time bounds (seconds)
- `--limit`: cap returned samples for series-returning modes
- `--summary`: omit heavy series payloads and return counts/aggregates
- `--max-lines`, `--tail`: cap row-heavy output in text/JSON
- `--json`: structured results

#### query modes

| Mode | What it returns | Key options |
| --- | --- | --- |
| `timestamps` | start/end/count of selected key series | `--key` |
| `values` | raw timestamp-value points | `--key`, `--limit` |
| `avg` | scalar average | `--key` |
| `minmax` | min/max values and timestamps | `--key` |
| `deriv` | derivative time series | `--key`, `--limit` |
| `integral` | integral time series | `--key`, `--limit` |
| `stats` | count, mean, stddev, percentiles | `--key` |
| `smooth` | moving-average series | `--key`, `--window`, `--limit` |
| `threshold` | threshold events and durations | `--key`, exactly one of `--above` or `--below`, `--min-duration` |
| `rms` | RMS scalar | `--key` |
| `expr` | expression-evaluated series | `--expr`, `--limit` |
| `fft` | dominant frequency components | `--key`, `--top` |
| `settle` | rise/settle/overshoot/steady-state metrics | `--key`, `--setpoint` or `--setpoint-key`, `--tolerance` |
| `ds` | reconstructed DriverStation state | optional `--start`, `--end` |

`expr` mode uses `{Key/Path}` placeholders:

```bash
wpilib-agent-tools query --mode expr \
  --expr "{Shooter/Setpoint} - {Shooter/Velocity}" \
  --json
```

`ds` mode resolves common key naming variants, including prefixed forms such as `/AdvantageKit/DriverStation/...`.

### graph

Create PNG plots from numeric series.

```bash
wpilib-agent-tools graph --key "Shooter/Velocity"
wpilib-agent-tools graph --key "Drive/Vx" --key "Drive/Vy" --mode deriv --scatter
wpilib-agent-tools graph --key "Arm/Position" --title "Arm Position" --output arm.png --json
```

- `--mode`: `values` (default), `deriv`, `integral`
- Output is always written under `agent/visualizations/`
- Non-numeric samples are skipped and reported (`skipped_non_numeric_by_key`)

### record

Record live NT4 topics into WPILOG.

```bash
wpilib-agent-tools record --address localhost --duration 10
wpilib-agent-tools record --address 10.0.0.2 --duration 15 --keys /Shooter --keys /Drive --json
wpilib-agent-tools record --address 10.0.0.2:5810 --duration 10
```

- `--keys` accepts repeatable key-prefix filters
- `--address` accepts `host`, `host:port`, or bracketed IPv6 `"[addr]:port"`
- Struct/proto topic type strings are preserved in WPILOG output for proper AdvantageScope struct decoding
- If `--output` is relative, it is written under `agent/logs/`
- Fails with a clear error if NT4 server is unreachable within a short timeout

### view

Open the selected log with platform opener behavior.

```bash
wpilib-agent-tools view --file agent/logs/latest.wpilog
wpilib-agent-tools view
```

On macOS, it prefers `AdvantageScope.app` if available, then falls back to `open`.

### math

Symbolic and numeric math helper powered by SymPy.

```bash
wpilib-agent-tools math --mode deriv --expr "x**3 + x" --var x
wpilib-agent-tools math --mode integral --expr "sin(x)" --var x
wpilib-agent-tools math --mode simplify --expr "sin(x)**2 + cos(x)**2"
wpilib-agent-tools math --mode solve --equation "x**2 - 4 = 0" --var x
wpilib-agent-tools math --mode eval --expr "x**2 + y" --value x=3 --value y=2
```

Modes:

- `deriv`, `integral`, `simplify` require `--expr`
- `solve` requires `--equation`
- `eval` requires `--expr`, accepts repeatable `--value name=value`

### sim

Run a simulation Gradle task for a bounded duration.

```bash
wpilib-agent-tools sim --duration 15 --gradle-task simulateJava --direct-workspace
wpilib-agent-tools sim --duration 20 --no-analyze --json --direct-workspace
wpilib-agent-tools sim --duration 15 --record-address localhost --record-delay 2.0 --direct-workspace
wpilib-agent-tools sim --duration 15 --no-record --direct-workspace
wpilib-agent-tools sim --duration 12 --assert-key Shooter/turretUsedUnwindFallback --direct-workspace
wpilib-agent-tools sim --duration 12 --assert-range Shooter/turretResolvedSetpointDeg -450 630 --direct-workspace
wpilib-agent-tools sim --duration 12 --max-lines 80 --tail --include "WARN|ERROR|turret" --direct-workspace
wpilib-agent-tools sim --duration 12 --verbose --direct-workspace
```

Important behavior:

- By default, direct workspace execution is blocked for safety.
- Normal path is `sandbox run --name <id> -- sim ...`.
- `sim` enforces one active instance by stopping prior tracked sim process before starting a new run.
- By default, `sim` auto-records NT4 data to `agent/logs` (`--record` is on by default).
- You can control auto-recording with `--record-address`, `--record-delay`, and `--record-output`, or disable with `--no-record`.
- Recorder failure does not fail the run if a new analyzable `.wpilog` is still generated during the same sim run.
- `sim` fails with `no_log_file_found` when no new analyzable `.wpilog` log is produced.
- By default, output is concise and bounded; use `--verbose` to stream full Gradle output.
- `--assert-key` and `--assert-range` support pass/fail validation without manual log scanning.
- Unless `--no-analyze` is used, it reports the summary of the newly generated/updated log from that run.

### sandbox

Manage isolated sandboxes under `~/.wpilib-agent-tools/sandboxes`.

#### create

```bash
wpilib-agent-tools sandbox create --name expA --source workspace
wpilib-agent-tools sandbox create --name expB --source branch:main
wpilib-agent-tools sandbox create --name expC --source rev:abc1234
```

Source options:

- `workspace`: snapshot current workspace state (including tracked diffs and untracked files)
- `branch:<name>`: use branch commit
- `rev:<sha>`: use explicit revision

#### list and status

```bash
wpilib-agent-tools sandbox list
wpilib-agent-tools sandbox status
wpilib-agent-tools sandbox status --name expA --json
```

#### run

```bash
wpilib-agent-tools sandbox run --name expA -- sim --duration 15
wpilib-agent-tools sandbox run --name expA -- query --mode avg --key "Shooter/Velocity"
wpilib-agent-tools sandbox run --name expA --detach -- ./gradlew test
wpilib-agent-tools sandbox run --name expA --max-lines 80 --tail --include "WARN|ERROR" -- sim --duration 15
wpilib-agent-tools sandbox run --name expA --verbose -- ./gradlew test
```

- Add `--detach` to return immediately and keep process running in sandbox
- Attached runs are concise by default; `--verbose` streams full child output
- `--max-lines`, `--tail`, and `--include` bound returned output excerpts
- Internal CLI commands (`sim`, `query`, etc.) are mapped automatically

#### stop, clean, patch

```bash
wpilib-agent-tools sandbox stop --name expA
wpilib-agent-tools sandbox clean --name expA
wpilib-agent-tools sandbox clean --all --older-than 24
wpilib-agent-tools sandbox patch --name expA --output expA.diff
```

- `stop --force` escalates to SIGKILL if needed
- `clean --force` can remove busy sandboxes
- `patch` requires a git-backed sandbox

### harness

Install shared workspace support for the three supported harnesses: Codex, Claude Code, and Cursor.

```bash
wpilib-agent-tools harness install --workspace /path/to/robot-repo
wpilib-agent-tools harness install --workspace /path/to/robot-repo --harnesses codex,claude
wpilib-agent-tools harness install --workspace /path/to/robot-repo --cursor-mode all
wpilib-agent-tools harness install --workspace /path/to/robot-repo --force --json
```

#### Behavior

- Creates a workspace-local runner at `.wpilib-agent-tools/run_cli.sh`
- Installs a managed Codex block into `AGENTS.md`
- Installs a managed Claude Code block into `CLAUDE.md`
- Installs a Claude Code slash command into `.claude/commands/`
- Installs Cursor rules into `.cursor/rules/`
- Keeps one shared installer path for all three harnesses

### rules

Install Cursor rule templates for sandbox-oriented workflows.

Use this command only when you specifically want the Cursor-only compatibility path. For near-parity multi-harness setup, prefer `harness install`.

```bash
wpilib-agent-tools rules install
wpilib-agent-tools rules install --mode all
wpilib-agent-tools rules install --force
wpilib-agent-tools rules install --target custom --output-dir /path/to/rules
wpilib-agent-tools rules install --json
```

#### Modes

| Mode | Templates installed | Default? |
| --- | --- | --- |
| `core` | `wpilib-agent-tools-core.mdc` — sandbox-first workflow, execution safety, evidence collection, robot mode alignment, self-discovery via `--help` | Yes |
| `all` | Everything in `core` plus example rules: `wpilib-agent-tools-token-efficient.mdc` — delegates broad searches and bulk output to subagents for lower context consumption | No |

#### Behavior

- Idempotent by default (existing files are skipped)
- `--force` overwrites existing installed files
- `--json` returns machine-readable install results
- Templates are intentionally opinionated about safety: stop prior runs before new execution, keep iteration in sandboxes, and apply reviewed patches only after explicit approval
- All installed templates use `alwaysApply: true` so Cursor applies them automatically once present

## Data Formats and Struct Decoding

### Supported log inputs

- `.wpilog` (WPILib DataLog format)

### Struct decoding in query

When reading `.wpilog`, `query --mode values` decodes several `struct:*` payloads to human-readable dictionaries:

- Geometry:
  - `Rotation2d`, `Translation2d`, `Pose2d`, `Transform2d`, `Twist2d`
  - `Quaternion`, `Rotation3d`, `Translation3d`, `Pose3d`, `Transform3d`, `Twist3d`
  - `Rectangle2d`, `Ellipse2d`
- Kinematics:
  - `ChassisSpeeds`
  - `SwerveModuleState`, `SwerveModulePosition`
  - `DifferentialDriveWheelSpeeds`, `DifferentialDriveWheelPositions`, `DifferentialDriveKinematics`
  - `MecanumDriveWheelSpeeds`, `MecanumDriveWheelPositions`, `MecanumDriveKinematics`
  - `SwerveDriveKinematics__N` (N-module struct type emitted by WPILib)
- Arrays:
  - Any fixed-size supported struct can be decoded as `struct:<Type>[]` (for example `Pose2d[]`, `SwerveModuleState[]`)

Unknown struct types are returned with fallback metadata (`wpilog_type`, `raw_size_bytes`, `raw_hex`) so samples remain inspectable.

## Filesystem Locations

Default paths used by the tool:

- Logs: `agent/logs`
- Graph output: `agent/visualizations`
- Reports directory placeholder: `agent/reports`
- Sandboxes root: `~/.wpilib-agent-tools/sandboxes`
- Sandbox metadata: `~/.wpilib-agent-tools/metadata`
- Sandbox locks: `~/.wpilib-agent-tools/locks`
- Sim runtime pid file: `~/.wpilib-agent-tools/runtime/sim.pid`

## Automation Scripts

### `agent/scripts/sandbox_lifecycle.sh`

Non-interactive wrapper around sandbox subcommands:

```bash
agent/scripts/sandbox_lifecycle.sh create --name expA --source workspace --json
agent/scripts/sandbox_lifecycle.sh list --json
agent/scripts/sandbox_lifecycle.sh delete --name expA
```

### `agent/scripts/cleanup_instances.sh`

Best-effort cleanup utility that stops common lingering tool/sim processes before a fresh run.

## Typical End-to-End Workflow

1. Create sandbox from current working state:

   ```bash
   wpilib-agent-tools sandbox create --name tune_shooter --source workspace
   ```

2. Run simulation and analysis in sandbox:

   ```bash
   wpilib-agent-tools sandbox run --name tune_shooter -- sim --duration 15
   wpilib-agent-tools sandbox run --name tune_shooter -- sim --duration 15 --assert-key Shooter/turretUsedUnwindFallback
   wpilib-agent-tools sandbox run --name tune_shooter -- query --mode stats --key "Shooter/Velocity" --summary --json
   wpilib-agent-tools sandbox run --name tune_shooter -- graph --key "Shooter/Velocity" --output shooter.png
   ```

3. Produce patch for review:

   ```bash
   wpilib-agent-tools sandbox patch --name tune_shooter --output tune_shooter.diff
   ```

4. Apply reviewed patch to workspace manually.

5. Clean sandbox unless you need to preserve it:

   ```bash
   wpilib-agent-tools sandbox clean --name tune_shooter
   ```

## Adaptive Validation Guidance

- Define acceptance checks before running simulation.
- Start with the minimum run set and expand only if a check is failing or inconclusive.
- Keep output bounded by default; use verbose output only for targeted debug passes.
- Prefer assertion-based checks and summary queries over raw log scans.

## Troubleshooting

- **No logs found / `no_log_file_found` after sim**: confirm NT4 is reachable at `--record-address`, increase `--record-delay` if NT4 starts late, or ensure your sim code emits `.wpilog` during the run.
- **Cannot read `.wpilog`**: verify `robotpy-wpiutil` is installed in current environment.
- **`record` connection failure**: check robot/server address and NT4 availability; try longer `--duration`.
- **`sim` refused in workspace**: run via `sandbox run ... -- sim ...` or add `--direct-workspace`.
- **`graph` produced no output**: verify selected key contains numeric or bool data.
- **`sandbox patch` failed**: patch generation requires a git-backed sandbox.

## Development

From `agent/`:

```bash
python -m pip install -e .
pytest -q
```

Run specific tests:

```bash
pytest -q tests/test_query_math_modes.py
pytest -q tests/test_nt_recorder.py
```
