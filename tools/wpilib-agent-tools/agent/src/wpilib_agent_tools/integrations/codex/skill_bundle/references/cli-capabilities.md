# CLI Capabilities

This reference helps agents operate the full CLI directly, not only the automation wrapper scripts.

## Execution Path Selection

Use the lightest path that satisfies the goal:

1. Direct CLI mode: analysis and evidence from existing logs, no code edits required.
2. Sandbox mode: code edits or behavior experiments in isolation.
3. Validator script mode: repeatable pass/fail report for a target repo/profile.

## Command Matrix

| Command | Primary use | High-value flags | Common failure signal | First recovery action |
| --- | --- | --- | --- | --- |
| `logs` | List available logs and metadata | `--dir`, `--summary`, `--max-lines`, `--json` | Empty set or wrong directory | Point `--dir` to expected log root |
| `keys` | Discover keys in a log | `--file`, `--filter`, `--summary`, `--json` | Missing key | Re-check file selection and broaden filter |
| `query` | Data extraction, stats, DS state, derived signals | `--mode`, `--key`, `--summary`, `--limit`, `--json` | No samples for selected key/mode | Verify key path with `keys`, widen time window |
| `graph` | Generate PNG evidence from numeric series | `--key`, `--mode`, `--output`, `--start`, `--end`, `--json` | Empty/non-useful graph | Confirm key is numeric and time range has samples |
| `record` | Record live NT4 stream to WPILOG | `--address`, `--duration`, `--keys`, `--output`, `--json` | Connection timeout/no data | Confirm NT4 endpoint and increase duration |
| `view` | Open a log in AdvantageScope/system opener | `--file` | Viewer not available | Use `query`/`graph` for non-GUI evidence |
| `math` | Symbolic/numeric quick checks | `--mode`, `--expr`, `--equation`, `--value`, `--json` | Invalid expression/equation | Reduce expression and validate syntax |
| `sim` | Run bounded simulation with optional auto-recording and assertions | `--duration`, `--record-delay`, `--assert-key`, `--assert-range`, `--json` | `no_log_file_found`, missing early behavior | Increase duration, lower record delay, add startup wait in robot command flow |
| `sandbox` | Isolated edit/run lifecycle | `create`, `run`, `status`, `stop`, `patch`, `clean`, `--json` | Busy/stale sandbox or run collision | `sandbox status`, then `sandbox stop`/`clean --force` as needed |
| `harness` | Install shared workspace support for Codex, Claude Code, and Cursor | `install --workspace`, `--harnesses`, `--cursor-mode`, `--force`, `--json` | Workspace mismatch or stale generated files | Re-run with correct `--workspace` and `--force` if replacing generated files |
| `rules` | Install Cursor rule templates | `install --mode core|all`, `--target`, `--force`, `--json` | Files already present or target mismatch | Re-run with `--force` or correct target/output-dir |

## Fluent Usage Rules

- Prefer `--json` for machine parsing.
- Start bounded, then expand:
  - `--summary`
  - `--limit`
  - `--max-lines`
- For unknown command details, run:
  - `scripts/run_cli.sh --help`
  - `scripts/run_cli.sh <subcommand> --help`

## Failure Interpretation

- Bounded sim completion may end with raw termination semantics, but pass/fail should come from telemetry and assertions.
- Use `query --mode ds` and targeted state queries to confirm behavior.
- Use process exit as secondary context, not sole evidence.

## Behavior Validation Patterns

Path behavior:

```bash
wpilib-agent-tools sim --duration 30 --record-delay 3 --json
wpilib-agent-tools query --mode values --key "Robot/AutoState" --json
```

Non-path subsystem behavior:

```bash
wpilib-agent-tools sim --duration 20 --record-delay 3 \
  --assert-key "Shooter/turretUsedUnwindFallback" \
  --assert-range "Shooter/turretResolvedSetpointDeg" -450 630 \
  --json
```

Timing fallback when behavior starts too early for recording:

- Add a startup `WaitCommand` (or equivalent behavior gate) before the tested logic.
- Re-run with tuned `--record-delay` and duration.
