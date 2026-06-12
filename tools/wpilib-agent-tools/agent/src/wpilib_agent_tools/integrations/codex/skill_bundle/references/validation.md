# Validation

## Goal

Produce a deterministic pass/fail result for simulation and robot behavior using CLI evidence.

This is not only a sandbox health check. It validates that behavior evidence can be produced from a sandboxed run.

## Script

Primary entrypoint:

```bash
scripts/validate_robot_repo.sh --repo <path> [options]
```

Important options:

- `--profile generic|2026-robot-code`
- `--branch <name>`
- `--duration <seconds>`
- `--record-delay <seconds>`
- `--state-key <log-key>`
- `--expected-states CSV` (ordered subsequence check)
- `--check-ds`
- `--keep-sandbox`
- `--keep-sandbox-on-fail`

## Profile Behavior

`generic`:

- Portable baseline for unknown repos.
- No repo-specific code patching.
- `duration=30`, `record_delay=3` defaults.
- DS/state checks only when explicitly requested by flags.

`2026-robot-code`:

- Author-specific example profile for `~/FRC/2026-Robot-Code`.
- Applies temporary sandbox patching for known simulation/DS setup.
- Defaults include DS and state-sequence checks plus `auto_path=straight`.
- Intended as an example pattern for profile-driven validation.

## Evaluation Logic

The validator reports pass when all required checks are true:

1. Sim produced a log (`log_generation.passed=true`).
2. Optional DS check (`--check-ds`) shows `Enabled=true` and `Autonomous=true`.
3. Optional state-sequence check confirms ordered appearance of expected states.

Exit code:

- `0`: passed
- `1`: failed

## Notes On Sim Exit Code

Bounded sim runs intentionally terminate the gradle process. In CLI output, `exit_code` is normalized to success while `exit_code_raw` may still be `143`.

Use telemetry checks (DS values, state sequence, assertions) as primary evidence.

## Troubleshooting

- If DS keys are missing, verify DS auto-enable logic in target repo startup path.
- If state transitions are missing, lower `--record-delay` and re-run.
- If behavior starts too early to capture reliably, add a startup `WaitCommand` (or equivalent gate), then re-run.
- If recorder fails to connect, increase duration and adjust delay/address.
