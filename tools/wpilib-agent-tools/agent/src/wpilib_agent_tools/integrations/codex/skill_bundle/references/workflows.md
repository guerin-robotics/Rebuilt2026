# Workflows

## Direct Analysis (No Sandbox Required)

Use this path when no code edits are required.

```bash
wpilib-agent-tools logs --json
wpilib-agent-tools keys --file <log.wpilog> --json
wpilib-agent-tools query --mode stats --file <log.wpilog> --key "<key>" --json
wpilib-agent-tools graph --file <log.wpilog> --key "<key>" --output plot.png --json
```

Tips:

- Prefer `--json` for machine parsing.
- Prefer `--summary`, `--limit`, and `--max-lines` for bounded output.

## Direct Behavior Validation (No Code Edit)

Use this when the target repo already contains the behavior under test.

```bash
wpilib-agent-tools sim --duration 20 --record-delay 3 --json
wpilib-agent-tools query --mode ds --json
wpilib-agent-tools query --mode stats --key "<behavior/key>" --json
wpilib-agent-tools graph --key "<behavior/key>" --output behavior.png --json
```

For assertion-led checks:

```bash
wpilib-agent-tools sim \
  --duration 20 \
  --record-delay 3 \
  --assert-key "<boolean_or_state_key>" \
  --assert-range "<numeric_key>" <min> <max> \
  --json
```

## Sandbox Iteration Workflow

Use this path for robot-code iteration plus sim validation.

```bash
wpilib-agent-tools sandbox create --name <id> --source workspace
wpilib-agent-tools sandbox run --name <id> -- sim --duration 30 --record-delay 3 --json
wpilib-agent-tools sandbox run --name <id> -- query --mode ds --json
wpilib-agent-tools sandbox patch --name <id> --output <id>.diff
wpilib-agent-tools sandbox clean --name <id>
```

Rules:

- Keep generated artifacts in sandbox unless asked otherwise.
- Export patch only after collecting evidence.
- Keep all behavior checks tied to log evidence, not process exit alone.
- If startup timing drops early behavior from logs, tune `--record-delay` and duration, then consider adding startup wait/gating in robot command flow.

## Reusable Validation Script Workflow

Use `scripts/validate_robot_repo.sh` for repeatable validation and a single report.

```bash
scripts/validate_robot_repo.sh --repo /path/to/repo --profile generic
```

Generic profile with explicit checks:

```bash
scripts/validate_robot_repo.sh \
  --repo /path/to/repo \
  --profile generic \
  --check-ds \
  --state-key "<behavior/key>" \
  --expected-states "STATE_A,STATE_B"
```

For the author-specific repo profile example:

```bash
scripts/validate_robot_repo.sh \
  --repo ~/FRC/2026-Robot-Code \
  --branch comp-dev \
  --profile 2026-robot-code
```
