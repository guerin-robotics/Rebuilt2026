# Tuning Tips

## Recorder Timing

Simulation startup and NT4 availability are not instantaneous.

Practical baseline:

- `--duration 30`
- `--record-delay 3`

Why:

- Gives startup headroom for simulator and NT4 server.
- Still captures early behavior transitions.

Practical startup windows:

- roughly 5 seconds for sim/NT initialization
- at least 10 seconds for behavior execution window
- many full autonomous path routines are closer to 15-20 seconds

## Delay Tradeoffs

- Lower `record-delay` captures earlier state transitions.
- Higher `record-delay` can avoid early noise but may miss short paths.

Example learned behavior:

- `record-delay 8` captured only post-path `IDLE` in one run.
- `record-delay 3` captured `FOLLOW_PATH -> IDLE` in the same repo.

## If Behavior Is Missed

Use this order:

1. Lower `--record-delay`.
2. Increase `--duration`.
3. Add a startup `WaitCommand` (or equivalent gate) before the behavior under test, then re-run.

The same approach applies to non-path behavior tests (subsystem state, command triggers, safeguards), not only autonomous path tracking.

## Validation Signal Priority

Use telemetry as source of truth:

1. DS mode values
2. behavior state sequence
3. log generation success

Use process exit as secondary context only.
