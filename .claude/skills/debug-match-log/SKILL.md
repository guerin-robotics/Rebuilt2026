---
name: debug-match-log
description: Analyze an AdvantageKit match log (.wpilog/.hoot) to root-cause a field problem. Use when something went wrong on the robot during a match or practice and a log file exists. Correlates signals around the anomaly, tests hypotheses against the log, and sets up deterministic replay.
---

# Debug a Match Log

You are analyzing an AdvantageKit log to find the root cause of an on-field
problem. This is a **read-only** investigation — do not modify robot code
unless the user explicitly asks for a fix afterward.

## Step 1 — Gather the facts

If the user hasn't provided them, ask for:

1. **Log file path** (`.wpilog` or `.hoot` — usually from the USB stick or `/U/logs`)
2. **What went wrong** — what the robot did vs. what it should have done, as specifically as possible
3. **When it happened** — match time, or an event ("3rd auto shot", "first teleop shoot press")
4. **What the human saw** — physical behavior: directions, oscillation frequency, sounds, anything

Remember: the log has the data; the human has the physical context. You need both.

## Step 2 — Form hypotheses, then check signals

For each plausible cause, state **what the log would show if this hypothesis
were true**, then check. Work through hypotheses one at a time and report
which signals confirm or eliminate each.

### Key signal map for common problems

**Robot didn't shoot when button was pressed**
```
Triggers/isShootClear              — was the composite trigger true?
Triggers/isShootSafeZone           — was robot in alliance zone?
Triggers/isShootSafeTime           — was the hub active?
RobotState/IsAlignedToHub          — was heading within tolerance?
Flywheel/[leader]/closedLoopRef    — was flywheel targeting a velocity?
Flywheel/isSpunUp                  — did spinup condition pass?
```

**Robot aligned wrong direction**
```
RobotState/AngleToAllianceHub      — what heading was being targeted?
RobotState/EstimatedPose           — was the pose correct?
Drive/Odometry/Robot               — field visualization
Vision/Camera[0-3]/AcceptedPose    — were vision updates being applied?
```

**Auto ran wrong path / started from wrong position**
```
Auto/SelectedAuto                  — what auto was selected?
Auto/StartPose                     — expected starting pose
Auto/StartCheck/TranslationError   — how far off was the robot?
Auto/StartCheck/RotationError      — rotation error at start
Odometry/Trajectory                — what path was being followed?
```

**Vision caused pose jump**
```
Vision/Camera[N]/AcceptedPoses     — which camera sent a pose?
Vision/Camera[N]/RejectedPoses     — what was rejected and why?
RobotState/FieldRelativeVelocity   — was the robot moving fast at the jump?
Drive/Odometry/Robot               — visualization of the jump
```

**Brownout during match**
```
PowerDistribution/Voltage          — when did voltage drop below 7 V?
BatteryLogger/[Subsystem]/Current  — which subsystem drew the most current?
[Subsystem]/StatorAmps             — stall detection per subsystem
```

**Intake didn't compress**
```
IntakePivot/GoalPosition           — what position was being commanded?
IntakePivot/ActualPosition         — what position was reached?
Commands/Active                    — was the compress command scheduled?
Triggers/compressCancelled         — was the cancellation flag set?
```

Also always check `Commands/Active` around the event time — a missing or
unexpectedly-interrupted command explains most "it just didn't do it" reports.

## Step 3 — Report

Deliver:
1. **Root cause** (or the shortlist, ranked, with the evidence for each)
2. **The evidence** — which signals showed what, at what timestamps
3. **What was ruled out** and why
4. **Suggested fix** as a description only — do not implement it in this
   session unless asked. If the fix touches gains, thresholds, timeouts, or
   vision filters, name the risk tier and the failure mode per the rules.
5. **Regression-test note** — once a fix lands, a test must lock the bug in
   (see `.claude/prompts/write-test.md`).

## Replay: adding instrumentation

If the existing signals aren't enough to decide between hypotheses, use
deterministic replay to add new outputs:

1. Set `Constants.getMode()` to `REPLAY`
2. Point `WPILOGReader` at the log file
3. Add `Logger.recordOutput()` calls for the values you need
4. Run `./gradlew simulateJava`
5. Open AdvantageScope and connect to the replay output

Same inputs, same code path, same bug, every time — with new eyes on it.
