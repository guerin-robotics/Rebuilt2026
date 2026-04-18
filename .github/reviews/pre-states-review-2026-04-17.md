# Pre-States Code Review — April 17, 2026

## Summary

The codebase is in solid shape overall. The previous round of bug fixes (captured trigger lambdas, CPU optimizations) all look correct. There are a handful of things that could bite you at states, ranging from "will definitely cause problems if triggered" to "probably fine but worth knowing about."

---

## Critical Issues

### 1. `HoodDown` EventTrigger will interrupt the entire auto group

**File:** `RobotContainer.java`, line 362–364

The comment block above `registerEventTriggers()` (lines 318–330) correctly explains that EventTrigger commands must NOT declare subsystem requirements that overlap with the auto group, or the scheduler will cancel the entire auto. The `DeployIntake`, `RetractIntake`, and `RunIntake` triggers all follow this rule — but `HoodDown` doesn't:

```java
new EventTrigger("HoodDown")
    .onTrue(
        HoodCommands.setHoodPos(hood, HardwareConstants.CompConstants.Positions.hoodDownPos));
```

`HoodCommands.setHoodPos()` passes `hood` as a requirement. The auto's "Shoot" NamedCommand includes `HoodCommands.setHoodPosForHub(hood)`, which also requires `hood`. If any auto path uses a `HoodDown` event marker, the scheduler will kill the entire auto sequence.

**Fix:** Wrap it the same way as the others — `Commands.runOnce(() -> hood.setHoodPos(...))` without passing `hood` as a requirement.

### 2. `RunIntake` EventTrigger doesn't run transport — NamedCommand version does

**File:** `RobotContainer.java`, lines 350–359 vs 283–291

The `RunIntake` NamedCommand runs both `intakeRoller` and `transport`. The `RunIntake` EventTrigger only runs `intakeRoller`. If your auto paths use the zoned event marker version (which they should for intake-while-driving), balls will get stuck between the intake and the hopper because the transport belt isn't moving.

This was flagged in the April 12 review but the code hasn't changed.

---

## Warnings

### 3. Shoot button + pass trigger can both fire simultaneously

**File:** `RobotContainer.java`, lines ~449–467 (hub shoot flywheel) and ~478–498 (pass flywheel)

When the shoot button is pressed and `isShootSafeZone` is true and `isShootClear` is true, the hub-shoot trigger fires. But the pass trigger condition is `shootButton AND NOT isShootSafeZone`. Since `isShootClear = isShootSafeTime AND isShootSafeZone`, these two should be mutually exclusive in theory. However:

If the robot crosses the zone boundary mid-press (you're driving while holding shoot), there's a brief window where `isShootSafeZone` flips. The hub-shoot trigger's `whileTrue` command gets interrupted and its `.onFalse` stop commands fire. Then the pass trigger activates, scheduling new commands. The `.onFalse` stops from the hub trigger and the `.whileTrue` starts from the pass trigger race against each other for the same subsystems during that transition frame.

This probably won't cause a crash, but you might see a momentary velocity dip on the flywheel as it gets stopped and restarted. If you notice inconsistent shots near the zone line, this is why.

### 4. `compressPivot` has a stray `Logger.recordOutput` at construction time

**File:** `IntakePivotCommands.java`, line 94

```java
Logger.recordOutput("RobotState/IntakePivot", "JostleCalled");
return Commands.sequence(...)
```

That log call runs when the command object is *created* (at robot boot during `configureStateBindings`), not when the command *executes*. It's harmless but it'll write "JostleCalled" to the log once at startup and never again, which is misleading if you're trying to debug pivot behavior. Move it inside the command if you want it to log at execution time, or just delete it.

### 5. `compressPivot` doesn't reset pivot position on end

**File:** `IntakePivotCommands.java`, lines 95–104 vs `RobotContainer.java` line 597–599

The `compressPivot` command is a finite sequence (wait → middle → wait → up) with no `finallyDo` or `repeatedly()`. When the shoot button is released and the `whileTrue` interrupts this command, the pivot just stops wherever it is. The `jostlePivotByPos` command has a `finallyDo` that resets position — `compressPivot` doesn't.

In `configureStateBindings`, there's an `.onFalse` that sets pivot to `pivotDownPos`, which handles this. But in `configureSimBindings` (line 834), there is no `.onFalse` — the pivot will stay at whatever position it was interrupted at.

### 6. `zonePassOrShoot` uses `Commands.either()` — evaluates condition once

**File:** `ShootSequences.java`, lines 210–244

`Commands.either()` evaluates its boolean supplier once at schedule time and picks one branch permanently. If the robot drives across the zone boundary while this command is running, it won't switch from pass to shoot or vice versa. The `zoneAndTimePassOrShoot` wrapper uses `ContinuousConditionalCommand` which re-evaluates, but `zonePassOrShoot` by itself doesn't. Just make sure you're not calling `zonePassOrShoot` directly from anywhere that expects live switching.

---

## Minor Notes

- The `default` case in `RobotContainer`'s constructor only creates 2 `VisionIO` instances but the real/sim cases create 4. Won't matter in practice (replay mode will use the log data) but it's inconsistent.
- `getThrustX/Y/Rot` don't apply the deadband — `deadband()` is defined but never called. The raw axis values go straight to the drive commands. If joystick drift is an issue at states, this is the place to look.
- Class name `intakeRollerCommands` doesn't follow Java naming conventions (lowercase start). Not a bug, but it'll confuse anyone reading the code later.
