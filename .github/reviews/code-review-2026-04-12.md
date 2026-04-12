# Code Review — Logical & Behavioral Audit

## Summary

The codebase has **two critical captured-variable bugs** in `Triggers.java` that cause hub-shift timing logic to evaluate once at construction and never update. There is a **stack-overflow infinite recursion** in `DriveCommands.stopWithX()`, a **null pointer crash** in the `isIntakeSafe` trigger, and several **joystick axis swaps** in pass-align bindings that would invert driver controls. The command files and subsystem architecture are generally sound, but the sequence/parallel compositions in shooting sequences contain subtle issues with `runOnce`-based commands finishing instantly inside sequential groups.

---

## Critical Issues

**1. `Triggers.isShootSafeTime()` — Captured variables, never re-evaluated (lines 136–138)**

```java
boolean disabled = HubShiftUtil.disabled;
boolean safe = HubShiftUtil.getShiftedShiftInfo().active();
return new LoggedTrigger("isShootSafeTime", () -> (safe || disabled));
```

`disabled` and `safe` are captured **once** when the trigger is constructed. The lambda closes over their values at that instant, so the trigger never reflects changes to `HubShiftUtil.disabled` or the current shift state. This means once the hub becomes inactive (or vice versa), the trigger will **never change its output**. This is the classic trigger construction bug.

**Fix:** Move both reads inside the lambda:
```java
return new LoggedTrigger("isShootSafeTime", () ->
    HubShiftUtil.getShiftedShiftInfo().active() || HubShiftUtil.disabled);
```

**2. `Triggers.isShootSafeTimeSure()` — Same captured-variable bug (lines 142–143)**

```java
boolean safe = HubShiftUtil.getShiftedShiftInfo().active();
return new LoggedTrigger("isShootSafeTimeSure", () -> safe);
```

Identical problem. `safe` is captured once. Since `isShootSafeTimeSure` drives the **flywheel idle speed ramp-up** in `configureStateBindings()`, this means the flywheel will either always or never idle at high speed, regardless of actual hub shift timing.

**3. `DriveCommands.stopWithX()` — Infinite recursion / StackOverflowError (line 398–399)**

```java
public static Command stopWithX(Drive drive) {
    Logger.recordOutput("RobotState/Drive", "Stopping with X");
    return Commands.run(() -> stopWithX(drive), drive);
}
```

The `Commands.run()` lambda calls `stopWithX(drive)` — the **same static method** — recursively. Every 20ms, `execute()` will call `stopWithX()` which creates a new Command object (and logs), never actually calling `drive.stopWithX()` or similar. This will either stack-overflow or simply do nothing useful. The intent is almost certainly `drive.stopWithX()` (an instance method on Drive).

**4. `Triggers.facingAllianceHub(null)` and `facingOpposingHub(null)` — NullPointerException (lines 165–169)**

`isIntakeSafe()` calls `facingAllianceHub(null)` and `facingOpposingHub(null)`. Inside those methods:
```java
double heading = AllianceFlipUtil.apply(pose.getRotation()).getDegrees();
```
`pose` is `null`, so `pose.getRotation()` throws a `NullPointerException` every time the trigger is polled. The intent is clearly to use the current estimated pose, but the parameter is hardcoded to `null`.

**5. `Triggers.isIntakeSafe()` — Logic is inverted (lines 162–170)**

The comment says "returns false if intake is in danger of crashing into hub." The trigger is named `isIntakeSafe`. The lambda returns `true` when close + facing + moving toward a hub — which represents the **unsafe** condition. But callers (e.g., `DriveCommands.driveLucasProof`) treat `isIntakeSafe().getAsBoolean()` as `true` = safe. This means the trigger returns `true` when the intake is **in danger**, and the speed limiter is disabled exactly when it should be active.

The `.or()` composition also means: "safe = (danger from alliance hub) OR (danger from opposing hub)." This should logically be `.negate()` or the internal condition should be inverted.

**6. `ShootSequences.shootToHub()` and `pass()` — `Logger.recordOutput` called at command construction time (lines 91, 124)**

```java
Logger.recordOutput("RobotState/shooting", true);
return Commands.parallel(...);
```

The `Logger.recordOutput` call runs when the **command is built** (e.g., at trigger binding time in `RobotContainer`), not when the command executes. This will log the wrong value at startup and never log again when the command actually runs. This should be inside an `initialize()` or wrapped in a `Commands.runOnce()`.

---

## Warnings

**7. `RobotContainer` pass-align bindings — Joystick axes are swapped (lines 407–408, 681–682)**

In `configureStateBindings()` and `configureSimBindings()`, the pass-align drive command passes:
```java
() -> -thrustmaster.getX(),  // passed as xSupplier
() -> -thrustmaster.getY(),  // passed as ySupplier
```

But the shoot-align just above and all other align commands pass `getY()` as `xSupplier` and `getX()` as `ySupplier`. The `joystickDriveAtAngle` signature is `(drive, xSupplier, ySupplier, rotation)` where `x = forward/back` and `y = strafe` (matching `getLinearVelocityFromJoysticks`). The thrustmaster's Y axis is forward/back, so it should be `xSupplier`. The pass-align has them **swapped**, which would invert the driver's controls when passing.

**8. `ShootSequences.shootForTower()` — Sequential feeding commands run one-at-a-time (lines 71–79)**

The feeding commands after the `WaitCommand(0.15)` are in a `Commands.sequence()`:
```java
Commands.sequence(
    new WaitCommand(0.15),
    FeederCommands.setLowerFeederVelocity(...),  // runOnce — finishes instantly
    FeederCommands.setUpperFeederVelocity(...),   // runOnce — finishes instantly
    TransportCommands.setTransportVoltage(...),   // startEnd — runs until interrupted
    intakeRollerCommands.setRollerVoltage(...));  // never reached
```

`setLowerFeederVelocity` and `setUpperFeederVelocity` are `runOnce` commands that finish instantly — that's fine. But `setTransportVoltage` is a `startEnd` command that **runs indefinitely** (until interrupted). Since it's in a sequence, `setRollerVoltage` will **never execute** because `setTransportVoltage` never finishes on its own. Compare with `shootToHub()` which correctly uses `Commands.parallel()` for this group.

**9. `PrestageCommands.prestageIdle()` — Missing subsystem requirement (lines 36–41)**

```java
public static Command prestageIdle(Prestage prestage) {
    return Commands.run(
        () -> prestage.setPrestageVelocity(...));
}
```

The `prestage` subsystem is **not** passed as a requirement to `Commands.run()`. This means the scheduler won't prevent conflicts — another command requiring `prestage` won't interrupt this one, and this one won't be recognized as using the subsystem.

**10. `IntakePivotCommands.zeroPivot()` — Missing subsystem requirement (line 97)**

```java
return Commands.runOnce(() -> intakePivot.zeroPivotEncoder());
```

`intakePivot` is not passed as a requirement. While this is likely only called manually, it could run concurrently with other IntakePivot commands without the scheduler knowing.

**11. `RobotState.getAngleToAllianceHub()` and `getAngleToTarget()` — `new Rotation2d().kPi` is a static field access on an instance (lines 345, 358)**

```java
return new Rotation2d(robotToHub.getX(), robotToHub.getY()).plus(new Rotation2d().kPi);
```

`new Rotation2d().kPi` creates a new default `Rotation2d` (0 radians) and then accesses the static field `kPi` on it. This **works** in Java (it's equivalent to `Rotation2d.kPi`), but it allocates an unnecessary object every call and is confusing to read. More importantly: the intent is to add 180° so the robot faces the hub with its shooter (presumably rear-mounted). Verify this is actually desired — if the shooter is front-mounted, this would aim the robot **away** from the hub.

**12. `RobotContainer.registerEventTriggers()` — `RunIntake` event trigger only runs intake roller, not transport (lines 345–356)**

The comment says "running intake rollers + transport," but the `startEnd` only controls `intakeRoller`. The `NamedCommands` version of `RunIntake` includes transport, but the `EventTrigger` version does not. This means during auto paths that use the zoned `RunIntake` event marker, the transport belt won't run, potentially jamming balls in the intake.

**13. Multiple flywheel commands competing on the same subsystem (`configureStateBindings` flywheel section)**

Several trigger bindings can be true simultaneously and all schedule commands requiring `flywheel`:
- `shootButton.and(isShootClear)` → `setVelocityForHub`
- `shootButton.and(!isShootSafeZone)` → `setPassVelocity`
- `shootFromTowerButton` → `setFlywheelVelocity` (tower)
- `isShootSafeTimeSure` → `setFlywheelVelocity` (idle high)
- `shootButton.and(TUNING_MODE)` → `setFlywheelVelocity` (tuning)

When `shootButton` is pressed in the alliance zone with the hub active, both the shoot and the idle-high triggers are true. These will fight for the flywheel subsystem, with whichever was scheduled last winning. The idle-high `onFalse(FlywheelCommands.stop(flywheel))` could also stop the flywheel when hub timing transitions while the driver is still holding the shoot button.

---

## Suggestions

- **`ShootSequences.shootEndBehavior()`** runs a sequence ending with `FlywheelCommands.setFlywheelVelocity(flywheel, idleVelocity)`, which is `runOnce` — it sets the velocity and instantly finishes. If nothing else is scheduled on the flywheel afterward, no command holds the flywheel requirement, and if a default command existed, it would take over immediately. This is probably fine as-is but consider whether idle speed should be a persistent command.

- **`SpitSequences.spitAll()` / `spitHopper()`** use `.finallyDo()` to stop motors. Since `.finallyDo()` doesn't declare requirements, there's a small window where the cleanup lambda could race with a newly-scheduled command on the same subsystem. Consider using `andThen()` stop commands instead.

- **`Triggers` creates new `LoggedTrigger` instances on every call** to methods like `isShootSafeZone()`, `isShootSafeTime()`, etc. Each call constructs a fresh trigger with its own `LoggingBooleanSupplier` internal state. If these are called frequently (e.g., inside lambdas or every loop), it wastes memory and resets the logging state tracker. Consider caching trigger instances as fields.

- **`HoodCommands.hoodIdle()`** sets position to 1 degree rather than sending zero voltage. The Javadoc says "continuously sends zero voltage," but the implementation does closed-loop position control. This is misleading.

- **No default commands for flywheel, prestage, hood, transport, feeders, or intake roller in real mode.** If a command ends (e.g., interrupted shoot sequence), these subsystems will hold whatever setpoint was last sent with no command to reset them. The commented-out default commands suggest this was intentional but worth verifying that hardware doesn't hold stale setpoints.
