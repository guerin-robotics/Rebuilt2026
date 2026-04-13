# Code Review Fixes Changelog — 2026-04-12

All changes made in response to the code review on `code-review-of-main` branch.

---

## Changes Made

### `Triggers.java` — Issues 1, 2, 4, 5, T1

- **T1 (Trigger Caching)**: Converted all state-based trigger methods (returning `new LoggedTrigger(...)`) to `public final` or `private final` fields initialized once. Button mapping triggers remain as methods (thin wrappers). `isHoodSafe(Pose2d)` remains a method since it takes a parameter.
- **Issue 1 (`isShootSafeTime`)**: Moved `HubShiftUtil.disabled` check and `HubShiftUtil.getShiftInfo().active()` read inside the lambda so they evaluate at trigger-poll time instead of construction time.
- **Issue 2 (`isShootSafeTimeSure`)**: Moved `HubShiftUtil.getShiftInfo().active()` read inside the lambda.
- **Issue 4 (`facingAllianceHub`/`facingOpposingHub`)**: Removed `Pose2d` parameter. Now reads `RobotState.getInstance().getEstimatedPose()` inside the lambda, eliminating the `null` NPE risk.
- **Issue 5 (`isIntakeSafe`)**: Added `.negate()` to the composed trigger. Internal logic detects DANGER (close + facing + moving toward hub), then negation means `true` = safe. Verified the only caller (`driveLucasProof` in `DriveCommands`) expects `true` = safe.
- Sub-triggers (`tooCloseToAllianceHub`, `tooCloseToOpposingHub`, `facingAllianceHub`, `facingOpposingHub`, `movingTowardAllianceHub`, `movingTowardOpposingHub`) made `private final` fields.
- Zone triggers (`isRobotInTrench`, `isRobotApproachingTrench`, `isRobotOnBump`, `isRobotApproachingBump`) made `public final` fields.

### `DriveCommands.java` — Issue 3

- **Issue 3 (`stopWithX` infinite recursion)**: Changed `Commands.run(() -> stopWithX(drive), drive)` to `Commands.run(() -> { Logger.recordOutput(...); drive.stopWithX(); }, drive)`. Also fixed `stopWithXAfterWait()` similarly.

### `ShootSequences.java` — Issues 6, 8

- **Issue 6 (Logger at construction)**: In `shootToHub()` and `pass()`, moved `Logger.recordOutput` calls from construction-time evaluation into `Commands.runOnce()` wrappers inside the parallel composition.
- **Issue 8 (`shootForTower` sequential blocking)**: Changed the feeding commands after the `WaitCommand` from `Commands.sequence()` to `Commands.parallel()`. The `setTransportVoltage` is a `startEnd` command (never finishes naturally), so in a sequence it would block `setRollerVoltage` from ever running.

### `PrestageCommands.java` — Issue 9

- **Issue 9 (`prestageIdle` missing subsystem requirement)**: Added `prestage` as a subsystem requirement to `Commands.run()` in `prestageIdle()`, preventing concurrent command conflicts.

### `RobotState.java` — Issue 11

- **Issue 11 (`Rotation2d.kPi` cleanup)**: Changed `new Rotation2d().kPi` to `Rotation2d.kPi` in both `getAngleToAllianceHub()` and `getAngleToTarget()`. Added comments explaining the 180° offset is because the shooter faces the back of the robot. Fixed `getAngleToTarget` comment from "point at the hub" to "point at the target".

### `RobotContainer.java` — Issues 7, 12, 13, T2

- **T2 (Field access syntax)**: Updated all call sites from `.triggerName()` to `.triggerName` for triggers that were converted to fields (e.g., `isShootClear`, `isShootSafeZone`, `isShootSafeTimeSure`, `isShootSafeTime`, `isIntakeSafe`, zone triggers).
- **Issue 7 (Pass-align axis swap)**: In both `configureStateBindings()` and `configureSimBindings()`, swapped X/Y joystick suppliers in pass-align `joystickDriveAtAngle` calls to match the correct convention (Y=forward as xSupplier, X=strafe as ySupplier), consistent with all other align commands.
- **Issue 12 (RunIntake comment)**: Updated comment from "running intake rollers + transport while in a zoned area" to "running intake rollers while in a zoned area" to accurately reflect the code (only runs intake roller, not transport).
- **Issue 13 (Idle-high flywheel conflict)**: Added `.and(() -> !Triggers.getInstance().shootButton().getAsBoolean())` to the idle-high flywheel trigger, preventing the idle-high command from conflicting with active shooting commands.

### Skipped

- **Issue 10 (`zeroPivot` missing subsystem requirement)**: Skipped per user request — acceptable since it only resets the encoder.

---

## Subsystem IO Layer Review — Fixes

All changes below address issues found during a second review pass focused on IO
interface implementations and sim/real parity.

### Critical Fixes

#### `FlywheelIOSim.java` — Follower 1 Alignment (Bug #2)

Sim configured `follower1` with `MotorAlignmentValue.Opposed`, but real hardware
(`FlywheelIOPhoenix6`) configures it as `Aligned`. Simulated follower was spinning
in the wrong direction.

- **Change:** `MotorAlignmentValue.Opposed` → `MotorAlignmentValue.Aligned`

#### `intakeRollerIOSim.java` — Follower Alignment (Bug #3)

Sim configured the follower as `Aligned`, but real hardware (`intakeRollerIOReal`)
configures it as `Opposed`. Simulated follower was spinning in the wrong direction.

- **Change:** `MotorAlignmentValue.Aligned` → `MotorAlignmentValue.Opposed`

#### `PrestageIOSim.java` — Swapped ClosedLoopReference / Error (Bugs #4 & #5)

`prestageLeftClosedLoopReference` was reading `getClosedLoopError()` instead of
`getClosedLoopReference()`, and `prestageLeftClosedLoopError` was never populated.

- **Changes:** Reference now reads `getClosedLoopReference()`; added error reading `getClosedLoopError()`.

#### `LowerFeeder.java` / `UpperFeeder.java` — Duplicate Logger Key (Bug #6)

Both used `"Feeder"` as their `Logger.processInputs` key, causing AdvantageKit
log collisions — one subsystem's data would overwrite the other.

- **Changes:** LowerFeeder → `"Feeder/Lower"`, UpperFeeder → `"Feeder/Upper"`

#### `PrestageIOSim.java` — Right Motor Never Commanded (Bug #10)

`setPrestageVoltage()` and `setPrestageVelocity()` only commanded the left motor.
The right motor was constructed but never received any control requests.

- **Change:** Added `prestageRight.setControl(...)` calls in both methods.

### Warnings Fixed

#### `FlywheelIOSim.java` — Missing leaderAngle (Bug #7)

`inputs.leaderAngle` was never populated in `updateInputs()`.

- **Change:** Added `inputs.leaderAngle = leader.getPosition().getValue();`

#### All IO Inputs — Null Measure Defaults (Bug #8)

All `Measure<?>` fields in `@AutoLog` inputs classes had no default values. If
`updateInputs()` is skipped before the first `periodic()`, AdvantageKit's
auto-logger would hit a `NullPointerException`.

- **Files:** `FlywheelIO`, `IntakePivotIO`, `intakeRollerIO`, `LowerFeederIO`, `PrestageIO`, `TransportIO`, `UpperFeederIO`
- **Change:** Every `Measure` field now has a zero-value initializer (e.g., `= RotationsPerSecond.of(0)`).

#### `IntakePivotIOSim.java` — Wrong NeutralMode (Bug #9)

Sim used `NeutralModeValue.Coast` while real hardware uses `Brake`. Pivot would
sag under gravity in sim but hold position on real hardware.

- **Change:** `NeutralModeValue.Coast` → `NeutralModeValue.Brake`

#### `FlywheelConstants.java` — Motor Count Mismatch (Bug #16)

`DCMotor.getKrakenX60Foc(4)` was used in the sim gearbox, but real hardware has
1 leader + 4 followers = 5 motors. Sim was modeling 20% less torque.

- **Change:** `DCMotor.getKrakenX60Foc(4)` → `DCMotor.getKrakenX60Foc(5)`

### Suggestions Implemented

#### `intakeRollerIOSim.java` — Missing Follower Fields (S6)

`updateInputs()` only populated leader fields. Follower velocity, voltage, current,
temperature, closed-loop status, and position were never written.

- **Change:** Added all follower input field assignments mirroring the leader pattern.

#### Missing Position Fields in Sim IOs (S7)

Position fields were defined in IO interfaces but never populated in sim.

- **Files:** `LowerFeederIOSim`, `UpperFeederIOSim`, `TransportIOSim`, `PrestageIOSim`
- **Change:** Added `inputs.<field> = motor.getPosition().getValue();` in each sim's `updateInputs()`.

### Other

#### `IntakePivotCommands.java` — Dead Command Factory

`jostlePivotByCurrent()` referenced `IntakePivot.intakeJostleByCurrent()` which no
longer exists, causing a compile error. The method was never called anywhere.

- **Change:** Removed the entire `jostlePivotByCurrent` command factory method.

### Skipped (per user instruction)

| # | Issue | Reason |
|---|-------|--------|
| 1 | HoodIOSim follower alignment | Already fixed |
| 11 | HoodIOSim sim physics | Subsystem removed |
| 12 | IntakePivotIOSim physics accuracy | Skip physics tuning |
| 13 | HoodIO null defaults | Subsystem removed |
| 14 | IntakePivotIOSim voltage clamp | Leave as voltage |
| 15 | intakeRollerIOSim FOC mismatch | Don't change |
