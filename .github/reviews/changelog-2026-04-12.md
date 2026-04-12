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

