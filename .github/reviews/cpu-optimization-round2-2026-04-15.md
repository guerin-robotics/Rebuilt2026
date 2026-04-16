# CPU Optimization Review — Round 2

**Date:** 2025-04-15
**Branch:** `cpu-optimization`
**Build:** ✅ SUCCESSFUL

---

## Summary

Removed debug logging and dead code across 12 files. These were all running every 20 ms loop and contributing to unnecessary CPU load.

---

## Debug Logging Removed

### 1. `getCurrentCommand()` logging — 8 subsystems

Removed `Logger.recordOutput("<Name>/CurrentCommand", ...)` from `periodic()` in:

| Subsystem | File |
|-----------|------|
| LowerFeeder | `subsystems/lowerFeeder/LowerFeeder.java` |
| UpperFeeder | `subsystems/upperFeeder/UpperFeeder.java` |
| Hood | `subsystems/hood/Hood.java` |
| Flywheel | `subsystems/flywheel/Flywheel.java` |
| Prestage | `subsystems/prestage/Prestage.java` |
| IntakePivot | `subsystems/intakePivot/IntakePivot.java` |
| intakeRoller | `subsystems/intakeRoller/intakeRoller.java` |
| Transport | `subsystems/transport/Transport.java` |

**Why:** Each call invoked `getCurrentCommand()` twice per loop (null-check + `.getName()`), totaling 16 virtual method lookups across 8 subsystems every 20 ms. Pure debug info with no match-debugging value.

### 2. Zone logging in `Robot.java`

Removed three `Logger.recordOutput` calls from `robotPeriodic()`:
- `RobotState/broadZone`
- `RobotState/specificZone`
- `RobotState/approachingZone`

**Why:** Each call invoked `getBroadZone()`, `getSpecificZone()`, `getApproachingZone()` on `RobotState` every loop. These zone values are only consumed by triggers (which already evaluate them on-demand). Logging them separately was redundant.

### 3. ShotCalculator debug logging

Removed 4 `Logger.recordOutput` calls from `ShotCalculator`:
- `Flywheel/ShotCalculator/TargetPosition` (Translation3d)
- `Flywheel/ShotCalculator/RobotPosition` (Translation2d)
- `Flywheel/ShotCalculator/Distance_m` (double)
- `Flywheel/ShotCalculator/CalculatedSpeed_RPM` (double)

**Why:** These fire on every shot calculation call (which runs every 20 ms as the default flywheel command). Serializing geometry objects is expensive.

### 4. `RobotState.getPassTarget()` debug log

Removed `Logger.recordOutput("Flywheel/passTarget", passTarget)`.

**Why:** Logs a `Translation3d` every time pass target is queried. Pure debug.

### 5. `HoodPosCalculator.getHoodPosForDistance()` debug log

Removed `Logger.recordOutput("Hood/HoodPosCalculator/HoodPosDegrees", hoodPosDegrees)`.

**Why:** Fires every 20 ms from Hood's default command. The hood position is already logged via `@AutoLogOutput` on the Hood IO inputs.

---

## Dead Code Removed

### 6. `ShotCalculator` — 4 dead methods

| Method | Reason |
|--------|--------|
| `getDistanceToTarget(Translation3d)` | Never called from any code |
| `isInShootingRange()` | Never called from any code |
| `getMinTimeOfFlight()` | Never called from any code |
| `getMaxTimeOfFlight()` | Never called from any code |

Also removed now-unused imports: `inchesToMeters`, `Logger`.

### 7. `Flywheel.shootDynamic()` — dead method

Never called from any command or other code. Was a placeholder for shoot-on-the-move. Also cleaned 6 unused imports (`MetersPerSecond`, `RotationsPerSecond`, `Translation2d`, `LinearVelocity`, `FieldConstants`, `RobotState`).

### 8. `Drive.getHeadingForShootDynamic()` — dead method

Never called from any code. Companion to `shootDynamic()`. Also removed unused `FieldConstants` import.

---

## Import Cleanup

| File | Removed Imports |
|------|----------------|
| `RobotState.java` | `org.littletonrobotics.junction.Logger` |
| `HoodPosCalculator.java` | `org.littletonrobotics.junction.Logger` |
| `ShotCalculator.java` | `org.littletonrobotics.junction.Logger`, `inchesToMeters` |
| `Flywheel.java` | `MetersPerSecond`, `RotationsPerSecond`, `Translation2d`, `LinearVelocity`, `FieldConstants`, `RobotState` |
| `Drive.java` | `frc.lib.FieldConstants` |

---

## Per-Loop Savings Estimate

| Item | Calls saved per loop |
|------|---------------------|
| `getCurrentCommand()` (×2 per subsystem × 8) | 16 |
| Zone calculations (broad + specific + approaching) | 3 method calls + internal zone logic |
| ShotCalculator logging (4 Logger.recordOutput) | 4 serializations |
| passTarget logging (Translation3d) | 1 serialization |
| HoodPosCalculator logging | 1 serialization |
| **Total Logger.recordOutput calls removed** | **17** |

---

## What's Still Logging (intentionally kept)

- `@AutoLogOutput` methods on `RobotState` (pose, velocity, distance, angle) — essential telemetry
- `Logger.processInputs()` in every subsystem — required by AdvantageKit for replay
- `BatteryLogger` current tracking — useful for brownout diagnosis
- `FlywheelVisualizer` trajectory — gated behind `MIN_RPM_FOR_TRAJECTORY` threshold
- HubShift logging in `teleopPeriodic()` — game-specific, needed during matches
