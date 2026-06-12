# Rebuilt2026 Architecture
*Guerin Robotics — FRC 2025/2026 Competition Robot*
*Documented post-Worlds. Treat as proven-but-imperfect reference material.*

---

## Project Layout

```
src/main/java/frc/robot/
├── Robot.java                    # LoggedRobot lifecycle
├── RobotContainer.java           # Subsystem wiring + all button bindings
├── RobotState.java               # Singleton: pose, zones, shot angles
├── Triggers.java                 # Singleton: button objects + state triggers
├── HardwareConstants.java        # All CAN IDs, tuning values, zone boundaries
├── Constants.java                # Mode selector (REAL/SIM/REPLAY) + robot type
├── BuildConstants.java           # Auto-generated git metadata
├── subsystems/
│   ├── drive/                    # Swerve drive, module IO, gyro IO, odometry thread
│   ├── flywheel/                 # 4x TalonFX shooter, ShotCalculator, visualizer
│   ├── hood/                     # TalonFX + CANcoder pivot, HoodPosCalculator
│   ├── intakePivot/              # TalonFX + CANcoder pivot, IntakePivotVisualizer
│   ├── intakeRoller/             # 2x TalonFX rollers (voltage-only)
│   ├── prestage/                 # 2x TalonFX staging wheels
│   ├── transport/                # 1x TalonFX belt (voltage-only)
│   ├── upperFeeder/              # 1x TalonFX upper feed
│   ├── lowerFeeder/              # 1x TalonFX lower feed
│   └── vision/                   # 4x PhotonVision cameras, AprilTag fusion
├── commands/
│   ├── DriveCommands.java        # Joystick drive, heading-snap alignment
│   ├── FlywheelCommands.java     # RPM setpoints, idle, distance-based
│   ├── HoodCommands.java         # Position tracking, pass/hub
│   ├── IntakePivotCommands.java  # Deploy/retract/compress sequences
│   ├── intakeRollerCommands.java # Voltage control
│   ├── PrestageCommands.java     # Velocity/voltage/idle
│   ├── TransportCommands.java    # Voltage/velocity with wait sequences
│   ├── FeederCommands.java       # Complex spinup→align→feed sequences
│   ├── ShootSequences.java       # High-level composite shot commands
│   └── SpitSequences.java        # Reverse/spit sequences
└── util/
    ├── BatteryLogger.java        # Per-subsystem current tracking
    ├── HubShiftUtil.java         # Match-schedule hub activation timing
    ├── LoggedTrigger.java        # AdvantageKit-logged Trigger wrapper
    ├── AllianceFlipUtil.java     # Cached red/blue flipping
    ├── CANUpdateThread.java
    ├── LocalADStarAK.java        # Custom PathPlanner A* pathfinder
    ├── Elastic.java              # Dashboard tab switching
    └── PhoenixUtil.java          # Phoenix config retry utilities
generated/
    ├── TunerConstants.java       # Delegates to COMP or ALPHA
    ├── COMP_TunerConstants.java  # Competition robot swerve constants
    └── ALPHA_TunerConstants.java # Alpha/practice robot swerve constants
```

---

## Hardware & CAN Layout

### CAN Bus Topology

Two buses are in use:

**CANivore — `"Canivore"`** (low-latency, for swerve)

| Device | Type | CAN ID |
|---|---|---|
| Pigeon 2 IMU | Gyro | 0 |
| Back Left Drive | TalonFX | 1 |
| Back Left Steer | TalonFX | 2 |
| Back Left Encoder | CANcoder | 3 |
| Front Left Drive | TalonFX | 4 |
| Front Left Steer | TalonFX | 5 |
| Front Left Encoder | CANcoder | 6 |
| Front Right Drive | TalonFX | 7 |
| Front Right Steer | TalonFX | 8 |
| Front Right Encoder | CANcoder | 9 |
| Back Right Drive | TalonFX | 10 |
| Back Right Steer | TalonFX | 11 |
| Back Right Encoder | CANcoder | 12 |
| Intake Pivot Motor | TalonFX | 41 |
| Intake Pivot Encoder | CANcoder | 44 |

> Intake pivot is on CANivore despite not being a swerve device. Reason not documented.

**RIO CAN — `"rio"`**

| Device | Type | CAN ID |
|---|---|---|
| Flywheel Leader | TalonFX | 30 |
| Flywheel Follower 1 | TalonFX | 31 |
| Flywheel Follower 2 | TalonFX | 32 |
| Flywheel Follower 3 | TalonFX | 33 |
| Flywheel Follower 4 | TalonFX | 34 |
| Hood Motor | TalonFX | 35 |
| Upper Feeder | TalonFX | 36 |
| Prestage Right (Follower) | TalonFX | 37 |
| Prestage Left (Leader) | TalonFX | 38 |
| Lower Feeder | TalonFX | 39 |
| Transport | TalonFX | 40 |
| Intake Roller Leader | TalonFX | 42 |
| Intake Roller Follower | TalonFX | 43 |
| Hood CANcoder | CANcoder | 50 |

### Motor Control Modes

| Subsystem | Control Mode |
|---|---|
| Swerve steer | MotionMagicTorqueCurrentFOC |
| Swerve drive | VelocityTorqueCurrentFOC |
| Flywheel | MotionMagicVelocityTorqueCurrentFOC |
| Hood | MotionMagicTorqueCurrentFOC |
| Prestage | MotionMagicVelocityTorqueCurrentFOC |
| Upper/Lower Feeder | MotionMagicVelocityTorqueCurrentFOC |
| Transport | VoltageOut (open loop) |
| Intake Roller | VoltageOut (open loop) |

---

## Subsystem Architecture

All subsystems follow the **AdvantageKit IO pattern**:
- `XxxIO` interface defines `updateInputs(XxxIOInputs)` and all control methods
- `XxxIOInputsAutoLogged` is the annotation-generated loggable inputs record
- Real (`XxxIOPhoenix6` / `XxxIOReal`) and sim (`XxxIOSim`) implementations are swapped at startup in `RobotContainer`
- Every `periodic()` calls `io.updateInputs(inputs)` → `Logger.processInputs()`

### Drive
- 4 SDS MK5n modules, each behind a `ModuleIO` abstraction
- `PhoenixOdometryThread` runs at 100–250 Hz, locked with `ReentrantLock`
- Pose fusion via `SwerveDrivePoseEstimator.addVisionMeasurement()`
- Gyro fallback: integrates kinematics twist if Pigeon2 disconnects
- `AutoBuilder.configure()` called here with `PPHolonomicDriveController` (Kp=5.0)

### Vision
- 4 cameras: `RobotRight`, `RobotLeft`, `ShooterRight`, `ShooterLeft`
- Strategy: multi-tag coprocessor PnP first, fallback to lowest-ambiguity single-tag
- Rejection filters (applied in order):
  1. Angular velocity > 4.0 rad/s
  2. No tags, or single-tag ambiguity > 0.4
  3. Robot Z outside [−0.15 m, 2.0 m]
  4. Tag distance > 6.0 m
  5. Pitch or roll > 25°
  6. Pose outside field boundary
- Std devs scale as `dist² / tagCount`; MegaTag2 gets 0.5× linear, ∞ angular

### Flywheel
- 1 leader + 3 followers (IDs 31–33 oppose leader, ID 34 same direction)
- `ShotCalculator` singleton: `InterpolatingDoubleTreeMap` maps distance (in) → RPM
- `HoodPosCalculator` computes hood angle from the same distance
- `isSpunUp()` threshold: within 200 RPM of target

### Hood
- TalonFX + remote CANcoder (FusedCANcoder feedback mode)
- Default command stows to 0°

### Intake Pivot
- TalonFX + remote CANcoder
- `IntakePivotVisualizer` provides mechanism2d animation
- Named positions: `pivotDownPos`, `pivotUpPos`, `pivotJostleUpPos`, `pivotJostleFirstPos`
- `ContinuousConditionalCommand` re-evaluates single/double compress mode each tick

### Transport, Feeders, Rollers, Prestage
- Simple voltage or velocity wrappers
- Transport and rollers are open-loop with no jam detection

---

## Command Patterns

All command files are **static factory classes** — no instances, only static methods returning `Command` objects.

### Shoot Sequence (`ShootSequences.java`)

Three-phase composition:
1. **Parallel (forever):** spin flywheel + prestage + set hood
2. **Gate:** `waitUntil(flywheel::isSpunUp).withTimeout(...)`
3. **After gate:** start feeders + transport + agitate

Shot readiness is implicit — no explicit state machine.

### Feeder Wait Pattern (`FeederCommands.java`)

```
waitUntil(spinup).withTimeout(0.5 s)
  → waitUntil(isAligned OR timeout).withTimeout(1.0 s)
  → run feeder
```

Total 1.5 s cap: if alignment never comes, the robot fires anyway. This is intentional.

### `ContinuousConditionalCommand`

Custom utility that re-evaluates its `BooleanSupplier` selector every tick. Used in `compressPivot` to switch between single/double compress mid-execution. WPILib's `ConditionalCommand` only evaluates at schedule time.

### Named Commands vs. Event Triggers

- **Named Commands** — full commands used inside auto paths
- **Event Triggers** — zone-based markers; registered with `Commands.runOnce()` and *no subsystem requirements* to avoid interrupting the running auto path

---

## RobotState — Central Singleton

Single source of truth for all geometry and alignment:
- Current pose (supplier callback from Drive)
- Distance and bearing to alliance hub / pass targets
- `isAlignedToHub`, `isAlignedToPass`, loose variants
- Zone classification (broad, specific, approaching)

No subsystem references another subsystem directly — all shared state goes through `RobotState`.

### Zone System

`HardwareConstants.Zones` defines:
- **Broad** (X axis): `ALLIANCE_ZONE`, `ALLIANCE_TRENCH`, `NEUTRAL`, `OPPOSING_TRENCH`, `OPPOSING_ZONE`
- **Specific** (X + Y): tower, trench near/far, bump near/far, hub
- **Approaching**: margins ahead of boundaries for predictive alignment

All checks are alliance-aware via `AllianceFlipUtil.shouldFlip()`, cached once per loop.

### HubShiftUtil

Encodes match-schedule timing for when the hub is active. `isShootSafeTime` is true when hub is powered, when disabled, or in demo mode.

---

## Triggers & Button Bindings

`Triggers.java` owns all `Trigger` / `LoggedTrigger` objects. `RobotContainer` reads from it.

### Primary Driver — Thrustmaster Joystick

| Button | Action |
|---|---|
| 1 | Shoot to hub |
| 2 | Trench alignment |
| 3 | Intake retract |
| 4 | Intake deploy |
| 5 | Intake roller |
| 6 | Intake compress |
| 7 | Demo distance shot |
| 8 | Bump alignment |
| 10 | Tower shot |
| 11 | Pass |
| 12 | X-wheel override |

### Key Composite Triggers

- `isShootClear` = `isShootSafeZone` AND `isShootSafeTime`
- `isAlignedForCurrentShot` = hub (in zone) OR pass (outside zone), debounced 0.3 s
- `isAlignedLooser` = same logic, 7° tolerance, no debounce

### Cancellation Flags

| Flag | Set when | Cleared when |
|---|---|---|
| `compressCancelled` | Driver touches intake while shooting | Shoot button released |
| `xCancelled` | Driver manually overrides X-wheels | Shoot button released |
| `doubleCompress` | Operator B button toggle | Same |

---

## PathPlanner Integration

- `AutoBuilder.configure()` in `Drive.java`
- `LocalADStarAK` custom A* registered via `Pathfinding.setPathfinder()`
- Auto chooser: `LoggedDashboardChooser` backed by `AutoBuilder.buildAutoChooser()`
- Auto delay: SmartDashboard input applied at `autonomousInit()`
- Start pose validation in `disabledPeriodic()`: ± 6 in translation, ± 5° rotation

**Named Commands:** `DeployIntake`, `RetractIntake`, `RunIntake`, `Shoot`, `stopAll`, `HoodDownNamed`

---

## AdvantageKit Logging

- `Robot.java` extends `LoggedRobot`
- Real: `WPILOGWriter("/U/logs")` + `NT4Publisher`
- Sim: `NT4Publisher` only
- Replay: `WPILOGReader` + `WPILOGWriter`

`@AutoLogOutput` on `RobotState` auto-records pose, velocities, distances, and alignment booleans each loop. `BatteryLogger` tracks per-subsystem current draw.

---

## Vendor Library Versions

| Library | Version |
|---|---|
| WPILib (GradleRIO) | 2026.2.1 |
| CTRE Phoenix 6 | 26.2.0 |
| PhotonVision | v2026.3.4 |
| AdvantageKit | 26.0.2 |
| PathPlannerLib | (vendordep) |

---

## Strong Patterns — Keep These

1. AdvantageKit IO layer — real/sim/replay fidelity; don't break this
2. `RobotState` singleton — all geometry lives here
3. Static command factories — composable, no hidden state
4. Feeder spinup→align→feed with timeout — battle-tested at Worlds
5. Zone-based shot routing — hub vs. pass vs. tower by position + match state
6. `AllianceFlipUtil` caching — avoids per-loop `Optional` allocations
7. Vision multi-filter stack — layered rejection prevents bad poses
8. Per-subsystem current logging — caught brownout issues in-season
9. Auto start pose check — prevented wrong-position autons
10. `ContinuousConditionalCommand` — correct for mid-execution mode switching

---

## Technical Debt & Risk Areas

### Disabled Code
- `Vision.java:140–141` — `maxPoseJumpMeters` filter commented out (was causing false rejections on good single-tag solves; re-enable and tune if needed)
- `DriveCommands.java:147–171` — `driveLucasProof()` disabled, never removed
- `FlywheelConstants.java:51–60` — NT-tunable gain scaffolding abandoned for static constants

### Magic Numbers
- Drive deadband `0.1` inline in `DriveCommands.java`
- Feeder total timeout `1.5 s` not tied to any named constant
- Flywheel spinup threshold `200 RPM`, prestage `500 RPM` — not in constants

### Structural Issues
- Intake pivot (IDs 41, 44) is on CANivore with no explanation — if someone plugs it into RIO CAN by accident, it silently fails
- Switching COMP ↔ ALPHA requires editing `Constants.java` — no deploy-time guard
- `intakeRoller` class uses lowercase-i, inconsistent with every other subsystem
- Transport and intake roller are open-loop with no jam detection
- `HubShiftUtil` encodes game-specific match timing — update when rules change
- `RobotContainer.java` is very long; all wiring and all bindings in one file
- Pass targets are four hardcoded field coordinates — no dynamic selection
