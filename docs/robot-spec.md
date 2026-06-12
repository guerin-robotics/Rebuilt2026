# Guerin Robotics — Rebuilt2026 Complete Specification

> This document is written at the level of detail required to reconstruct the codebase from
> scratch. Every constant, IO field, command behavior, and architectural decision is captured.

---

## 1. Game Context

The 2026 FRC game **"Rebuilt"** is a re-run of a hub-shooting game. Each alliance scores by
launching fuel (balls) into a central hub structure on the field.

### Hub Shift Mechanic

The hub alternates between being active for one alliance and the other on a fixed 140-second
teleop schedule. Scoring only counts during your alliance's active window. The FMS game-specific
message (first character `'R'` or `'B'`) determines which alliance starts active — the character
is inverted in code (`'R'` → Blue starts active, `'B'` → Red starts active).

**Teleop schedule windows:**

| Enum | Time into teleop | Length |
|---|---|---|
| TRANSITION | 0 – 10 s | 10 s |
| SHIFT1 | 10 – 35 s | 25 s |
| SHIFT2 | 35 – 60 s | 25 s |
| SHIFT3 | 60 – 85 s | 25 s |
| SHIFT4 | 85 – 110 s | 25 s |
| ENDGAME | 110 – 140 s | 30 s |

**activeSchedule** (for alliance that starts active): `{true, true, false, true, false, true}`
**inactiveSchedule** (for alliance that starts inactive): `{true, false, true, false, true, true}`

Fudge factors shift window boundaries for ball flight time and sensor delay:
- `minTimeOfFlight` = 0.75 s, `minFuelCountDelay` = 1.0 s → `approachingActiveFudge` = −1.75 s
- `maxTimeOfFlight` = 1.0 s, `maxFuelCountDelay` = 2.0 s, `shiftEndFuelCountExtension` = 3.0 s
  → `endingActiveFudge` = 3.0 − (1.0 + 2.0) = 0.0 s

Adjacent windows with the same active/inactive value are combined into a single elapsed/remaining
time for display and gating purposes.

---

## 2. Software Stack

| Library | Version |
|---|---|
| WPILib | 2026.2.1 |
| CTRE Phoenix 6 | 26.2.0 |
| PhotonVision | v2026.3.4 |
| AdvantageKit | 26.0.2 |
| PathPlannerLib | current |
| Build tool | Gradle, Google Java Format (Spotless) |
| Language | Java |
| Log storage | USB drive at `/U/logs` (WPILOG format) |

---

## 3. Package Structure

```
frc.robot
├── Robot.java                    — LoggedRobot; lifecycle, mode management, periodic logging
├── RobotContainer.java           — Wiring: subsystem construction, button bindings, named commands
├── RobotState.java               — Singleton: pose, velocity, zone, alignment booleans
├── Triggers.java                 — Singleton: all Trigger/LoggedTrigger objects
├── Constants.java                — Mode enum (REAL/SIM/REPLAY), RobotType enum (COMP/ALPHA)
├── HardwareConstants.java        — All CAN IDs, voltages, velocities, positions, thresholds
├── BuildConstants.java           — Auto-generated at deploy: git SHA, branch, build date
│
├── commands/
│   ├── ShootSequences.java       — autoShootToHub, shootEndBehavior, stopAll
│   ├── SpitSequences.java        — spitAll, spitHopper, clearShooter, spitAfterShoot
│   ├── DriveCommands.java        — joystickDrive, joystickDriveAtAngle, alignOrXForShoot, trench/bump align
│   ├── FlywheelCommands.java     — setFlywheelVelocity, setVelocityForHub, flywheelIdle, stop
│   ├── HoodCommands.java         — setHoodPos, setHoodPosForHub, setPosForPassing, hoodIdle
│   ├── FeederCommands.java       — setUpperFeederVelocity, setLowerFeederVelocity, *AfterWait
│   ├── TransportCommands.java    — setTransportVoltage, setTransportVelocity, *AfterWait
│   ├── PrestageCommands.java     — setPrestageVelocity, stop, prestageIdle
│   ├── IntakePivotCommands.java  — setPivotPosition, compressPivot, autoPivotCompress
│   └── intakeRollerCommands.java — setRollerVoltage, stopIntakeRoller, setVoltageAfterWait
│
├── subsystems/
│   ├── drive/          — Drive, Module, GyroIO/Pigeon2, ModuleIO/TalonFX/Sim, PhoenixOdometryThread
│   ├── flywheel/       — Flywheel, ShotCalculator, FlywheelVisualizer
│   │   └── io/         — FlywheelIO, FlywheelIOPhoenix6, FlywheelIOSim
│   ├── hood/           — Hood, HoodPosCalculator, HoodVisualizer
│   │   └── io/         — HoodIO, HoodIOReal, HoodIOSim
│   ├── intakePivot/    — IntakePivot, IntakePivotVisualizer
│   │   └── io/         — IntakePivotIO, IntakePivotIOReal, IntakePivotIOSim
│   ├── intakeRoller/   — intakeRoller
│   │   └── io/         — intakeRollerIO, intakeRollerIOReal, intakeRollerIOSim
│   ├── prestage/       — Prestage
│   │   └── io/         — PrestageIO, PrestageIOReal, PrestageIOSim
│   ├── upperFeeder/    — UpperFeeder
│   │   └── io/         — UpperFeederIO, UpperFeederIOReal, UpperFeederIOSim
│   ├── lowerFeeder/    — LowerFeeder
│   │   └── io/         — LowerFeederIO, LowerFeederIOReal, LowerFeederIOSim
│   ├── transport/      — Transport
│   │   └── io/         — TransportIO, TransportIOReal, TransportIOSim
│   └── vision/         — Vision
│       └── io/         — VisionIO, VisionIOPhotonVision, VisionIOPhotonVisionSim
│
├── generated/
│   ├── TunerConstants.java       — Active swerve config (delegates to COMP or ALPHA)
│   ├── COMP_TunerConstants.java  — Competition robot swerve calibration
│   └── ALPHA_TunerConstants.java — Alpha robot swerve calibration
│
└── util/
    ├── BatteryLogger.java        — Per-subsystem current/power/energy tracking to AdvantageKit
    ├── HubShiftUtil.java         — Hub shift schedule logic and timing
    ├── LoggedTrigger.java        — Trigger subclass that logs state changes to AdvantageKit
    ├── PhoenixUtil.java          — tryUntilOk() helper for CTRE device configuration
    ├── Elastic.java              — Elastic dashboard tab switching
    ├── LocalADStarAK.java        — AdvantageKit-compatible PathPlanner ADStar pathfinder
    ├── HubShiftUtil.java         — (see above)
    └── CANUpdateThread.java      — (internal utility)

frc.lib
├── AllianceFlipUtil.java         — Cached alliance color flip for coordinates
├── FieldConstants.java           — Field geometry: hub, tower, trench, bump, wall positions
└── ContinuousConditionalCommand.java — ConditionalCommand that re-evaluates condition every loop
```

---

## 4. Architecture Rules

### AdvantageKit IO Layer

Every subsystem follows this pattern exactly:

```java
class Flywheel extends SubsystemBase {
    private final FlywheelIO io;
    private final ShooterIOInputsAutoLogged inputs;

    public Flywheel(FlywheelIO io) {
        this.io = io;
        this.inputs = new ShooterIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);  // MUST be present in every periodic()
    }
}
```

- `XxxIO` is a Java interface with `@AutoLog` inputs class and `default` method stubs
- `XxxIOReal` implements hardware using CTRE Phoenix 6
- `XxxIOSim` implements physics simulation using WPILib DCMotorSim
- `XxxIOInputsAutoLogged` is code-generated by the AdvantageKit annotation processor
- **Never** put hardware calls (TalonFX, CANcoder) inside the subsystem class
- **Never** remove `Logger.processInputs()` from `periodic()` — it breaks log replay

### Command Pattern

All commands are static factory methods. No `Command` subclasses exist for game logic:

```java
// CORRECT
public class FlywheelCommands {
    public static Command setFlywheelVelocity(Flywheel flywheel, AngularVelocity velocity) {
        return Commands.run(() -> flywheel.setFlywheelVelocity(velocity), flywheel)
            .withName("FlywheelVelocity_" + velocity.in(RotationsPerSecond) + "rps");
    }
}
```

All commands are named with `.withName()`. Format: `"SubsystemName_Action_OptionalParam"`.

### Singleton Pattern

`RobotState`, `Triggers`, `ShotCalculator`, `HoodPosCalculator`, `HubShiftUtil` are all accessed
via `getInstance()`. `RobotState` and `Triggers` are created on first call; `HubShiftUtil` is
static-only.

### No Cross-Subsystem References

No subsystem holds a reference to another subsystem. Shared state passes through `RobotState`.
The only exception: `RobotContainer` holds references to all subsystems for wiring.

### Pose Estimator

One `SwerveDrivePoseEstimator` lives in `Drive.java`. `RobotState` holds only a
`Supplier<Pose2d>` set during `Drive`'s constructor via
`RobotState.getInstance().setPoseSupplier(this::getPose)`.

---

## 5. Robot.java — Lifecycle

`Robot extends LoggedRobot`.

**Constructor:**
- Records build metadata (git SHA, branch, date, dirty flag)
- Switches on `Constants.currentMode`: REAL → WPILOG writer + NT4 publisher; SIM → NT4 only;
  REPLAY → log reader + WPILOG writer (no timing)
- Calls `Logger.start()`
- Manually registers `RobotState.getInstance()` with `AutoLogOutputManager.addObject()` (singleton
  not discoverable via object graph scan)
- Constructs `RobotContainer`
- Publishes `Field2d` ("Robot Pose Field Map") and pre-flight checklist to SmartDashboard

**`robotPeriodic()`:**
1. Runs `CommandScheduler`
2. Updates `BatteryLogger` voltage and RIO current; calls `batteryLogger.periodicAfterScheduler()`
3. Calls `AllianceFlipUtil.refresh()` — caches alliance color once per loop
4. Updates `fieldMap` robot pose
5. Mode flag management: if `atComp=true` and FMS attached → force `TUNING_MODE=false`,
   `DEMO_MODE=false`; else read from `isTuning`/`demo` fields

**`disabledPeriodic()`:** calls `robotContainer.updateAutoPreview()` and
`robotContainer.checkStartPose()`

**`autonomousInit()`:** schedules `getAutonomousCommand()` (with configurable delay from
SmartDashboard key `"Auto Delay"`)

**`autonomousPeriodic()`:** updates auto preview field with current pose; publishes match time

**`teleopInit()`:**
1. Cancels autonomous command
2. Schedules `getAutoStopCommand()` (stops all shooter mechanisms)
3. `HubShiftUtil.initialize()` — starts hub shift timer
4. Schedules `getIntakeRollerCommand()` (runs rollers at intake voltage)
5. Schedules `getIntakePivotCommand()` (deploys intake to down position)
6. `Elastic.selectTab("Teleoperated")`

**`teleopPeriodic()`:** publishes to SmartDashboard: time left in shift (rounded to 0.1 s),
win auto?, hub active, match time, aligned, spun up, wheels xed. Logs hub shift state and
first active alliance.

**`testInit()`:** cancels all commands.

---

## 6. Constants.java

```java
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static final RobotType robotType = RobotType.COMP;
    public static boolean disableHAL = false;

    public enum Mode { REAL, SIM, REPLAY }
    public enum RobotType { COMP, ALPHA, NONE }
}
```

---

## 7. HardwareConstants.java — Complete Values

### CAN IDs (`HardwareConstants.CanIds`)

| Field | Value | Device |
|---|---|---|
| `MAIN_FLYWHEEL_LEADER_ID` | 30 | Flywheel leader |
| `MAIN_FLYWHEEL_FOLLOWER1_ID` | 31 | Flywheel follower 1 |
| `MAIN_FLYWHEEL_FOLLOWER2_ID` | 32 | Flywheel follower 2 |
| `MAIN_FLYWHEEL_FOLLOWER3_ID` | 33 | Flywheel follower 3 |
| `MAIN_FLYWHEEL_FOLLOWER4_ID` | 34 | Flywheel follower 4 |
| `HOOD_MOTOR` | 35 | Hood motor |
| `UPPER_FEEDER_MOTOR_ID` | 36 | Upper feeder |
| `PRESTAGE_FOLLOWER_ID` | 37 | Prestage follower (opposes leader) |
| `PRESTAGE_LEADER_ID` | 38 | Prestage leader |
| `LOWER_FEEDER_MOTOR_ID` | 39 | Lower feeder |
| `TRANSPORT_MOTOR_ID` | 40 | Transport |
| `INTAKE_PIVOT_MOTOR_ID` | 41 | Intake pivot motor (CANivore) |
| `INTAKE_ROLLER_LEADER_ID` | 42 | Intake roller leader |
| `INTAKE_ROLLER_FOLLOWER_ID` | 43 | Intake roller follower |
| `INTAKE_PIVOT_ENCODER_ID` | 44 | Intake pivot CANcoder (CANivore) |
| `HOOD_ENCODER` | 50 | Hood CANcoder |

### CompConstants.Voltages

| Field | Value |
|---|---|
| `transportVoltage` | −7.0 V |
| `intakeRollerVoltage` | +12.0 V |
| `intakeRollerAgitateVoltage` | +3.0 V |
| `prestageIdleVoltage` | −1.0 V |
| `prestageVoltage` | +8.0 V |

### CompConstants.SpitVoltages

| Field | Value |
|---|---|
| `transportSpitVoltage` | +12.0 V |
| `intakeRollerSpitVoltage` | +10.0 V |

### CompConstants.Velocities

| Field | Value |
|---|---|
| `prestageVelocity` | 3,000 RPM |
| `feederVelocity` | −3,000 RPM |
| `prestageIdleVelocity` | 1,300 RPM |
| `flywheelIdleVelocity` | 1,200 RPM |
| `prestageIdleVelocityHigh` | 1,200 RPM |
| `flywheelIdleVelocityHigh` | 60 RPM |
| `intakeRollerVelocity` | 2,400 RPM |
| `transportVelocity` | −1,800 RPM |

### CompConstants.SpitVelocities

| Field | Value |
|---|---|
| `prestageSpitVelocity` | 50 rot/s |
| `feederSpitVelocity` | 50 rot/s |
| `flywheelSpitVelocity` | 17 rot/s |

### CompConstants.Positions

| Field | Value |
|---|---|
| `hoodDownPos` | 0° |
| `pivotUpPos` | 0.3 rot |
| `pivotDownPos` | 0.0 rot |
| `pivotJostleUpPos` | 0.25 rot |
| `pivotJostleFirstPos` | 0.115 rot |
| `pivotJostleSecondPos` | 0.16 rot |

### CompConstants.Waits

| Field | Value | Purpose |
|---|---|---|
| `flywheelSpinupSeconds` | 0.5 s | Wait before checking alignment in feeder/transport sequences |
| `passSpinUpSeconds` | 0.75 s | Pass shot spinup |
| `alignmentTimeoutSeconds` | 1.5 s | Max time to wait for heading alignment before firing anyway |
| `spinUpTimeOut` | 0.5 s | Max time waiting for flywheel to reach target RPM |
| `waitToCompressSeconds` | 0.50 s | Teleop compress: wait before jostling |
| `waitToDropSeconds` | 0.5 s | Double compress: wait between positions |
| `waitBetweenCompressSeconds` | 0.15 s | Double compress: pause between moves |
| `autoWaitToCompressSeconds` | 0.85 s | Auto compress: longer wait for stable position |

### CompConstants.Thresholds

| Field | Value | Purpose |
|---|---|---|
| `flywheelSpinupThreshold` | 200 RPM | `isSpunUp()` = within 200 RPM of target |
| `prestageSpinupThreshold` | 500 RPM | (unused in current active code) |
| `hubAlignmentToleranceDegrees` | 1.5° | Tight: triggers `isAlignedToHub()` |
| `hubLooseAlignmentToleranceDegrees` | 7.0° | Loose: triggers feeders in real teleop path |
| `passAlignmentToleranceDegrees` | 7.0° | Triggers `isAlignedToPass()` |
| `passLooseAlignmentToleranceDegrees` | 7.0° | Triggers `isAlignedToPassLoose()` |

### CompConstants.Autos

| Field | Value |
|---|---|
| `DefaultAutoName` | `"2.5-Left-Comp"` |

### TuningConstants

| Field | Default | Meaning |
|---|---|---|
| `isTuning` | `false` | Static: whether tuning mode is desired |
| `demo` | `true` | Static: whether demo mode is desired |
| `TUNING_MODE` | dynamic | Computed from `isTuning` / FMS presence |
| `DEMO_MODE` | dynamic | Computed from `demo` / FMS presence |
| `atComp` | `false` | If true, forces modes off when FMS attached |
| `FlywheelTuningVelocity` | 2,000 RPM | Fixed velocity in tuning mode |
| `HoodTuningPos` | 12.25° | Fixed hood angle in tuning mode |
| `HoodDemoPos` | 45° | Hood angle in demo mode |

### TowerConstants

| Field | Value |
|---|---|
| `FlywheelTowerVelocity` | 1,625 RPM |
| `hoodTowerPos` | 2.5° |

### PassConstants

| Field | Value |
|---|---|
| `FlywheelPassVelocity` | 2,700 RPM |
| `hoodPassPos` | 35.0° |

### ControllerConstants

| Field | Value |
|---|---|
| `ButtonPanelPort` | 0 |
| `XboxControllerPort` | 1 |
| `JoystickControllerPort` | 2 |
| `DEADBAND` | 0.08 |
| `SimControllerPort` | 5 |

---

## 8. CAN Bus Layout

### CANivore Bus (`"Canivore"`)

| CAN ID | Device | Inversion | Offset |
|---|---|---|---|
| 0 | Pigeon 2 (gyro) | — | — |
| 1 | Back Left Drive (Kraken X60) | No | — |
| 2 | Back Left Steer (Kraken X60) | No | — |
| 3 | Back Left CANcoder | — | −0.373046875 rot |
| 4 | Front Left Drive (Kraken X60) | No | — |
| 5 | Front Left Steer (Kraken X60) | No | — |
| 6 | Front Left CANcoder | — | +0.139404296875 rot |
| 7 | Front Right Drive (Kraken X60) | Yes | — |
| 8 | Front Right Steer (Kraken X60) | No | — |
| 9 | Front Right CANcoder | — | −0.123779296875 rot |
| 10 | Back Right Drive (Kraken X60) | Yes | — |
| 11 | Back Right Steer (Kraken X60) | No | — |
| 12 | Back Right CANcoder | — | −0.212646484375 rot |
| 41 | Intake Pivot Motor (TalonFX) | No | — |
| 44 | Intake Pivot CANcoder | — | −0.58 rot |

### RIO CAN Bus (`"rio"`)

| CAN ID | Device | Notes |
|---|---|---|
| 30 | Flywheel Leader (Kraken X60) | |
| 31 | Flywheel Follower 1 (Kraken X60) | Opposes leader |
| 32 | Flywheel Follower 2 (Kraken X60) | Opposes leader |
| 33 | Flywheel Follower 3 (Kraken X60) | Opposes leader |
| 34 | Flywheel Follower 4 (Kraken X60) | Same direction as leader |
| 35 | Hood Motor (TalonFX) | |
| 36 | Upper Feeder (Kraken X60) | |
| 37 | Prestage Follower (Kraken X60) | Opposes leader (ID 38) |
| 38 | Prestage Leader (Kraken X60) | |
| 39 | Lower Feeder (Kraken X60) | |
| 40 | Transport (TalonFX) | Open-loop only |
| 42 | Intake Roller Leader (TalonFX) | Open-loop only |
| 43 | Intake Roller Follower (TalonFX) | Same direction as leader |
| 50 | Hood CANcoder | Absolute encoder for hood angle |

---

## 9. Drivetrain Subsystem

**Class:** `Drive extends SubsystemBase`

### Physical

| Property | Value |
|---|---|
| Wheel base | 22 in × 22 in (±11 in from center) |
| Drive gear ratio | 7.03125:1 |
| Steer gear ratio | 26.09:1 |
| Wheel radius | 2 in (0.0508 m) |
| Coupling ratio | 4.5 motor rot per steer rot |
| Max speed | ~4.39 m/s @ 12 V (`kSpeedAt12Volts`) |
| Odometry frequency | 250 Hz (CANivore FD network) |

### COMP_TunerConstants Gains

Steer Slot0: KP=3750, KI=0, KD=50, KS=0.1, KV=1.94, KA=0, TorqueCurrentFOC,
`StaticFeedforwardSign=UseClosedLoopSign`

Drive Slot0: KP=35, KI=0, KD=0, KS=3.18, KV=1.2, TorqueCurrentFOC

### PathPlanner Configuration

Configured in `Drive` constructor via `AutoBuilder.configure()`:
- Pose supplier: `this::getPose`
- Pose resetter: `this::setPose`
- Chassis speeds getter: `this::getChassisSpeeds`
- Chassis speeds consumer: `this::runVelocity`
- Translation PID: `PIDConstants(5.0, 0.0, 0.0)`
- Rotation PID: `PIDConstants(5.0, 0.0, 0.0)`
- PP_CONFIG: mass=63.503 kg, MOI=5.162 kg·m², COF=1.2
- Alliance flip: `DriverStation.getAlliance() == Alliance.Red`
- Pathfinder: `LocalADStarAK` (AdvantageKit-wrapped ADStar)

### Odometry

`PhoenixOdometryThread` runs at 250 Hz, acquires odometry lock, reads module positions and gyro
yaw, calls `poseEstimator.updateWithTime()` for each sample batch.

`Drive.periodic()` acquires the odometry lock, calls `module.periodic()` for each module, runs
the odometry update loop over all buffered samples, then calls
`RobotState.getInstance().updateModuleStates(states)`.

After construction: `RobotState.getInstance().setPoseSupplier(this::getPose)`.

### ModuleIO Inputs (`@AutoLog`)

```java
boolean driveConnected;
double drivePositionRad;
double driveVelocityRadPerSec;
double driveAppliedVolts;
double driveCurrentAmps;
boolean turnConnected;
boolean turnEncoderConnected;
Rotation2d turnAbsolutePosition;
Rotation2d turnPosition;
double turnVelocityRadPerSec;
double turnAppliedVolts;
double turnCurrentAmps;
double[] odometryTimestamps;
double[] odometryDrivePositionsRad;
Rotation2d[] odometryTurnPositions;
```

### GyroIO Inputs (`@AutoLog`)

```java
boolean connected;
Rotation2d yawPosition;
double yawVelocityRadPerSec;
double[] odometryYawTimestamps;
Rotation2d[] odometryYawPositions;
```

---

## 10. Flywheel Subsystem

**Class:** `Flywheel extends SubsystemBase`

**Constructor:** takes `FlywheelIO io`. Creates `ShooterIOInputsAutoLogged inputs`.
Creates `FlywheelVisualizer visualizer`. Creates `LoggedNetworkNumber tuningRPM` at
`"Tune/flywheel/tuningRPM"` default 20. `currentRPMTarget` tracks last commanded RPM.
`hoodAngleSupplier` defaults to `() -> Degrees.of(0)`; set via `setHoodAngleSupplier()`.

**`periodic()`:**
1. `io.updateInputs(inputs); Logger.processInputs("Flywheel", inputs)`
2. Reports per-motor supply current to `Robot.batteryLogger` for leader, followers 1–4
3. `visualizer.updateTrajectory(inputs.flywheelVelocity, hoodAngleSupplier.get())`
4. `Logger.recordOutput("Flywheel/targetRPM", currentRPMTarget)`

**Key methods:**
- `setFlywheelVelocity(AngularVelocity)` — stores `currentRPMTarget`, calls `io.setFlywheelVelocity`
- `setFlywheelIdle()` — calls `setFlywheelVelocity(flywheelIdleVelocity)` = 1,200 RPM
- `setSpeedForHub()` — calls `ShotCalculator.getInstance().getFlywheelSpeedForAllianceHub()`
- `setSpeedForPassing()` — calls `ShotCalculator.getInstance().getFlywheelSpeedForPassTarget()`
- `isSpunUp()` — `|currentRPMTarget - inputs.leaderVelocity.in(RPM)| < 200 RPM`
- `isFlywheelSpunUp` — `LoggedTrigger` wrapping `isSpunUp()`

### FlywheelIO Inputs (`@AutoLog` class `ShooterIOInputs`)

```java
AngularVelocity flywheelVelocity;           // combined
AngularVelocity closedLoopError;
AngularVelocity closedLoopReference;
AngularVelocity leaderVelocity;
Voltage leaderAppliedVolts;
Current leaderSupplyCurrentAmps;
Current leaderStatorCurrentAmps;
Temperature leaderTemp;
Angle leaderAngle;
// follower1..4: same fields (velocity, appliedVolts, supplyCurrentAmps, statorCurrentAmps, temp)
```

### FlywheelIO Methods

```java
default void updateInputs(ShooterIOInputs inputs) {}
default void setFlywheelVoltage(Voltage volts) {}
default void setFlywheelVelocity(AngularVelocity velocity) {}  // MotionMagicVelocityTorqueCurrentFOC
```

### FlywheelConstants

**TorqueControl (real robot):** KS=8.0, KV=0.12, KP=15.0, KD=0
**Sim:** KS=4.0, KV=0.005, KP=8.0, KI=0, KD=0
**`getKS/KV/KP/KD()`** switch on `Constants.currentMode`

**Limits:** MIN=100 RPM, MAX=5,600 RPM, MAX_ACCEL=160 rot/s²
**Gear ratio:** 36:24 sensor→mechanism
**Current limits:** supply=40 A (trigger 35 A/1 s), stator=45 A
**`flywheelMagicConstants.flywheelAccel`** = 100

**Distance→RPM map** (`SPEED_MAP`, key = meters):

| Distance (in) | RPM |
|---|---|
| 75 | 1,450 |
| 85 | 1,525 |
| 110 | 1,625 |
| 130 | 1,700 |
| 145 | 1,750 |
| 160 | 1,875 |
| 175 | 1,925 |
| 180 | 1,950 |
| 190 | 1,975 |

**Passing speed map** (`PASSING_SPEED_MAP`):

| Distance (in) | RPM |
|---|---|
| 70 | 1,600 |
| 140 | 1,700 |
| 225 | 2,050 |
| 410 | 2,700 |

### ShotCalculator

Singleton. `getFlywheelSpeedForAllianceHub()` → calls `RobotState.getAllianceHubTarget()` →
2D distance → `SPEED_MAP.get()` → clamp to [MIN, MAX] → return RPM.

`getFlywheelSpeedForPassTarget()` → `RobotState.getPassTarget()` → 2D distance →
`PASSING_SPEED_MAP.get()` → clamp.

---

## 11. Hood Subsystem

**Class:** `Hood extends SubsystemBase`

**`periodic()`:** `io.updateInputs(inputs); Logger.processInputs("Hood", inputs)`.
Reports supply current to `Robot.batteryLogger`.

**Key methods:**
- `setHoodPos(Angle)` — calls `io.setHoodPos(position)`
- `setHoodPosForHub()` — calls `HoodPosCalculator.getInstance().getHoodPosForHub()` then `io.setHoodPos`
- `setHoodPosForPass()` — calls `HoodPosCalculator.getInstance().getHoodPosForPassing()`
- `stowHood()` — `io.setHoodPos(Degrees.of(0))`
- `getPosition()` — returns `inputs.hoodPosition`
- `incrementHoodPos()` — adds 5° to current position

### HoodIO Inputs (`@AutoLog`)

```java
Voltage hoodVoltage;
Current hoodSupplyCurrent;
Current hoodStatorCurrent;
Temperature hoodTemperature;
AngularVelocity hoodVelocity;
Angle hoodPosition;
Angle hoodClosedLoopReference;
Angle hoodClosedLoopError;
```

### HoodIO Methods

```java
default void updateInputs(HoodIOInputs inputs) {}
default void setHoodPos(Angle position) {}  // MotionMagicTorqueCurrentFOC position
```

### HoodPosCalculator

Singleton. `getHoodPosForHub()` → `RobotState.getAllianceHubTarget()` → 2D distance →
`ANGLE_MAP.get()` → clamp to [0°, 234°] → return degrees.

`getHoodPosForPassing()` → `RobotState.getPassTarget()` → 2D distance →
`PASSING_ANGLE_MAP.get()` → clamp.

### HoodConstants

**Mechanical:** shaft→hood ratio = 122/12, motor→shaft belt ratio = 5.33,
magnet offset = −0.16, discontinuity point = 0.625, maxPos = 234°, minPos = 0°

**SoftwareConstants:** softwareLowerLimit=0°, softwareUpperLimit=62°,
`ENCODER_DIRECTION=Clockwise_Positive`, `MOTOR_INVERTED=false`

**PID:** KS=9, KP=4000, KD=0
**HoodMagicConstants:** accel=10.0, velo=1
**CurrentLimits:** supply=40 A (trigger 35 A/1 s), stator=20 A

**Distance→angle map** (`ANGLE_MAP`):

| Distance (in) | Degrees |
|---|---|
| 75 | 1.0 |
| 85 | 1.5 |
| 110 | 2.5 |
| 130 | 3.5 |
| 145 | 5.25 |
| 160 | 11.0 |
| 175 | 11.5 |
| 180 | 12.25 |

**Passing angle map** (`PASSING_ANGLE_MAP`):

| Distance (in) | Degrees |
|---|---|
| 70 | 20.0 |
| 140 | 25.0 |
| 225 | 28.0 |
| 410 | 35.0 |

---

## 12. Intake Pivot Subsystem

**Class:** `IntakePivot extends SubsystemBase`

**`periodic()`:** `io.updateInputs(inputs); Logger.processInputs("Intake Pivot", inputs)`.
Reports pivot supply current. Computes `atGoal = |current - goal| < 0.005 rot`.
Calls `visualizer.update(currentRot, goalRot, atGoal)`.

**Key methods:**
- `setPivotPosition(Angle)` — stores `goalPosition`, calls `io.setPivotPosition`
- `setPivotVoltage(Voltage)` — calls `io.setPivotVoltage`
- `getPosition()` — returns `inputs.intakePivotPosition`
- `zeroPivotEncoder()` — calls `io.zeroPivotEncoder()`

### IntakePivotIO Inputs (`@AutoLog`)

```java
Voltage intakePivotVoltage;
Current intakePivotSupplyCurrent;
Current intakePivotStatorCurrent;
Temperature intakePivotTemperature;
AngularVelocity intakePivotVelocity;
Angle intakePivotPosition;
double intakePivotClosedLoopReference;
double intakePivotClosedLoopError;
```

### IntakePivotIO Methods

```java
default void updateInputs(IntakePivotIOInputs inputs) {}
default void setPivotVoltage(Voltage volts) {}
default void setPivotVelocity(AngularVelocity velocity) {}
default void setPivotPosition(Angle position) {}    // MotionMagicTorqueCurrentFOC position
default void zeroPivotEncoder() {}
```

### IntakePivotConstants

**PID (real):** KG=8, KP=750, KI=0, KD=5
**PID (sim):** KS=0, KG=10, KV=0, KP=300, KI=0, KD=15
**`getKG/KP/KD()`** switch on `Constants.currentMode`

**Mechanical:** ratio=45, magnetOffset=−0.58 rot, discontinuityPoint=0.625,
jostleCurrentLimit=70 A
**SoftwareConstants:** softwareUpperLimit=0.4 rot, softwareLowerLimit=0.0 rot,
MOTOR_INVERTED=false, ENCODER_DIRECTION=Clockwise_Positive
**PivotMagicConstants:** accel=2.0, velo=1
**CurrentLimits:** supply=50 A (trigger 45 A/1 s), stator=80 A
**Visualization.POSITION_TOLERANCE_ROTATIONS** = 0.005

---

## 13. Intake Roller Subsystem

**Class:** `intakeRoller extends SubsystemBase`

Open-loop voltage control. Leader + follower (same direction).

### intakeRollerIO Inputs (`@AutoLog`)

```java
Voltage intakeRollerVoltage;
Current intakeRollerSupplyCurrent;
Current intakeRollerStatorCurrent;
Temperature intakeRollerTemperature;
AngularVelocity intakeRollerVelocity;
AngularVelocity rollerClosedLoopReference;
AngularVelocity rollerClosedLoopError;
Angle rollerPos;
Voltage intakeRollerFollowerVoltage;
Current intakeRollerFollowerSupplyCurrent;
Current intakeRollerFollowerStatorCurrent;
Temperature intakeRollerFollowerTemperature;
AngularVelocity intakeRollerFollowerVelocity;
```

### intakeRollerIO Methods

```java
default void updateInputs(intakeRollerIOInputs inputs) {}
default void setRollerVoltage(Voltage volts) {}
default void setRollerVelocity(AngularVelocity rollerVelo) {}
```

---

## 14. Prestage Subsystem

Two-motor staged roller (leader CAN 38, follower CAN 37 opposes).

### PrestageIO Inputs (`@AutoLog`)

```java
Voltage prestageLeftVoltage, prestageRightVoltage;
Current prestageLeftStatorAmps, prestageLeftSupplyAmps;
Current prestageRightStatorAmps, prestageRightSupplyAmps;
AngularVelocity prestageLeftVelocity, prestageRightVelocity;
Temperature prestageLeftTemperature, prestageRightTemperature;
AngularVelocity prestageLeftClosedLoopReference, prestageRightClosedLoopReference;
AngularVelocity prestageLeftClosedLoopError, prestageRightClosedLoopError;
Angle prestageLeftPos, prestageRightPos;
```

### PrestageIO Methods

```java
default void updateInputs(PrestageIOInputs inputs) {}
default void setPrestageVoltage(Voltage volts) {}
default void setPrestageVelocity(AngularVelocity prestageVelo) {}  // TorqueCurrentFOC
```

### PrestageConstants

**PID (real):** KS=8, KV=0, KP=8, KI=0, KD=0
**Sim per side:** KS=2, KV=0, KP=3, KI=0, KD=0
**prestageMagicConstants.prestageAccel** = 80
**Mechanical.prestageRatio** = 24/11
**CurrentLimits:** supply=40 A (trigger 35 A/1 s), stator=45 A

---

## 15. Upper Feeder Subsystem

### UpperFeederIO Inputs (`@AutoLog`)

```java
Voltage upperFeederVoltage;
Current upperFeederStatorAmps, upperFeederSupplyAmps;
AngularVelocity upperFeederMotorVelocity;
Temperature upperFeederMotorTemperature;
AngularVelocity upperFeederClosedLoopReference, upperFeederClosedLoopError;
Angle upperFeederPos;
```

### UpperFeederIO Methods

```java
default void updateInputs(UpperFeederIOInputs inputs) {}
default void setUpperFeederVoltage(Voltage volts) {}
default void setUpperFeederVelocity(AngularVelocity feederVelo) {}
```

### UpperFeederConstants

**PID (real):** KS=2, KV=0, KP=13, KI=0, KD=0
**Sim:** KS=3, KV=0, KP=0, KI=0, KD=0
**feederMagicConstants.upperFeederAccel** = 120
**Mechanical.upperFeederRatio** = 24/11
**CurrentLimits:** supply=40 A (trigger 35 A/1 s), stator=40 A

---

## 16. Lower Feeder Subsystem

### LowerFeederIO Inputs (`@AutoLog`)

```java
Voltage lowerFeederVoltage;
Current lowerFeederStatorAmps, lowerFeederSupplyAmps;
AngularVelocity lowerFeederMotorVelocity;
Temperature lowerFeederMotorTemperature;
AngularVelocity lowerFeederClosedLoopReference, lowerFeederClosedLoopError;
Angle lowerFeederPos;
```

### LowerFeederIO Methods

```java
default void updateInputs(LowerFeederIOInputs inputs) {}
default void setLowerFeederVoltage(Voltage volts) {}
default void setLowerFeederVelocity(AngularVelocity feederVelo) {}
```

### LowerFeederConstants

**PID (real):** KS=2, KV=0, KP=16, KI=0, KD=0
**feederMagicConstants.lowerFeederAccel** = 120
**Mechanical.lowerFeederRatio** = 24/11
**CurrentLimits:** supply=40 A (trigger 35 A/1 s), stator=35 A

---

## 17. Transport Subsystem

### TransportIO Inputs (`@AutoLog`)

```java
Voltage TransportVoltage;
Current TransportStatorAmps, TransportSupplyAmps;
AngularVelocity TransportMotorVelocity;
Temperature TransportMotorTemperature;
AngularVelocity transportClosedLoopReference, transportClosedLoopError;
Angle transportPos;
```

### TransportIO Methods

```java
default void updateInputs(TransportIOInputs inputs) {}
default void setTransportVoltage(Voltage volts) {}
default void setTransportVelocity(AngularVelocity transportVelo) {}
```

### TransportConstants

**PID (real):** KS=5, KV=0, KP=4.2, KI=0, KD=0
**transportMagicConstants.transportAccel** = 120
**Mechanical.transportRatio** = 33/11
**CurrentLimits:** supply=40 A (trigger 35 A/1 s), stator=40 A

---

## 18. Vision Subsystem

**Camera coprocessor:** Orange Pi running PhotonVision
**AprilTag layout:** `AprilTagFields.k2026RebuiltWelded`

### Camera Transforms (robot→camera, all pitch=−15°, roll=0°)

| Camera | X (in) | Y (in) | Z (in) | Yaw |
|---|---|---|---|---|
| RobotRight (cam0) | +1.0 | −12.171 | +6.438 | 270° |
| RobotLeft (cam1) | +1.0 | +12.421 | +6.438 | 90° |
| ShooterRight (cam2) | −12.572 | −6.125 | +12.509 | 180° |
| ShooterLeft (cam3) | −12.572 | +5.375 | +12.509 | 180° |

### VisionIO Inputs (`@AutoLog`)

```java
boolean connected;
TargetObservation latestTargetObservation;  // record(Rotation2d tx, Rotation2d ty)
PoseObservation[] poseObservations;
int[] tagIds;
```

**PoseObservation** record: `double timestamp, Pose3d pose, double ambiguity, int tagCount,
double averageTagDistance, PoseObservationType type`

**PoseObservationType** enum: `MEGATAG_1, MEGATAG_2, PHOTONVISION`

### Active Rejection Filters

| Filter | Value |
|---|---|
| Max single-tag ambiguity | 0.4 |
| Max tag distance | 6.0 m |
| Max angular velocity | 4.0 rad/s |
| Max pitch/roll | 25° (Math.toRadians(25.0)) |
| Max Z error | 2 m |
| Floor cutoff | 6 in below floor |

### Standard Deviations

| Parameter | Value |
|---|---|
| Linear baseline | 0.01 m |
| Angular baseline | 0.03 rad |
| Scaling | baseline × dist² / tagCount |
| Single-tag multiplier | ×2.0 |
| MegaTag2 linear factor | ×0.5 |
| MegaTag2 angular factor | ∞ (no rotation data) |
| Per-camera factors | [1.0, 1.0, 1.0, 1.0] |

Vision measurements are fed into `Drive.addVisionMeasurement()` which calls
`poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs)`.

---

## 19. RobotState.java — Complete API

Singleton. Holds `Supplier<Pose2d> poseSupplier` (default returns `Pose2d()`; set by
`Drive`). Holds `SwerveDriveKinematics kinematics` (from `Drive.getModuleTranslations()`).
Holds `SwerveModuleState[] currentModuleStates`.

Registered with `AutoLogOutputManager.addObject()` in `Robot` constructor so `@AutoLogOutput`
annotations are picked up.

**Key methods:**

```
setPoseSupplier(Supplier<Pose2d>)   — called once from Drive constructor

@AutoLogOutput getEstimatedPose()   — returns poseSupplier.get()
getRotation()                       — returns pose.getRotation()

@AutoLogOutput getFieldRelativeVelocity()  — kinematics.toChassisSpeeds then fromRobotRelative
@AutoLogOutput getRobotRelativeVelocity()  — kinematics.toChassisSpeeds(currentModuleStates)

updateModuleStates(SwerveModuleState[])  — called from Drive.periodic()

@AutoLogOutput getDistanceToAllianceHub()  — 2D distance to getAllianceHubTarget().toTranslation2d()
getDistanceToPoint(Translation2d)          — getEstimatedPose().getTranslation().getDistance(point)
getAllianceHubTarget()                      — AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint)

@AutoLogOutput getAngleToAllianceHub()   — atan2 of (hub2d - robotPos), +180° (shooter faces rear)
getAngleToTarget(Translation2d target)   — atan2 of (target - robotPos), +180°

@AutoLogOutput isAlignedToHub()          — |getAngleToAllianceHub - pose.rotation| < 1.5°
isAlignedToHubLoose()                    — same, threshold 7.0°
@AutoLogOutput isAlignedToPass()         — |getAngleToPassTarget - pose.rotation| < 7.0°
isAlignedToPassLoose()                   — same, threshold 7.0°

getPassTarget()    — Translation3d; selects based on poseY relative to field center and alliance
                     Blue: poseY < fieldWidth/2 → (4.5, 2.3, 0); else → (4.5, 6.1, 0)
                     Red:  poseY > fieldWidth/2 → (12, 6.1, 0); else → (12, 2.3, 0)

getBroadZone()     — ALLIANCE_ZONE / ALLIANCE_TRENCH / NEUTRAL / OPPOSING_TRENCH / OPPOSING_ZONE
                     based on AllianceFlipUtil.applyX(poseX) vs FieldConstants.LinesVertical thresholds
getSpecificZone()  — detailed zone (tower, trench near/far, hub, bump near/far)
getApproachingZone() — composite approaching zone
```

---

## 20. Triggers.java — Complete Listing

Singleton. `CommandXboxController controller` (port 1), `CommandJoystick thrustmaster` (port 2),
`CommandXboxController simController` (port 5).

### Button Triggers (thrustmaster)

| Method | Button |
|---|---|
| `shootButton()` | 1 |
| `trenchAlignButton()` | 2 |
| `intakeInButton()` | 3 |
| `intakeOutButton()` | 4 |
| `intakeRollerButton()` | 5 |
| `intakeCompressButton()` | 6 |
| `demoDistanceShot()` | 7 |
| `bumpAlignButton()` | 8 |
| `shootFromTowerButton()` | 10 |
| `passButton()` | 11 |
| `autoXOverride()` | 12 |

### Button Triggers (Xbox controller)

| Method | Button |
|---|---|
| `allianceWinFlipper()` | A |
| `allianceWinDisabler()` | Y |
| `doubleCompressOverride()` | B |
| `xWheels()` | X |

### State Triggers (LoggedTrigger)

**`isShootSafeZone`** — `getBroadZone() == ALLIANCE_ZONE`

**`isShootSafeTime`** — `HubShiftUtil.getShiftedShiftInfo().active() || HubShiftUtil.disabled
|| DEMO_MODE`

**`isShootClear`** — `isShootSafeTime.and(isShootSafeZone)`

**`isAlignedForCurrentShot`** — if in ALLIANCE_ZONE: `isAlignedToHub()` (1.5°); else:
`isAlignedToPass()` (7.0°). Debounced 0.3 s rising edge.

**`isAlignedLooser`** — if in ALLIANCE_ZONE: `isAlignedToHubLoose()` (7.0°); else:
`isAlignedToPassLoose()` (7.0°).

---

## 21. Command Implementations

### DriveCommands

All alignment uses a `ProfiledPIDController` with:
- KP=7.5, KD=0.4, max velocity=8 rad/s, max acceleration=20 rad/s²
- Continuous input enabled (−π to π)

**`joystickDrive`:** Field-relative. Applies deadband 0.1, squares magnitude. Flips chassis speeds
if Red alliance. Sets `drive.areWheelsXed = false`.

**`joystickDriveAtAngle`:** Same linear control. Angular output = profiled PID error on rotation.

**`alignOrXForShoot`:** `ContinuousConditionalCommand(stopWithX, joystickDriveAtAngle, isAlignedForCurrentShot)`.
When aligned: X-wheels. When not aligned: drives at angle toward hub.

**`joystickDriveAlignForTrench`:** `joystickDriveAtAngle` with rotation locked to trench heading
(from FieldConstants).

**`joystickDriveAlignForBump`:** `joystickDriveAtAngle` with rotation locked to bump crossing
heading.

**`stopWithX`:** Locks all swerve modules in X pattern. Sets `drive.areWheelsXed = true`.

### FlywheelCommands

| Method | Implementation |
|---|---|
| `setFlywheelVelocity(vel)` | `Commands.run(() -> flywheel.setFlywheelVelocity(vel), flywheel)` |
| `setFlywheelVoltage(v)` | `Commands.startEnd(set, stop=0V, flywheel)` |
| `flywheelIdle()` | `Commands.run(() -> flywheel.setFlywheelIdle(), flywheel)` → 1,200 RPM |
| `setVelocityForHub()` | `Commands.run(() -> flywheel.setSpeedForHub(), flywheel)` — re-evaluates each loop |
| `setVelocityForPassing()` | `Commands.run(() -> flywheel.setSpeedForPassing(), flywheel)` |
| `stop()` | `Commands.runOnce(() -> flywheel.setFlywheelVelocity(0 rps), flywheel)` |

### HoodCommands

| Method | Implementation |
|---|---|
| `setHoodPos(pos)` | `Commands.startEnd(set pos, stow=0°, hood)` |
| `setHoodPosForHub()` | `Commands.runEnd(setHoodPosForHub, stow, hood)` — re-evaluates each loop |
| `setPosForPassing()` | `Commands.run(setHoodPosForPass, hood)` |
| `hoodIdle()` | `Commands.run(() -> hood.setHoodPos(hoodDownPos), hood)` — default command |
| `stowHood()` | `Commands.runOnce(stowHood, hood)` |

### IntakePivotCommands

| Method | Implementation |
|---|---|
| `setPivotPosition(pos)` | `Commands.runOnce(() -> intakePivot.setPivotPosition(pos), intakePivot)` — one-shot, not held |
| `setPivotVoltage(v)` | `Commands.startEnd(set, stop=0V, intakePivot)` |
| `compressPivot(doubleCompress)` | `ContinuousConditionalCommand(twoCompressBranch, oneCompressBranch, doubleCompress)` |
| `compressPivot()` | overload with `() -> false` → single compress |
| `manualPivotCompress()` | overload with `() -> true` → double compress |
| `autoPivotCompress()` | `sequence(wait(0.85s), setPivotPos(pivotJostleUpPos))` |

**Single compress** (`oneCompressBranch`):
```
sequence(
    wait(waitToCompressSeconds=0.50s),
    setPivotPosition(pivotJostleUpPos=0.25rot)
)
```

**Double compress** (`twoCompressBranch`):
```
sequence(
    setPivotPosition(pivotJostleFirstPos=0.115rot),
    wait(waitToDropSeconds=0.5s),
    setPivotPosition(pivotDownPos=0.0rot),
    wait(waitBetweenCompressSeconds=0.15s),
    setPivotPosition(pivotJostleUpPos=0.25rot)
)
```

### FeederCommands

**`setUpperFeederVelocity(feeder, vel)`:** `Commands.runOnce(() -> feeder.setUpperFeederVelocity(vel), feeder)`
**`setLowerFeederVelocity(feeder, vel)`:** same pattern
**`stopUpper/Lower(feeder)`:** `Commands.runOnce(() -> setVelocity(0 rps), feeder)`

**`setUpperVelocityAfterWait(feeder, vel)`** (no-align overload):
```
sequence(
    wait(flywheelSpinupSeconds=0.5s),
    wait(alignmentTimeoutSeconds - flywheelSpinupSeconds=1.0s),  // isAligned=()→true
    setUpperFeederVelocity(vel)
)
```

**`setUpperVelocityAfterWait(feeder, vel, isAligned)`** (align overload):
```
sequence(
    wait(0.5s),
    waitUntil(isAligned).withTimeout(1.0s),
    setUpperFeederVelocity(vel)
)
```

Lower feeder: identical structure.

### TransportCommands

**`setTransportVoltage(vol)`:** `Commands.startEnd(set, stop=0V, transport)`
**`setTransportVelocity(vel)`:** `Commands.runOnce(set, transport)`
**`setVelocityAfterWait(transport, vel)`:** same pattern as feeder
**`setVoltageAfterWait(transport, vol)`:**
```
sequence(wait(0.5s), setTransportVoltage(vol))
```

### PrestageCommands

**`setPrestageVelocity(vel)`:** `Commands.runOnce(set, prestage)`
**`stop()`:** `Commands.runOnce(() -> setVelocity(0 rps), prestage)`
**`prestageIdle()`:** `Commands.run(() -> setVelocity(prestageIdleVelocity=1300RPM), prestage)`

### intakeRollerCommands

**`setRollerVoltage(vol)`:** `Commands.startEnd(set, stop=0V, intakeRoller)`
**`stopIntakeRoller()`:** `Commands.runOnce(() -> setVelocity(0 rps), intakeRoller)`
**`setVoltageAfterWait(vol, isAligned)`:**
```
sequence(
    wait(0.5s),
    waitUntil(isAligned).withTimeout(1.0s),
    setRollerVoltage(vol)
)
```

### ShootSequences

**`autoShootToHub(...)`:**
```
parallel(
    runOnce(() -> Logger.recordOutput("RobotState/shooting", true)),
    parallel(
        FlywheelCommands.setVelocityForHub(flywheel),
        PrestageCommands.setPrestageVelocity(prestage, 3000RPM),
        HoodCommands.setHoodPosForHub(hood)
    ),
    sequence(
        waitUntil(flywheel.isFlywheelSpunUp).withTimeout(spinUpTimeOut=0.5s),
        parallel(
            FeederCommands.setLowerFeederVelocity(lowerFeeder, -3000RPM),
            FeederCommands.setUpperFeederVelocity(upperFeeder, -3000RPM),
            TransportCommands.setTransportVelocity(transport, -1800RPM),
            intakeRollerCommands.setRollerVoltage(intakeRoller, agitateVoltage=3V),
            IntakePivotCommands.autoPivotCompress(intakePivot)  // wait 0.85s then jostle
        )
    )
).withName("ShootToHub")
```

**`stopAll(...)`:** parallel stop of all mechanisms (flywheel, prestage, hood, upper/lower feeder,
transport, intakeRoller).

**`shootEndBehavior(...)`:**
```
sequence(
    parallel(stop prestage, feeder, transport, roller, pivot→down),
    wait(0.25s),
    FlywheelCommands.stop(flywheel)
)
```

### SpitSequences

**`spitAll(...)`:** parallel run all mechanisms at spit velocities/voltages; `finallyDo` stops all.

**`spitHopper(...)`:** parallel run feeder, transport, roller at spit velocities; `finallyDo` stops.

**`clearShooter(...)`:** parallel run flywheel, prestage, upper/lower feeder at spit velocities.

**`spitAfterShoot(...)`:** `WaitCommand(0.5s).deadlineFor(run all at spit)` then parallel stop all.

---

## 22. RobotContainer — Binding Logic

Three state flags reset on shoot button release:
- `compressCancelled` — set true by intakeIn/Out/Compress; prevents auto-compress
- `xCancelled` — set true by autoXOverride button
- `doubleCompress` — toggled by operator B button; reset on shoot release

### Default Commands

| Subsystem | Default Command |
|---|---|
| Drive | `joystickDrive` (thrustmaster axes 0=strafe, 1=forward, 2=twist, all negated, clamped ±1) |
| Flywheel | `FlywheelCommands.flywheelIdle()` → 1,200 RPM |
| Hood | `HoodCommands.hoodIdle()` → 0° position |

### Active Teleop Bindings (in priority / condition order)

**Drivetrain:**
- `shootButton && isShootSafeZone || shootFromTowerButton` → `alignOrXForShoot` at hub angle, 50% speed
- `shootButton && !isShootSafeZone || passButton && !DEMO_MODE` → `joystickDriveAtAngle` at pass target, 50%
- `trenchAlignButton` → `joystickDriveAlignForTrench`
- `bumpAlignButton` → `joystickDriveAlignForBump`

**Flywheel + Prestage:**
- `shootButton && isShootClear && !TUNING_MODE` → `setVelocityForHub + setPrestageVelocity(3000RPM)`; onFalse: stop both
- `(shootButton && !isShootSafeZone) || passButton && !TUNING_MODE` → `setVelocityForPassing + setPrestageVelocity`; onFalse: stop both
- `shootFromTowerButton` → flywheel at 1625 RPM + prestage + all feeders+transport after wait
- `shootButton && TUNING_MODE` → flywheel at 2000 RPM + prestage + feeders after wait
- `demoDistanceShot && DEMO_MODE` → flywheel at 2000 RPM + prestage + feeders after wait

**Feeder + Transport (real teleop path):**
- `shootButton && !TUNING_MODE && !(isShootSafeZone && !isShootSafeTime) && isAlignedLooser` →
  `sequence(waitUntil(isFlywheelSpunUp && isAlignedLooser).withTimeout(0.5s), run upper+lower feeder+transport)`;
  onFalse: stop all three

**Hood:**
- `(shootButton || shootFromTowerButton) && isShootClear && !TUNING_MODE` → `setHoodPosForHub`
- `(shootButton && !isShootSafeZone) || passButton && !TUNING_MODE` → `setPosForPassing`
- `shootButton && TUNING_MODE` → `setHoodPos(12.25°)`
- `demoDistanceShot && DEMO_MODE` → `setHoodPos(45°)`

**Intake Pivot:**
- `intakeInButton` → set `compressCancelled=true`, hold `pivotUpPos=0.3rot`
- `intakeOutButton` → set `compressCancelled=true`, hold `pivotDownPos=0.0rot`
- `intakeCompressButton` → set `compressCancelled=true`, `compressPivot(() -> doubleCompress)`, onFalse: pivotDown
- `shootButton && !compressCancelled && !(isShootSafeZone && !isShootSafeTime) && isAlignedLooser` →
  `sequence(waitUntil(isFlywheelSpunUp).withTimeout(0.5s), compressPivot(() -> doubleCompress))`; onFalse: pivotDown

**Intake Roller:**
- `intakeRollerButton` → hold 12 V; onFalse: stop

**Cancellations on shoot button release:**
- `compressCancelled = false`
- `xCancelled = false`
- `doubleCompress = false`

### Named Commands (PathPlanner)

Registered before `buildAutoChooser()`:

| Name | Command |
|---|---|
| `"DeployIntake"` | `IntakePivotCommands.setPivotPosition(pivotDownPos=0.0rot)` |
| `"RetractIntake"` | `IntakePivotCommands.setPivotPosition(pivotUpPos=0.3rot)` |
| `"RunIntake"` | `setRollerVoltage(12V).alongWith(setTransportVoltage(-7V))` |
| `"Shoot"` | `joystickDriveAtAngle(at hub angle).alongWith(autoShootToHub(...))` |
| `"stopAll"` | `ShootSequences.stopAll(...)` |
| `"HoodDownNamed"` | `HoodCommands.setHoodPos(0°)` |

### Event Triggers

All declared without subsystem requirements to avoid interrupting the path-following command group:

| Trigger | Action |
|---|---|
| `"DeployIntake"` onTrue | `runOnce(() -> intakePivot.setPivotPosition(0.0rot))` (no req) |
| `"RetractIntake"` onTrue | `runOnce(() -> intakePivot.setPivotPosition(0.3rot))` (no req) |
| `"RunIntake"` whileTrue | `startEnd(() -> intakeRoller.setRollerVoltage(12V), () -> intakeRoller.setRollerVoltage(0V))` (no req) |
| `"HoodDown"` onTrue | `HoodCommands.setHoodPos(hood, 0°)` |

### Pre-Match Auto Checks

- `updateAutoPreview()` — loads paths from selected auto `.auto` file, draws on `Field2d`
  "Auto Preview". Stores `autoStartPose` from first path's starting holonomic pose.
- `checkStartPose()` — compares current pose to `autoStartPose`. Logs:
  `Auto/StartCheck/DistanceInches`, `RotationDiffDegrees`, `PositionOK` (<6 in), `RotationOK` (<5°)

---

## 23. AllianceFlipUtil.java

Caches `shouldFlip()` result once per loop via `refresh()` (called from `robotPeriodic()`).
Returns `true` for Red alliance. All coordinate transformations:

```java
applyX(x) = shouldFlip() ? fieldLength - x : x
applyY(y) = shouldFlip() ? fieldWidth - y : y
apply(Translation2d) = new Translation2d(applyX, applyY)
apply(Rotation2d) = shouldFlip() ? rotation.rotateBy(π) : rotation
apply(Pose2d) = new Pose2d(apply(translation), apply(rotation))
apply(Translation3d) = new Translation3d(applyX, applyY, z)
```

---

## 24. LoggedTrigger.java

Extends `Trigger`. Wraps condition in `LoggingBooleanSupplier` that calls
`Logger.recordOutput(name, value)` on state change (or first evaluation). Name is the AdvantageKit
log key.

`and(LoggedTrigger)` returns a new `LoggedTrigger` with combined name `"A_AND_B"`.

`debounce(seconds, type)` returns a new `LoggedTrigger` wrapping the debounced super result.

---

## 25. ContinuousConditionalCommand.java

Extends `Command`. Re-evaluates `condition` every `execute()`. If condition changes, calls
`selectedCommand.end(true)`, then swaps to other command and calls `initialize()`.

Requires all subsystems from both `onTrue` and `onFalse`.

`InterruptionBehavior` = `kCancelSelf` if either branch is `kCancelSelf`; else `kCancelIncoming`.

---

## 26. BatteryLogger.java

Called from each subsystem's `periodic()`: `Robot.batteryLogger.reportCurrentUsage(key, isDrive, amps...)`.

Accumulates per-key current, power, energy. Aggregates parent keys (split on `/` or `-`).

`periodicAfterScheduler()` (called from `robotPeriodic()` after scheduler):
- Adds fixed overhead: roboRIO current, 4×CANcoder×0.05A, Pigeon 0.04A, CANivore 0.03A, Radio 0.5A
- Logs all to `BatteryLogger/Current/...`, `BatteryLogger/Power/...`, `BatteryLogger/Energy/...`
- Logs totals `BatteryLogger/Current`, `BatteryLogger/DriveCurrent`, `BatteryLogger/Power`,
  `BatteryLogger/Energy` (Wh)
- Resets per-cycle power and current; energy accumulates across match

---

## 27. Vision System

**`Vision.java`** subscribes to each camera's `VisionIOInputs`, applies all rejection filters
per observation, and calls `consumer.accept(pose2d, timestamp, stdDevMatrix)` where consumer is
`drive::addVisionMeasurement`.

Filtering pipeline per observation:
1. `ambiguity > 0.4` → reject (single-tag only; multi-tag is always trusted)
2. `averageTagDistance > 6.0 m` → reject
3. `|yawVelocityRadPerSec| > 4.0` → reject
4. `pitch or roll > 25°` → reject
5. `pose.z > 2 m` or `pose.z < -6in` → reject
6. Pass → compute stdDevs = baseline × dist² / tagCount × camera factor
   - If single-tag: × singleTagStdDevMultiplier=2.0
   - If MegaTag2: linear × 0.5, angular = ∞

---

## 28. HubShiftUtil — Complete API

**Key static fields:**
- `flipped` — whether alliance winner is flipped for testing
- `disabled` — when true, `isShootSafeTime` always returns true
- `shiftTimer` — internal Timer started at teleop init
- `shiftTimerOffset` — adjustment to sync with FMS teleop clock

**`initialize()`** — resets offset, restarts timer. Called at `teleopInit()`.

**`getOfficialShiftInfo()`** — returns `ShiftInfo` using raw schedule window times.

**`getShiftedShiftInfo()`** — returns `ShiftInfo` with fudge-adjusted window boundaries.
Switches on whether `shiftSchedule[1] == true` (starting-active path vs starting-inactive path).

**`ShiftInfo` record:** `(ShiftEnum currentShift, double elapsedTime, double remainingTime, boolean active)`

**`flipWinner()`** — `InstantCommand` that toggles `flipped` and sets `allianceWinOverride`.

**`disableHubShiftUtil()`** — `InstantCommand` that toggles `disabled`.

**`isActiveFirst()`** — returns true if DriverStation alliance == `getFirstActiveAlliance()`.

**`getFirstActiveAlliance()`** — priority: override supplier → FMS GSM (`'R'`→Blue, `'B'`→Red) → opposite of DriverStation alliance.

---

## 29. Robot Physical Specifications

| Property | Value |
|---|---|
| Mass | 63.5 kg |
| Moment of inertia | 5.162 kg·m² |
| Wheel COF | 1.2 |
| Wheel base | 22 in × 22 in (±11 in from center) |
| Drive base radius | computed from TunerConstants module locations |
| Max drive speed | ~4.39 m/s @ 12 V |
| Flywheel launch height | ~20 in above floor |
| Log storage | USB at `/U/` (required for match logs) |
| Build system | Gradle, JVM heap 100 MB max, SerialGC, 50 ms max pause |

---

## 30. Autonomous Routines

| File | Description |
|---|---|
| `2.5-Left-Comp` | **Default** — 2.5-piece left-side competition auto |
| `2.5-Right-Comp` | 2.5-piece right-side |
| `Safe-2-Left-Comp` / `Safe-2-Right-Comp` | Conservative 2-piece variants |
| `Champs-Left` / `Champs-Right` | Championship full autos |
| `Champs-Follow-Left/Right` | Follow-path championships variants |
| `Champs-Safe-Left/Right` | Safe championships variants |
| `Center-Bump` | Center bump auto |
| `Left-Depot` / `Left-Double` / `Right-Double` | Depot and double-piece |
| `Left-Disruptor` / `Right-Disruptor` | Disruption autos |
| `Qual55` / `qual2` / `State-Elims` | Event-specific saved autos |

Auto command = `Commands.sequence(Commands.waitSeconds(delay), autoChooser.get().asProxy())`.
Delay is read from SmartDashboard key `"Auto Delay"` (default 0.0 s).
