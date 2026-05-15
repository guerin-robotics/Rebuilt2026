# Robot Architecture Overview

## Project Structure

```
Rebuilt2026/
├── build.gradle                            GradleRIO 2026.2.1, Java 17
├── settings.gradle
├── vendordeps/                             Vendor library JSON files
│   ├── Phoenix6-26.2.0.json                CTRE motors, CANcoder, Pigeon2
│   ├── AdvantageKit.json                   Logging and replay framework
│   ├── PathplannerLib.json                 Autonomous path planning
│   ├── photonlib.json                      PhotonVision camera integration
│   └── WPILibNewCommands.json              Command-based framework
└── src/main/java/frc/robot/
    ├── Robot.java                          Entry point (AdvantageKit LoggedRobot)
    ├── RobotContainer.java                 Subsystem wiring, bindings, auto setup
    ├── Constants.java                      Mode/RobotType enum
    ├── HardwareConstants.java              Motor IDs, velocities, positions, thresholds
    ├── RobotState.java                     Singleton for pose and distance
    ├── Triggers.java                       Controller button definitions
    ├── generated/TunerConstants.java       Phoenix Tuner X auto-generated config
    ├── subsystems/                         10 subsystems (see subsystems.md)
    └── commands/                           11 command files (see commands.md)
```

---

## Framework Stack

| Layer | Technology |
|-------|-----------|
| Build | GradleRIO 2026.2.1 |
| Robot base class | AdvantageKit `LoggedRobot` |
| Command scheduler | WPILib command-based |
| Motor controllers | CTRE Phoenix 6 (TalonFX / Kraken X60) |
| Logging/replay | AdvantageKit 3.x |
| Autonomous | PathPlanner 2026 |
| Vision | PhotonVision |
| Path following | `PPHolonomicDriveController` |
| Pathfinding | `LocalADStarAK` (local AD* variant) |
| Pose estimation | WPILib `SwerveDrivePoseEstimator` |

---

## Operating Modes

Defined in `Constants.java`:

```
Constants.currentMode
  REAL    — Physical robot (logs to USB, broadcasts via NT4)
  SIM     — WPILib simulation (PhotonVision sim cameras, motor sims)
  REPLAY  — Offline replay from .wpilog file (runs without hardware)
```

`Constants.robotType` selects which `TunerConstants` variant to load:
- `COMP` — Competition robot
- `ALPHA` — Alpha/testing robot
- `NONE` — No hardware

AdvantageKit replay is fully supported: all subsystem IO inputs are logged and
can be replayed offline to re-run robot logic against recorded match data.

---

## IO Abstraction Pattern

Every subsystem that touches hardware follows this pattern:

```
SubsystemIO (interface)          Contract: what the hardware layer must provide
  SubsystemIOReal                Production: TalonFX, CANcoder, sensors
  SubsystemIOSim                 Simulation: WPILib FlywheelSim, DCMotorSim, etc.
  SubsystemIO {}                 Stub: no-op implementation for replay mode

SubsystemIOInputs (@AutoLog)     Data struct updated by the IO layer each cycle
SubsystemIOInputsAutoLogged      Generated class that auto-logs all fields

Subsystem (SubsystemBase)        Mode-independent: holds IO + inputs, runs logic
```

In `periodic()`:
```java
io.updateInputs(inputs);                     // Hardware reads happen here
Logger.processInputs("SubsystemName", inputs); // AdvantageKit logs everything
// ... use inputs.field for logic
```

Subsystem instantiation switches on `Constants.currentMode`:
```java
case REAL:    subsystem = new Subsystem(new SubsystemIOReal());
case SIM:     subsystem = new Subsystem(new SubsystemIOSim());
default:      subsystem = new Subsystem(new SubsystemIO() {});
```

This means the subsystem class itself never imports hardware libraries — all
hardware coupling is isolated inside `*IOReal` classes.

---

## Data Flow

```
Hardware sensors
  └─> SubsystemIO.updateInputs()       (IO layer reads registers)
        └─> Logger.processInputs()     (AdvantageKit logs raw inputs)
              └─> Subsystem.periodic() (business logic, state machines)
                    └─> RobotState     (shared singleton: pose, distance)
                          └─> Commands (read state, write outputs via subsystem methods)
                                └─> SubsystemIO.setVoltage/setVelocity()
                                      └─> TalonFX control requests
```

Vision cameras feed asynchronously into the drivetrain's pose estimator:
```
VisionIO.updateInputs()
  └─> Vision.periodic() — filters observations
        └─> consumer.accept() — drive::addVisionMeasurement
              └─> SwerveDrivePoseEstimator.addVisionMeasurement()
```

---

## Scheduler Interaction

The WPILib `CommandScheduler` runs at 50 Hz (every 20 ms). Each subsystem's
`periodic()` runs unconditionally. Commands are bound to `Trigger` objects that
poll controller state and schedule/cancel commands automatically.

Default commands run when no other command requires that subsystem:
- `Drive`: `joystickDrive()` (normal teleop driving)
- `Hood`: `hoodIdle()` (holds position, avoids stale PID reference)

Command composition uses WPILib factory methods:
- `Commands.sequence()` — run in order
- `Commands.parallel()` — run simultaneously, end when all finish
- `Commands.race()` — run simultaneously, end when first finishes
- `Commands.startEnd()` — shorthand for initialize + end
- `.whileTrue()` / `.onTrue()` — trigger bindings

---

## RobotState Singleton

`RobotState.java` provides a single read-only interface to shared robot state:

- `setPoseSupplier(Supplier<Pose2d>)` — wired to `drive.getPose()` at startup
- `getEstimatedPose()` — current robot position on field
- `getDistanceToHub()` — distance to the scoring hub (used for shot lookup)
- `getFieldRelativeVelocity()` — used by vision filter
- `getSpeakerAngleRad()` — heading error to hub target

All `@AutoLogOutput`-annotated methods are registered with
`AutoLogOutputManager.addObject(RobotState.getInstance())` in `Robot.java` so
they appear in AdvantageScope logs.

---

## CAN Bus Layout

| ID Range | Device |
|----------|--------|
| 0 | Pigeon 2 gyro (in Drive subsystem) |
| 30–34 | Flywheel motors (1 leader + 4 followers) |
| 35 | Hood motor |
| 36 | Upper feeder motor |
| 37–38 | Prestage motors (leader + follower) |
| 39 | Lower feeder motor |
| 40 | Transport motor |
| 41–43 | Intake pivot + roller (2 roller motors) |
| 44 | Intake pivot CANcoder |
| 50 | Hood CANcoder |
| Drive modules | Defined in generated `TunerConstants.java` |

---

## Field Coordinate System

- Origin: Blue alliance corner (bottom-left when field viewed from above)
- X: Positive toward red alliance wall
- Y: Positive toward left when facing red alliance
- Heading: CCW positive
- Red alliance: all coordinates auto-flipped via `AllianceFlipUtil.apply()`
