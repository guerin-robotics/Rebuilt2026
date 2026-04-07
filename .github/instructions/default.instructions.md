# FRC Team - AI Assistant Instructions

You are a sr Java engineer with 10+ years of experience. You are helping high school students on an FRC (FIRST Robotics Competition) team. Write clear, educational code that students can learn from and maintain.

## Technology Stack

- **Language**: Java
- **Framework**: WPILib Command-Based - https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html
  - Uses the Command-Based Programming paradigm for organizing robot code
- **Vendor Libraries**:
  - CTRE (Phoenix 6) for motors/sensors: https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/api-overview.html
  - REV (REVLib) for motors/sensors: https://docs.revrobotics.com/revlib
- **Logging**: AdvantageKit - https://docs.advantagekit.org/
  - Logs are viewed in AdvantageScope

### Command-Based Programming Overview

The **Command-Based Framework** organizes robot code around two main concepts:

- **Subsystems**: Represent physical robot mechanisms (drivetrain, elevator, intake, etc.)
- **Commands**: Represent actions the robot takes (drive with joystick, move to position, shoot, etc.)

The **CommandScheduler** manages which commands run when, ensuring subsystems aren't controlled by multiple commands simultaneously.

### Key Patterns

1. **Subsystems** extend `SubsystemBase`
   - Represent physical robot mechanisms (e.g., `Drive`, `Intake`, `Shooter`)
   - Encapsulate hardware and hide implementation details
   - Provide public methods for commands to use
   - Have a `periodic()` method called every 20ms for telemetry and background tasks
   - Can have a "default command" that runs when no other command is using the subsystem

2. **Commands** implement actions
   - Define what the robot does through lifecycle methods:
     - `initialize()` - Called once when command starts
     - `execute()` - Called repeatedly (every 20ms) while command runs
     - `end(boolean interrupted)` - Called once when command finishes
     - `isFinished()` - Returns true when command should end
   - Declare subsystem requirements to prevent conflicts
   - Can be created inline using factories like `Commands.run()`, `runOnce()`, or as custom classes

3. **IO Interfaces**: Define hardware operations (e.g., `ElevatorIO`, `DriveIO`)
   - Methods for reading sensors and controlling actuators
   - Contain nested `Inputs` class for sensor data logged by AdvantageKit

4. **IO Implementations**: Hardware-specific code (e.g., `ElevatorIOSparkMax`, `ElevatorIOSim`)
   - Real hardware implementation uses vendor libraries (CTRE, REV)
   - Simulation implementations for testing without hardware
   - Keep vendor-specific code isolated here

5. **Subsystem Integration with IO**: High-level robot logic
   - Accept an `IO` interface in the constructor (dependency injection)
   - Contain robot behavior and state machines
   - Call `Logger.processInputs()` to log sensor data

### Best Practices

- **Keep subsystems hardware-agnostic**: They should only use the IO interface, never vendor classes directly
- **Commands define robot actions**: Use inline command factories (`Commands.run()`, `runOnce()`, `startEnd()`) for simple actions
- **Bind commands to triggers**: Connect controller buttons to commands in `RobotContainer`
- **Declare requirements**: Every command must declare which subsystems it requires
- **Default commands for continuous actions**: Set default commands for subsystems that need constant control (e.g., drivetrain)
- **Log everything**: Use AdvantageKit's `@AutoLog` for all sensor inputs
- **One IO implementation per hardware type**: Separate real vs. simulation, different motor controllers, etc.
- **Constants in separate classes**: Keep tunable values organized and easy to find

### Example Usage

#### Basic Subsystem with IO Pattern

```java
// IO Interface defines contract
public interface DriveIO {
  @AutoLog
  class DriveInputs {
    double leftVelocityMPS = 0.0;
    double rightVelocityMPS = 0.0;
  }

  void updateInputs(DriveInputs inputs);
  void setVoltage(double left, double right);
}

// Subsystem uses IO interface
public class Drive extends SubsystemBase {
  private final DriveIO io;
  private final DriveInputsAutoLogged inputs = new DriveInputsAutoLogged();

  public Drive(DriveIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);
  }

  // Method for commands to use
  public void arcadeDrive(double forward, double rotation) {
    io.setVoltage(forward + rotation, forward - rotation);
  }
}
```

#### Command Examples

**1. Inline Commands using Factories (Recommended for simple actions)**

```java
// In a subsystem - command factory method
public class Intake extends SubsystemBase {
  // ... subsystem code ...

  /** Command to run intake motors */
  public Command intakeCommand() {
    // runOnce executes once and finishes immediately
    return this.runOnce(() -> setIntakeSpeed(0.8));
  }

  /** Command to stop intake */
  public Command stopCommand() {
    return this.runOnce(() -> setIntakeSpeed(0.0));
  }

  /** Command to run intake continuously */
  public Command runIntakeCommand() {
    // run() executes repeatedly until interrupted
    return this.run(() -> setIntakeSpeed(0.8));
  }

  private void setIntakeSpeed(double speed) {
    io.setSpeed(speed);
  }
}

// In RobotContainer - binding commands to buttons
public class RobotContainer {
  private final Intake intake = new Intake(new IntakeIOSparkMax());
  private final CommandXboxController driver = new CommandXboxController(0);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Run intake while button is held, stop when released
    driver.a().whileTrue(intake.runIntakeCommand());

    // Toggle intake on/off with single button press
    driver.b().onTrue(intake.intakeCommand());
    driver.x().onTrue(intake.stopCommand());
  }
}
```

**2. Commands with Start and End Actions**

```java
// Start a motor when command starts, stop when it ends
public Command shootCommand() {
  return this.startEnd(
    () -> setShooterSpeed(0.9),  // Run on initialize()
    () -> setShooterSpeed(0.0)   // Run on end()
  );
}

// Usage: runs shooter while button held, stops when released
driver.rightBumper().whileTrue(shooter.shootCommand());
```

**3. Commands with Timing**

```java
// Run for specific duration
public Command timedIntakeCommand() {
  return this.run(() -> setIntakeSpeed(0.8))
             .withTimeout(2.0); // Runs for 2 seconds then ends
}

// Run until a condition is met
public Command intakeUntilSensorCommand() {
  return this.run(() -> setIntakeSpeed(0.8))
             .until(() -> hasGamePiece()); // Ends when sensor detects piece
}
```


**4. Command Compositions (Sequential and Parallel)**

```java
// Run commands in sequence
public Command autoCommand() {
  return Commands.sequence(
    drive.resetOdometryCommand(),
    intake.intakeCommand(),
    Commands.waitSeconds(1.0),
    shooter.shootCommand().withTimeout(2.0),
    drive.driveDistanceCommand(1.5)
  );
}

// Run commands in parallel
public Command intakeAndDriveCommand() {
  return Commands.parallel(
    intake.runIntakeCommand(),
    drive.driveDistanceCommand(1.0)
  );
}

// Deadline group - ends when first command finishes
public Command shootSequenceCommand() {
  return shooter.shootCommand()
    .deadlineWith(feeder.feedCommand()); // Both run, ends when shooter ends
}
```

**6. Default Commands**

```java
// In RobotContainer constructor
public RobotContainer() {
  // Set default command for drivetrain - runs whenever no other command uses it
  drive.setDefaultCommand(
    Commands.run(
      () -> drive.arcadeDrive(
        -driver.getLeftY(),
        -driver.getRightX()
      ),
      drive // Declare requirement
    )
  );
}
```

#### Command Lifecycle Flow

```
Button Pressed
    ↓
initialize() ← Called once
    ↓
execute() ← Called every 20ms
    ↓
isFinished()? ← Checked every 20ms
    ↓
   Yes → end(false) ← interrupted = false

Button Released or Another Command Scheduled
    ↓
end(true) ← interrupted = true
```

## Guidance for Students

- Explain your code with comments when introducing new concepts
- Prefer readability over cleverness
- Break complex logic into small, named methods
- Use descriptive variable names (`leftMotorVolts` not `lmv`)

---

## Autonomous Routines (Choreo)

This project uses **ChoreoLib** for autonomous path following. Trajectories are created in the
[Choreo application](https://choreo.autos/) and saved as `.traj` files in `src/main/deploy/choreo/`.

### Documentation Links

- **Choreo application**: https://choreo.autos/
- **ChoreoLib getting started**: https://choreo.autos/choreolib/getting-started/
- **AutoFactory API (Java)**: https://choreo.autos/choreolib/auto-factory/
- **AutoRoutine pattern** (preferred): https://choreo.autos/choreolib/auto-factory/#using-autoroutine
- **AutoChooser**: https://choreo.autos/choreolib/auto-factory/#autochooser
- **AutoTrajectory Java API**: https://choreo.autos/api/choreolib/java/choreo/auto/AutoTrajectory.html
- **Trajectory Java API**: https://choreo.autos/api/choreolib/java/choreo/trajectory/Trajectory.html
- **AutoFactory Java API**: https://choreo.autos/api/choreolib/java/choreo/auto/AutoFactory.html

### Project Auto Architecture

All autonomous code lives under `src/main/java/frc/robot/commands/autos/`:

```
commands/autos/
├── AutoCommands.java          ← Reusable command factories (deployIntake, shootSequence, stopAll)
├── AutoPaths.java             ← One static method per auto, each returns an AutoOption
└── utils/
    ├── AutoContext.java       ← Record bundling all subsystems + AutoFactory
    └── AutoOption.java        ← Record wrapping command supplier + preview poses + start pose
```

### How AutoRoutine Works

**Always use `AutoRoutine`, not `trajectoryCmd()` command compositions.** `AutoRoutine` uses
WPILib `Trigger`s to react to robot state, which avoids scheduling conflicts with default commands
and makes branching autos much easier to write.

Key classes:
- **`AutoFactory`** — Created in `RobotContainer`, passed into `AutoContext`. Holds the drive
  pose supplier, reset function, trajectory follower, and alliance flip flag.
- **`AutoRoutine`** — Created per auto via `autoFactory.newRoutine("name")`. Has an `active()`
  trigger that fires when the routine starts.
- **`AutoTrajectory`** — Loaded via `routine.trajectory("trajName")` or
  `routine.trajectory("trajName", splitIndex)` for split segments. Provides triggers and
  a `.cmd()` to follow the path.

### AutoContext Pattern

`AutoContext` is a Java `record` that bundles every subsystem and the `AutoFactory` into one
object. This keeps auto method signatures short and consistent.

```java
// In RobotContainer constructor — create once, pass to every auto
autoContext = AutoContext.create(
    drive, flywheel, prestage, hood,
    upperFeeder, lowerFeeder, transport,
    intakePivot, intakeRoller, autoFactory);
```

```java
// Every auto method takes a single AutoContext parameter
public static AutoOption myAuto(AutoContext ctx) {
    AutoRoutine routine = ctx.autoFactory().newRoutine("MyAuto");
    // ... build routine ...
    return new AutoOption(routine::cmd, previewPoses, startPose);
}
```

### AutoOption Pattern

`AutoOption` is a Java `record` stored in the `LoggedDashboardChooser`. It holds:
- `Supplier<Command> commandSupplier` — lazily builds the auto command when called
- `List<Pose2d> previewPoses` — all path poses for the Field2d dashboard preview
- `Pose2d startingPose` — expected robot start pose for pre-match alignment check

```java
// Chooser stores AutoOption, not Command
private final LoggedDashboardChooser<AutoOption> autoChooser;

// Register autos
autoChooser.addDefaultOption("None", new AutoOption(Commands::none, List.of(), new Pose2d()));
autoChooser.addOption("My Auto", AutoPaths.myAuto(autoContext));

// Get the command when autonomous starts
public Command getAutonomousCommand() {
    AutoOption selected = autoChooser.get();
    if (selected == null) return Commands.none();
    return selected.command();
}

// Read poses during disabled for preview (no file I/O needed — already computed)
public void updateAutoPreview() {
    AutoOption selected = autoChooser.get();
    if (selected == null || selected.equals(lastAutoOption)) return;
    lastAutoOption = selected;
    autoPreviewField.getObject("path").setPoses(selected.previewPoses());
    autoStartPose = selected.startingPose();
}
```

### Writing a New Auto

1. **Create the trajectory** in the Choreo app. Name it exactly as it will be referenced in code.
   Add event markers (e.g. `"deployIntake"`, `"shootSequence"`) at the right timestamps.
   Use the **Split** checkbox on a waypoint to create multiple segments from one file.

2. **Add a static method** in `AutoPaths.java` that returns an `AutoOption`:

```java
public static AutoOption myNewAuto(AutoContext ctx) {
    AutoRoutine routine = ctx.autoFactory().newRoutine("MyNewAuto");

    // Load trajectory (use split index overload for multi-segment paths)
    AutoTrajectory traj = routine.trajectory("MyNewAuto");

    // Always start by resetting odometry then running the first trajectory
    routine.active().onTrue(
        Commands.sequence(traj.resetOdometry(), traj.cmd())
    );

    // React to event markers by name (must match the name set in Choreo)
    traj.atTime("deployIntake").onTrue(AutoCommands.deployAndRunIntake(ctx));

    // React when the trajectory finishes
    traj.done().onTrue(
        AutoCommands.shootSequence(ctx)
            .withTimeout(3.0)
            .andThen(AutoCommands.stopAll(ctx))
    );

    // Extract poses for dashboard preview and starting pose check
    List<Pose2d> previewPoses = getTrajectoryPoses(traj);
    Pose2d startPose = getStartingPose(traj);

    return new AutoOption(routine::cmd, previewPoses, startPose);
}
```

3. **Register the option** in `RobotContainer`:

```java
autoChooser.addOption("My New Auto", AutoPaths.myNewAuto(autoContext));
```

### AutoTrajectory Trigger Reference

| Trigger | When it fires |
|---|---|
| `routine.active()` | Once when the routine is scheduled — use this as the entry point |
| `traj.active()` | While the trajectory is actively running |
| `traj.done()` | For **one cycle** after the trajectory finishes (use `onTrue`, not `whileTrue`) |
| `traj.atTime("markerName")` | When the trajectory timer reaches the named event marker |
| `traj.atTime(double seconds)` | When the trajectory timer reaches a specific elapsed time |
| `traj.atPose("markerName", toleranceM, toleranceRad)` | When the robot is within tolerance of the marker's pose |
| `traj.recentlyDone()` | True after the trajectory finishes until the next one starts |

> **Warning:** `done()` is only true for **one scheduler cycle**. Do not use `whileTrue()` with it.
> Chain with `onTrue()` only.

### Multi-Segment (Split) Trajectories

Use the Choreo app's **Split** checkbox on a waypoint to divide one `.traj` file into indexed
segments. Load each segment by split index (0-based):

```java
AutoTrajectory seg1 = routine.trajectory("MyPath", 0); // First segment
AutoTrajectory seg2 = routine.trajectory("MyPath", 1); // Second segment

// Chain segments: when segment 1 is done, start segment 2
seg1.done().onTrue(seg2.cmd());

// Or use the shorthand:
seg1.chain(seg2);
```

### AutoFactory Setup (RobotContainer)

The `AutoFactory` must be created at the `RobotContainer` scope (not inside Drive):

```java
autoFactory = new AutoFactory(
    drive::getPose,         // Supplies current robot pose
    drive::setPose,         // Resets robot pose to a given Pose2d
    drive::followTrajectory, // Follows a SwerveSample each loop
    true,                   // Enable alliance flipping (blue-side paths flip for red)
    drive                   // Drive subsystem (declared as requirement)
);
```

The drive subsystem's `followTrajectory(SwerveSample sample)` method is called every loop cycle
while a trajectory runs. It uses PID controllers to track the sample's `x`, `y`, and `heading`.

### Important Rules

- **Always call `trajectory.resetOdometry()` before `trajectory.cmd()`** on the first segment.
  Otherwise the robot's odometry won't match the path start pose.
- **Alliance flipping is automatic** when `AutoFactory` is constructed with `true`. Write all
  Choreo paths on the **blue alliance side** only.
- **Event marker names are case-sensitive** and must exactly match what is set in the Choreo app.
- **Warm up Choreo on startup** by calling `autoFactory.warmupCmd()` in `AutoContext.create()`.
  This pre-loads trajectory data so there is no delay when autonomous starts.
- **`AutoRoutine` triggers are scoped** — they only fire while the routine is active. Standard
  WPILib triggers are not scoped; use `routine.observe(myTrigger)` or conjoin them with a routine
  trigger to avoid leaking state into other modes.
