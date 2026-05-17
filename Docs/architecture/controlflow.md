# Control Flow

All flows verified against source. References include file:line.

---

## 1. Autonomous Execution Flow

### Phase 1: Disabled (before match)

```
RobotContainer.updateAutoPreview()   ← called every disabled cycle
  ├── Read autoChooser.get().getName()
  ├── PathPlannerAuto.getPathGroupFromAutoFile(name)
  ├── Collect all path waypoints
  ├── autoPreviewField.getObject("path").setPoses(poses)
  └── checkStartPose()
        ├── compare Drive.getPose() to auto's starting pose
        ├── Logger.recordOutput("Auto/StartCheck/DistanceInches", dist)
        └── Logger.recordOutput("Auto/StartCheck/PositionOK", dist <= 6in)
```

### Phase 2: autonomousInit()

```
Robot.java: autonomousInit()
  ├── autonomousCommand = robotContainer.getAutonomousCommand()
  │     ├── delay = SmartDashboard.getNumber(autoDelayKey, 0.0)
  │     └── return Commands.sequence(
  │               Commands.waitSeconds(delay),
  │               autoChooser.get().asProxy()
  │           )
  └── autonomousCommand.schedule()
```

`.asProxy()` wraps the auto in a `ProxyCommand` so it runs in a separate
composition scope. This prevents the auto group from cancelling commands that
were already running (e.g., default commands) during its own initialization.

### Phase 3: Auto execution (PathPlanner)

```
PathPlanner auto group (from .auto file)
  │
  ├── Path segment (from .path file)
  │     ├── AutoBuilder follows path using PPHolonomicDriveController
  │     │     ├── Translation PID (kP=5.0): corrects X/Y error
  │     │     ├── Rotation PID (kP=5.0): corrects heading error
  │     │     └── Drive.runVelocity(ChassisSpeeds) called each cycle
  │     └── Event markers trigger named commands or event triggers
  │
  ├── Named command "DeployIntake"
  │     └── IntakePivotCommands.setPivotPosition(down)
  │           Requires: IntakePivot
  │
  ├── Named command "RunIntake"
  │     └── parallel(
  │             intakeRollerCommands.setRollerVoltage(12V),
  │             TransportCommands.setTransportVelocity(-1800 RPM)
  │         )
  │         Requires: intakeRoller, Transport
  │
  ├── Named command "Shoot"
  │     └── ShootSequences.autoShootToHub(...)
  │           ├── parallel(
  │           │     FlywheelCommands.setVelocityForHub()  ← looks up RPM from RobotState
  │           │     PrestageCommands.setPrestageVelocity(3000 RPM)
  │           │     HoodCommands.setHoodPosForHub()       ← looks up angle from RobotState
  │           │   )
  │           └── sequence(
  │                 waitUntil(flywheel.isFlywheelSpunUp).withTimeout(0.5s),
  │                 parallel(
  │                   FeederCommands.setLowerFeederVelocity(-3000 RPM)
  │                   FeederCommands.setUpperFeederVelocity(-3000 RPM)
  │                   TransportCommands.setTransportVelocity(-1800 RPM)
  │                   intakeRollerCommands.setRollerVoltage(3V agitate)
  │                   IntakePivotCommands.autoPivotCompress(...)
  │                 )
  │               )
  │
  └── Named command "stopAll"
        └── ShootSequences.stopAll()
              Zeros all shooter subsystem setpoints

EventTrigger callbacks (fire independently by robot position):
  ├── "DeployIntake" → Commands.runOnce(() -> intakePivot.setPivotPosition(down))
  ├── "RetractIntake" → Commands.runOnce(() -> intakePivot.setPivotPosition(up))
  ├── "RunIntake" → Commands.runOnce(() -> intakeRoller.setVoltage(12V))
  └── "HoodDown" → Commands.runOnce(() -> hood.setHoodPos(0°))
      (all without subsystem requirements — see dependency-graph.md)
```

---

## 2. Teleop Shoot Flow

### Button: shootButton (Joystick button 1)

```
shootButton.and(isShootSafeZone)
  .whileTrue(DriveCommands.alignOrXForShoot(drive, ...))
        │
        ▼
  alignOrXForShoot(ContinuousConditionalCommand):
    ├── condition: Triggers.isAlignedForCurrentShot
    ├── if aligned (true):  stopWithX(drive)
    │     └── sets all module angles to X pattern, drive.areWheelsXed = true
    └── if not aligned (false): joystickDriveAtAngle(drive, x, y, targetAngle)
          ├── targetAngle from RobotState.getSpeakerAngleRad() or getPassAngleRad()
          ├── ProfiledPIDController: Kp=7.5, Kd=0.4, max ω=8.0 rad/s
          └── drives translation with joystick, holds heading to target

shootButton.and(isShootClear).and(!TUNING_MODE)
  .whileTrue(
      FlywheelCommands.setVelocityForHub(flywheel)     ← looks up RPM
      .alongWith(PrestageCommands.setPrestageVelocity(prestage, 3000 RPM))
  )
  .onFalse(FlywheelCommands.stop(flywheel))

shootButton.and(!compressCancelled)
  .whileTrue(
      Commands.sequence(
          Commands.waitUntil(flywheel.isFlywheelSpunUp).withTimeout(0.5s),
          IntakePivotCommands.compressPivot(intakePivot, () -> doubleCompress)
      )
  )
```

### Aligned + spun up: full shot sequence

When all conditions met simultaneously:
```
Drive: X-locked (holding position, resisting defense)
Flywheel: running at distance-based RPM
Hood: at distance-based angle
IntakePivot: compressing toward shooter
  └── on spinup: feeders and transport engage
```

### Auto-compress state machine (RobotContainer)

```
compressCancelled = false (initial)

shootButton pressed:
  → start sequence: wait for spinup, then compress pivot

intakeInButton / intakeOutButton / compressButton pressed:
  → compressCancelled = true
  → compress sequence interrupted (scheduledCommand cancelled)

shootButton released:
  → compressCancelled = false (reset)
  → xCancelled = false (reset)
  → doubleCompress = false (reset)

doubleCompressOverride pressed:
  → doubleCompress = !doubleCompress (toggle)
```

---

## 3. Driver Input Flow

### Joystick Input Processing (DriveCommands.joystickDrive)

```
Raw joystick axes
  │
  ▼
Apply deadband (0.08, from HardwareConstants.ControllerConstants)
  ├── if magnitude < 0.08: zero entire vector
  └── else: scale magnitude by (magnitude - deadband) / (1 - deadband)
  │
  ▼
Square input magnitude (nonlinear feel: finer control at low speed)
  │
  ▼
Scale to max velocity (DriveConstants.maxLinearSpeed)
  │
  ▼
Alliance check: if RED alliance → rotate vector 180°
  (DriverStation.getAlliance() == RED → multiply x,y by -1)
  │
  ▼
Field-relative conversion:
  ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, ω, robot.getHeading())
  │
  ▼
Drive.runVelocity(chassisSpeeds)
  │
  ▼
SwerveDriveKinematics.toSwerveModuleStates(speeds)
  │
  ▼
ModuleIO.setDriveVelocity() + ModuleIO.setTurnPosition() per module
```

### Trigger Evaluation (every 20ms)

Every trigger declared in `configureButtonBindings` is polled by the
CommandScheduler every cycle:

```
CommandScheduler.run() [50 Hz]
  ├── Poll all Triggers
  │     isShootSafeZone  → RobotState.getBroadZone() == ALLIANCE_ZONE
  │     isShootClear     → isShootSafeZone AND isShootSafeTime
  │     isAlignedForCurrentShot → RobotState.isAlignedToHub() with debounce
  │     isAlignedLooser  → same without debounce
  │     isFlywheelSpunUp → Flywheel.isSpunUp() (200 RPM threshold)
  │     shootButton      → joystick.button(1)
  │     [all other buttons]
  │
  ├── Schedule commands whose trigger conditions became true
  ├── Cancel commands whose trigger conditions became false (.whileTrue)
  └── Run execute() on all active commands
```

**Debounce on isAlignedForCurrentShot:** 0.3 seconds (Triggers.java:171–182).
The alignment condition must be continuously true for 0.3s before the trigger
fires. This prevents the X-lock from engaging on brief accidental alignment.

---

## 4. Intake Sequence Flow

### Deploy and run:

```
intakeOutButton.whileTrue(
    IntakePivotCommands.setPivotPosition(pivot, down=0.0 rot)
)

intakeInButton.whileTrue(
    IntakePivotCommands.setPivotPosition(pivot, up=0.3 rot)
)

intakeRollerButton.whileTrue(
    intakeRollerCommands.setVoltageAfterWait(roller, 12V, delay, alignedGate)
        └── waits for alignedGate OR timeout, then sets voltage
)
```

### setVoltageAfterWait detail:

```
Commands.sequence(
    Commands.waitUntil(alignedGate).withTimeout(wait),
    Commands.startEnd(
        () -> intakeRoller.setRollerVoltage(voltage),
        () -> intakeRoller.setRollerVoltage(0)
    )
)
```

The gate delays roller engagement until the pivot has reached position (or times
out). This prevents roller damage if pivot moves slowly.

---

## 5. Pass Shot Flow

```
passButton.whileTrue(
    FlywheelCommands.setVelocityForPassing(flywheel)  ← 2700 RPM
    .alongWith(HoodCommands.setPosForPassing(hood))   ← 35°
    .alongWith(DriveCommands.joystickDriveAtAngle(drive, x, y,
                   () -> RobotState.getPassAngleRad()))
)
```

No auto-compress and no feed until driver manually fires (no separate "fire pass"
command — assumes the same shootButton controls the feeders).

---

## 6. Tower Shot Flow

```
towerShotButton.whileTrue(
    FlywheelCommands.setFlywheelVelocity(flywheel, 1625 RPM)
    .alongWith(HoodCommands.setHoodPos(hood, 2.5°))
)
```

Fixed velocity and angle regardless of distance. Used when very close to target.

---

## 7. Auto → Teleop Transition

```
Robot.teleopInit()
  ├── if autonomousCommand != null: autonomousCommand.cancel()
  └── RobotContainer.teleopInit() [if implemented]
```

Cancelling the autonomous command cascades through all composed commands in the
group, triggering `end(interrupted=true)` on each active command. Default
commands (joystickDrive, hoodIdle) resume immediately as the scheduler fills
vacated requirements.

---

## 8. Teleop → Auto Transition

```
Robot.autonomousInit()
  ├── CommandScheduler cancels all running commands (WPILib default)
  └── schedules new autonomousCommand
```

Any `whileTrue` bindings that were active in teleop (e.g., shooter running
because shoot button was held at transition) are cancelled. PathPlanner's auto
group takes over all drive and shooter commands.

---

## 9. Swerve Module Control Flow

Each of the 4 swerve modules follows this inner loop:

```
Drive.runVelocity(ChassisSpeeds)
  │
  ▼
kinematics.toSwerveModuleStates(speeds)
  │
  ▼
SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeed)
  │
  for each module:
  ▼
Module.runSetpoint(SwerveModuleState)
  ├── Optimize state (flip 180° if shorter turn path)
  ├── Apply cosine scaling (reduce drive speed during large turn)
  ├── ModuleIO.setDriveVelocity(optimizedVelocity)    VelocityTorqueCurrentFOC
  └── ModuleIO.setTurnPosition(optimizedAngle)        PositionVoltage (CANcoder fused)
```

The drive motor's closed-loop control runs entirely on the TalonFX at 1 kHz.
The main 50 Hz loop only sends new setpoints.

---

## 10. Scheduler Conflict Scenarios

### Scenario A: Teleop binding fires during auto

Teleop button bindings remain registered throughout the match. If a Trigger
condition evaluates to true during auto:

- `.whileTrue()` bindings will schedule commands that conflict with the auto group
- The scheduler resolves conflicts by **cancelling the lower-priority command**
- By default, auto commands have no higher priority — last-scheduled wins

**Mitigation in this codebase:** Button bindings require physical button presses
which won't happen during auto. Condition-based triggers (isShootSafeZone,
isAlignedForCurrentShot) could theoretically fire, but they depend on
the robot being in specific zones which won't coincide with auto normally.

**No explicit safeguard** (no `and(!DriverStation.isAutonomous())` guard on
teleop bindings). Low practical risk but worth noting.

### Scenario B: Event trigger vs. named command conflict

`autoShootToHub` requires `IntakePivot`. If an EventTrigger for `DeployIntake`
fires while `autoShootToHub` is running and uses a proper `runOnce(method,
subsystem)` form, the scheduler cancels `autoShootToHub`.

**Fix already in place:** Event triggers use `Commands.runOnce(() -> method())`
without the subsystem argument. This works but the subsystem is still mutated
from two contexts. See dependency-graph.md for full analysis.

### Scenario C: compressCancelled not reset on teleop disable

State flags (`compressCancelled`, `xCancelled`, `doubleCompress`) are instance
fields of RobotContainer. They are only reset when `shootButton.onFalse()` fires
(RobotContainer.java:675–677). If the driver holds the shoot button at the
disable boundary, the button release may not fire before the mode transition,
leaving flags in a stale state at re-enable.

**Risk level:** Low (all flags default to false at construction).
