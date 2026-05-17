# Commands

11 command files, all in `src/main/java/frc/robot/commands/`. Commands are
stateless factory classes — each method returns a new `Command` object and does
not itself implement the `Command` interface.

---

## Command Lifecycle Patterns

The codebase uses WPILib factory methods rather than subclassing `CommandBase`:

```java
// Runs action once on start, cleanup action on end (whileTrue / whenActive)
Commands.startEnd(
    () -> subsystem.start(),   // initialize()
    () -> subsystem.stop()     // end()
)

// Runs action every cycle while active
Commands.run(
    () -> subsystem.set(supplier.get()),
    subsystem   // declares requirement
)

// One-shot action (completes in one cycle)
Commands.runOnce(
    () -> subsystem.setOnce(value),
    subsystem
)

// Conditional wait then action
Commands.waitUntil(condition).withTimeout(seconds)
    .andThen(Commands.run(...))
```

---

## DriveCommands.java

Requires: `Drive`

| Method | Description |
|--------|-------------|
| `joystickDrive(drive, xSupplier, ySupplier, omegaSupplier)` | Field-relative teleop drive. Default drive command. |
| `joystickDriveLimited(...)` | Same but applies velocity cap. |
| `joystickDriveAtAngle(drive, x, y, targetAngle)` | Drive while locking heading to a fixed angle. Used during alignment. |
| `alignOrXForShoot(drive, ...)` | Aligns heading toward hub; when aligned within threshold, also X-locks wheels. |
| `joystickDriveAlignForTrench(drive, ...)` | Aligns to trench landmark heading. |
| `joystickDriveAlignForBump(drive, ...)` | Aligns to bump zone landmark. |
| `stopWithX(drive)` | Commands all wheels to X formation (resist pushing). |
| `wheelRadiusCharacterization(drive)` | Characterization routine — measures effective wheel radius. |
| `feedforwardCharacterization(drive)` | Characterization routine — measures drive motor kS/kV. |
| `sysIdQuasistatic(drive, direction)` | SysId quasistatic sweep. |
| `sysIdDynamic(drive, direction)` | SysId dynamic step. |

SysId routines are present but commented out of the auto chooser.

---

## FlywheelCommands.java

Requires: `Flywheel`

| Method | Description |
|--------|-------------|
| `setFlywheelVoltage(flywheel, voltage)` | Direct voltage output. |
| `setFlywheelVelocity(flywheel, rpmSupplier)` | Velocity closed-loop. |
| `flywheelIdle(flywheel)` | Runs at idle RPM. Suitable as a background command. |
| `setVelocityForHub(flywheel)` | Reads distance from `RobotState`, looks up RPM in `ShotCalculator`. |
| `setVelocityForPassing(flywheel)` | Fixed pass velocity from `HardwareConstants.PassConstants`. |
| `stop(flywheel)` | Stops flywheel (sets voltage to 0). |

---

## HoodCommands.java

Requires: `Hood`

| Method | Description |
|--------|-------------|
| `setHoodPos(hood, angle)` | Set to explicit angle. |
| `setHoodPosForHub(hood)` | Distance-based angle from `HoodPosCalculator`. |
| `setPosForPassing(hood)` | Fixed pass angle (35°). |
| `hoodIdle(hood)` | Default command — holds position, prevents stale PID reference. |
| `stowHood(hood)` | Moves to 0° (down position). |
| `incrementHoodPos(hood)` | Increases angle by 5°. |

---

## PrestageCommands.java

Requires: `Prestage`

| Method | Description |
|--------|-------------|
| `setPrestageVelocity(prestage, rpm)` | Velocity closed-loop. |
| `stop(prestage)` | Stop prestage. |
| `prestageIdle(prestage)` | Runs at idle RPM (1300 RPM). |

---

## FeederCommands.java

Requires: `UpperFeeder` and/or `LowerFeeder`

| Method | Description |
|--------|-------------|
| `setUpperFeederVelocity(feeder, rpm)` | Upper feeder velocity. |
| `setLowerFeederVelocity(feeder, rpm)` | Lower feeder velocity. |
| `setUpperVelocityAfterWait(feeder, rpm, seconds)` | Delay, then set velocity. |
| `setLowerVelocityAfterWait(feeder, rpm, seconds)` | Delay, then set velocity. |
| `stopUpper(feeder)` | Stop upper feeder. |
| `stopLower(feeder)` | Stop lower feeder. |

---

## TransportCommands.java

Requires: `Transport`

| Method | Description |
|--------|-------------|
| `setTransportVoltage(transport, volts)` | Direct voltage. |
| `setTransportVelocity(transport, rpm)` | Velocity closed-loop. |
| `setVelocityAfterWait(transport, rpm, seconds)` | Delay, then set velocity. |
| `stop(transport)` | Stop transport. |

---

## IntakePivotCommands.java

Requires: `IntakePivot`

| Method | Description |
|--------|-------------|
| `setPivotPosition(pivot, angle)` | Set pivot to explicit position. |
| `compressPivot(pivot, doubleCompress)` | Compress sequence. If `doubleCompress`, oscillates between jostle positions before stowing. |
| `manualPivotCompress(pivot)` | Manual compress without double-compress variant. |
| `autoPivotCompress(pivot)` | Used in autonomous — no double-compress, simpler sequence. |

---

## intakeRollerCommands.java

Requires: `intakeRoller`

| Method | Description |
|--------|-------------|
| `setRollerVoltage(roller, volts)` | Direct voltage. |
| `setVoltageAfterWait(roller, volts, seconds, alignedGate)` | Waits for `alignedGate` boolean OR timeout, then sets voltage. Used to delay roller start until pivot is in position. |
| `stopIntakeRoller(roller)` | Stop roller. |

---

## ShootSequences.java

Requires: `Flywheel`, `Hood`, `Prestage`, `UpperFeeder`, `LowerFeeder`, `Transport`

These are the highest-level shooting recipes. They combine multiple subsystem
commands via `Commands.parallel()` and `Commands.sequence()`.

### `autoShootToHub()`

Full autonomous shoot sequence:

```
parallel(
  setVelocityForHub(flywheel),          ← spin up flywheel
  setHoodPosForHub(hood),               ← set hood angle
  sequence(
    waitUntil(flywheel.isSpunUp)        ← wait for speed OR
        .withTimeout(spinupTimeout),
    parallel(                           ← feed all at once
      setPrestageVelocity(prestage),
      setUpperFeederVelocity(upperFeeder),
      setLowerFeederVelocity(lowerFeeder),
      setTransportVelocity(transport)
    )
  )
)
```

The flywheel and hood spin up in parallel with the sequence that waits for
spinup before engaging feeders. This prevents note feed-through before the
flywheel is at speed.

### `shootEndBehavior()`

Stop sequence after a shot with appropriate timing (allows last note to clear).

### `stopAll()`

Emergency stop — immediately zero all shooter subsystem voltages/velocities.
Also registered as the `stopAll` named command for PathPlanner event triggers.

---

## SpitSequences.java

Requires: `Flywheel`, `Prestage`, `UpperFeeder`, `LowerFeeder`, `Transport`

Complement to `ShootSequences` — runs all mechanisms in reverse to eject fuel.
Used for defensive scenarios or clearing jams.

---

## Command Binding (RobotContainer.java)

Commands are bound to triggers in `RobotContainer.configureButtonBindings()`.
Key bindings (see `Docs/hardware.md` for button assignments):

```java
// Shoot: align drive + spinup flywheel + compress intake
shootButton.and(isShootSafeZone)
    .whileTrue(DriveCommands.alignOrXForShoot(drive, ...));

shootButton.and(isShootClear)
    .whileTrue(
        FlywheelCommands.setVelocityForHub(flywheel)
            .alongWith(PrestageCommands.setPrestageVelocity(...))
    )
    .onFalse(FlywheelCommands.stop(flywheel));

// Auto-compress when flywheel is up
shootButton.and(() -> !compressCancelled)
    .whileTrue(
        Commands.sequence(
            Commands.waitUntil(flywheel.isFlywheelSpunUp).withTimeout(0.5),
            IntakePivotCommands.compressPivot(intakePivot, () -> doubleCompress)
        )
    );
```

### State Flags

| Flag | Meaning | Reset condition |
|------|---------|-----------------|
| `compressCancelled` | Driver overrode auto-compress | Shoot button released |
| `xCancelled` | Driver overrode auto-X | Shoot button released |
| `doubleCompress` | Toggle double-compress mode | Button press toggles |

---

## Event Triggers (PathPlanner)

Named commands and event triggers are registered in `RobotContainer`:

```java
NamedCommands.registerCommand("DeployIntake", ...);
NamedCommands.registerCommand("RetractIntake", ...);
NamedCommands.registerCommand("RunIntake", ...);
NamedCommands.registerCommand("Shoot", ...);
NamedCommands.registerCommand("stopAll", ...);
NamedCommands.registerCommand("HoodDownNamed", ...);

new EventTrigger("DeployIntake").onTrue(
    Commands.runOnce(() -> intakePivot.setPivotPosition(...))
    // No subsystem argument — avoids scheduler conflict with auto group
);
```

Event trigger commands must not declare requirements that overlap with the auto
group. Passing `Commands.runOnce(() -> method())` (without the subsystem
argument) calls the method without declaring a requirement, preventing the
scheduler from interrupting the auto to resolve a conflict.
