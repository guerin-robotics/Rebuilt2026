# Subsystem Ownership

This document defines which files belong to each subsystem and 
the rules for modifying them.

---

## Ownership Model

Each subsystem is a vertical slice. A change to one subsystem should 
**never require editing another subsystem's files.**

If a fix requires touching two subsystems, that is a design smell.
Surface it and fix the coupling rather than patching across boundaries.

---

## Drive

**Owner files:**
```
subsystems/drive/Drive.java
subsystems/drive/Module.java
subsystems/drive/GyroIO.java
subsystems/drive/GyroIOPigeon2.java
subsystems/drive/GyroIOSim.java
subsystems/drive/ModuleIO.java
subsystems/drive/ModuleIOTalonFX.java
subsystems/drive/ModuleIOSim.java
subsystems/drive/PhoenixOdometryThread.java
commands/DriveCommands.java
generated/COMP_TunerConstants.java
generated/ALPHA_TunerConstants.java
generated/TunerConstants.java
```

**Permitted cross-boundary interactions:**
- `Drive` calls `RobotState.getInstance().setPoseSupplier()` at construction — this is the designed wiring point
- `Vision` calls `drive.addVisionMeasurement()` via a consumer passed at construction
- `AutoBuilder.configure()` in `Drive.java` wires PathPlanner — this is the correct location

**High-risk files:** All of them. Drive changes are Level 3+.

---

## Vision

**Owner files:**
```
subsystems/vision/Vision.java
subsystems/vision/VisionIO.java
subsystems/vision/VisionIOPhotonVision.java
subsystems/vision/VisionIOPhotonVisionSim.java
subsystems/vision/VisionConstants.java
```

**Permitted cross-boundary interactions:**
- `Vision` calls a `Consumer<VisionMeasurement>` passed at construction (wired to `drive::addVisionMeasurement` in `RobotContainer`)

**Known debt:** `maxPoseJumpMeters` filter disabled at `Vision.java:140`. Do not re-enable without testing against real logs.

---

## Flywheel

**Owner files:**
```
subsystems/flywheel/Flywheel.java
subsystems/flywheel/FlywheelIO.java (+ IOInputs, Real, Sim)
subsystems/flywheel/ShotCalculator.java
subsystems/flywheel/FlywheelVisualizer.java
commands/FlywheelCommands.java
```

**Permitted cross-boundary interactions:**
- `Flywheel` uses `hoodAngleSupplier` (a `DoubleSupplier` passed at construction) — decouples from Hood

---

## Hood

**Owner files:**
```
subsystems/hood/Hood.java
subsystems/hood/HoodIO.java (+ IOInputs, Real, Sim)
subsystems/hood/HoodPosCalculator.java
commands/HoodCommands.java
```

---

## Intake Pivot

**Owner files:**
```
subsystems/intakePivot/IntakePivot.java
subsystems/intakePivot/IntakePivotIO.java (+ IOInputs, Real, Sim)
subsystems/intakePivot/IntakePivotVisualizer.java
commands/IntakePivotCommands.java
util/ContinuousConditionalCommand.java  ← used here and potentially elsewhere
```

---

## Intake Roller

**Owner files:**
```
subsystems/intakeRoller/intakeRoller.java    ← note: lowercase-i (known debt)
subsystems/intakeRoller/intakeRollerIO.java (+ IOInputs, Real, Sim)
commands/intakeRollerCommands.java           ← lowercase-i (known debt)
```

---

## Prestage

**Owner files:**
```
subsystems/prestage/Prestage.java
subsystems/prestage/PrestageIO.java (+ IOInputs, Real, Sim)
commands/PrestageCommands.java
```

---

## Transport

**Owner files:**
```
subsystems/transport/Transport.java
subsystems/transport/io/TransportIO.java (+ IOInputs, Real, Sim)
commands/TransportCommands.java
```

---

## Upper / Lower Feeder

**Owner files:**
```
subsystems/upperFeeder/UpperFeeder.java
subsystems/upperFeeder/io/UpperFeederIO.java (+ IOInputs, Real, Sim)
subsystems/lowerFeeder/LowerFeeder.java
subsystems/lowerFeeder/io/LowerFeederIO.java (+ IOInputs, Real, Sim)
commands/FeederCommands.java               ← shared command file for both feeders
```

---

## Integration Layer (not owned by any single subsystem)

These files wire subsystems together. They are the only place cross-subsystem 
logic is permitted.

```
RobotContainer.java      — subsystem wiring + command binding
ShootSequences.java      — composed shoot pipeline
SpitSequences.java       — composed spit/reject pipeline
RobotState.java          — shared field geometry (singleton)
Triggers.java            — shared button/state trigger objects (singleton)
```

**Rule for `RobotContainer`:** Wiring only. If you find yourself writing 
if-statements or game logic inside `RobotContainer`, it belongs in a command factory 
or `RobotState`.

---

## Utility (shared, no owner)

```
util/AllianceFlipUtil.java
util/BatteryLogger.java
util/CANUpdateThread.java
util/ContinuousConditionalCommand.java
util/Elastic.java
util/HubShiftUtil.java       ← game-specific; replace each season
util/LocalADStarAK.java
util/LoggedTrigger.java
util/PhoenixUtil.java
```
