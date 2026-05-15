# Data Flow

All flows confirmed by source reading. References include file:line.

---

## 1. Sensor Data Flow (Per 20ms Cycle)

### Drive Sensor Path

```
Hardware registers (TalonFX, CANcoder, Pigeon2)
  │
  │  [HIGH-FREQUENCY THREAD — 250 Hz on CAN FD, 100 Hz on standard CAN]
  ▼
PhoenixOdometryThread.run()
  ├── Acquires odometryLock (Drive.java:85, ReentrantLock)
  ├── Reads drive velocity signals from all 8 TalonFX motors
  ├── Reads turn position signals from all 4 CANcoders
  ├── Reads gyro yaw signal from Pigeon2
  ├── Appends timestamped samples to per-module queues
  └── Releases odometryLock
  │
  │  [MAIN LOOP — 50 Hz]
  ▼
Drive.periodic()
  ├── Acquires odometryLock
  ├── io.updateInputs(gyroInputs)          ← GyroIOInputsAutoLogged
  ├── for each module: io.updateInputs()   ← ModuleIOInputsAutoLogged
  ├── Logger.processInputs("Drive/Gyro", gyroInputs)
  ├── Logger.processInputs("Drive/Module[i]", moduleInputs[i])
  ├── Drains high-freq odometry queues
  ├── for each timestamp sample:
  │     poseEstimator.updateWithTime(ts, gyroAngle, modulePositions)
  ├── Releases odometryLock
  ├── RobotState.setModuleStates(moduleStates)  ← Drive.java:216
  └── Logger.recordOutput("Drive/EstimatedPose", getPose())
```

### Vision Sensor Path

```
PhotonVision coprocessor (network, ~20ms pipeline latency)
  │
  ▼
VisionIOPhotonVision.updateInputs()
  ├── photonCamera.getLatestResult()
  ├── Copies targets, timestamps, pose estimates into VisionIOInputs
  └── Inputs ready for main-loop consumption
  │
  ▼
Vision.periodic()
  ├── io[i].updateInputs(inputs[i])                 for each camera
  ├── Logger.processInputs("Vision/Camera[i]", ...)
  ├── PRE-FILTER: check RobotState.getFieldRelativeVelocity().omegaRadiansPerSecond
  │     if > maxAngularVelocityRadPerSec → skip ALL cameras this cycle
  ├── for each camera × each observation:
  │     REJECT if: tagCount == 0
  │     REJECT if: single tag AND ambiguity > maxAmbiguity
  │     REJECT if: Z < −floorError OR Z > maxZError
  │     REJECT if: averageTagDistance > maxDistanceMeters
  │     REJECT if: pitch > maxPitchRollRadians
  │     REJECT if: roll > maxPitchRollRadians
  │     REJECT if: pose outside field boundary
  │     COMPUTE stdDevFactor = distance² / tagCount
  │     COMPUTE linearStdDev = baseline × factor × [MegaTag2 mult] × [single-tag mult] × [camera factor]
  │     COMPUTE angularStdDev = same pattern
  │     consumer.accept(pose2d, timestampSeconds, stdDevMatrix)
  │         └── drive::addVisionMeasurement
  │               └── poseEstimator.addVisionMeasurement(pose2d, ts, stdDevs)
  └── Logger.recordOutput("Vision/EstimatedPoses", ...)
```

### Shooter Sensor Path

```
TalonFX integrated encoders (leader + followers)
  │
  ▼
FlywheelIOPhoenix6.updateInputs()
  ├── Reads leaderVelocity, followerVelocities
  ├── Reads applied voltage, supply current, stator current
  └── Reads temperature

  ▼
Flywheel.periodic()
  ├── io.updateInputs(inputs)
  ├── Logger.processInputs("Flywheel", inputs)
  ├── Logger.recordOutput("Flywheel/IsSpunUp", isSpunUp())
  └── hoodAngleSupplier read (from Hood.getPosition()) for visualizer
```

### Intake Pivot Sensor Path

```
CANcoder ID 44 (absolute)
  │
  ▼
IntakePivotIOReal.updateInputs()
  ├── Reads position (rotations), velocity
  ├── Reads applied voltage, supply current
  │
  ▼
IntakePivot.periodic()
  ├── io.updateInputs(inputs)
  ├── Logger.processInputs("IntakePivot", inputs)
  └── IntakePivotVisualizer.update(inputs.position, goalPosition)
```

---

## 2. Pose Estimation Data Flow

```
Source 1: Wheel Odometry (250 Hz via PhoenixOdometryThread)
  Drive velocity + turn angles → SwerveModulePosition deltas

Source 2: Gyro (250 Hz)
  Pigeon2 yaw → Rotation2d

Source 3: Vision (async, ~20ms latency, 4 cameras)
  AprilTag pose estimates → Pose2d + timestamp + stdDev matrix

                        ▼
          SwerveDrivePoseEstimator
            (WPILib Kalman filter)
          ├── updateWithTime(ts, gyro, modulePositions)
          │     Called for each high-freq sample (up to 5× per 20ms cycle)
          └── addVisionMeasurement(pose, ts, stdDevs)
                Called from Vision.periodic() for each accepted observation
                Retroactively applies to correct timestamp in internal buffer

                        ▼
                 Drive.getPose()
                        │
                        ▼
           RobotState.poseSupplier.get()
                        │
             ┌──────────┴──────────┐
             ▼                     ▼
     getDistanceToHub()     getSpeakerAngleRad()
             │                     │
             ▼                     ▼
     ShotCalculator         DriveCommands
     HoodPosCalculator      (heading target)
```

**Timestamp handling:** Vision observations include the PhotonVision pipeline
timestamp. `addVisionMeasurement` uses this to insert the measurement at the
correct point in the pose estimator's internal history buffer, rather than
applying it at the current cycle time. This corrects for network latency.

---

## 3. Shot Calculation Data Flow

```
RobotState.getEstimatedPose()
        │
        ▼
distance = pose.getTranslation()
           .getDistance(FieldConstants.hubPosition)
   (with AllianceFlipUtil.apply() if red alliance)
        │
   ┌────┴────┐
   ▼         ▼
ShotCalculator   HoodPosCalculator
  (distance→RPM)   (distance→degrees)
   │              │
   ▼              ▼
flywheel.setVelocity()   hood.setPosition()
```

Both calculators use linear interpolation between hand-tuned data points.

---

## 4. Logging and Telemetry Flow

### AdvantageKit Logging Pipeline

```
All subsystem periodic() calls:
  io.updateInputs(inputs)              ← hardware reads
  Logger.processInputs("Key", inputs)  ← AKit logs all @AutoLog fields
                │
                ▼
  WPILOGWriter → USB drive (real robot)
  NT4Publisher → NetworkTables (dashboard)
```

### Manual Log Points

```java
// Drive.java
Logger.recordOutput("Drive/EstimatedPose", getPose());
Logger.recordOutput("Drive/ActivePath", ...);          // while path following
Logger.recordOutput("Drive/TargetPose", ...);          // while path following

// Flywheel.java
Logger.recordOutput("Flywheel/IsSpunUp", isSpunUp());
Logger.recordOutput("Flywheel/currentRPMTarget", currentRPMTarget);

// Vision.java
Logger.recordOutput("Vision/EstimatedPoses[i]", ...);
Logger.recordOutput("Vision/TagIds[i]", ...);

// RobotState.java (@AutoLogOutput annotations — auto-registered)
@AutoLogOutput(key = "RobotState/EstimatedPose")
public Pose2d getEstimatedPose() { ... }

@AutoLogOutput(key = "RobotState/DistanceToHub")
public double getDistanceToHub() { ... }

// Auto checks
Logger.recordOutput("Auto/StartCheck/DistanceInches", ...);
Logger.recordOutput("Auto/StartCheck/PositionOK", ...);
```

### Dashboard Values

| SmartDashboard Key | Content | Direction |
|-------------------|---------|-----------|
| `autoDelayKey` | Autonomous delay seconds | Read by robot code |
| Auto chooser | Selected auto name | Read by robot code |
| Battery voltage | Voltage reading | Written by robot code |
| Auto preview Field2d | Path waypoints | Written by robot code |

### NetworkTable Tunables

```
LoggedNetworkNumber "Tune/flywheel/tuningRPM"
  ├── Default: 20 RPM
  ├── Editable from Shuffleboard during testing
  ├── Logged by AdvantageKit for replay
  └── Active only when TUNING_MODE == true
```

---

## 5. CAN Device Inventory

All CAN IDs. Phoenix 6 bus unless noted.

### Drive (from TunerConstants.java — auto-generated)

Module-level IDs are in `generated/TunerConstants.java`. Bus may be CAN FD
depending on `TunerConstants.kCANBus`.

| Module | Drive TalonFX | Turn TalonFX | CANcoder |
|--------|--------------|-------------|---------|
| Front Left | See TunerConstants | See TunerConstants | See TunerConstants |
| Front Right | See TunerConstants | See TunerConstants | See TunerConstants |
| Back Left | See TunerConstants | See TunerConstants | See TunerConstants |
| Back Right | See TunerConstants | See TunerConstants | See TunerConstants |

Gyro: Pigeon 2 (ID in TunerConstants or GyroIOPigeon2 constructor)

### Shooter/Feeder (from HardwareConstants.java)

| ID | Device | Subsystem |
|----|--------|-----------|
| 30 | TalonFX — Flywheel leader | Flywheel |
| 31 | TalonFX — Flywheel follower 1 | Flywheel |
| 32 | TalonFX — Flywheel follower 2 | Flywheel |
| 33 | TalonFX — Flywheel follower 3 | Flywheel |
| 34 | TalonFX — Flywheel follower 4 | Flywheel |
| 35 | TalonFX — Hood motor | Hood |
| 36 | TalonFX — Upper feeder | UpperFeeder |
| 37 | TalonFX — Prestage follower | Prestage |
| 38 | TalonFX — Prestage leader | Prestage |
| 39 | TalonFX — Lower feeder | LowerFeeder |
| 40 | TalonFX — Transport | Transport |
| 41 | TalonFX — Intake pivot | IntakePivot |
| 42 | TalonFX — Intake roller leader | intakeRoller |
| 43 | TalonFX — Intake roller follower | intakeRoller |
| 44 | CANcoder — Intake pivot encoder | IntakePivot |
| 50 | CANcoder — Hood encoder | Hood |

**Total CAN devices (excluding drive modules):** 17 (15 TalonFX + 2 CANcoder)
**Drive modules add:** 8 TalonFX + 4 CANcoder + 1 Pigeon2 = 13 devices

**Total CAN bus load estimate:** ~30 devices. With Phoenix 6 on CAN FD this is
well within capacity. On standard 1 Mbit/s CAN this is heavy but manageable
with Phoenix 6's optimized signal multiplexing.

---

## 6. Cross-Subsystem Data Consumers

Which subsystems/components read data from which sources:

| Consumer | Reads From | Data |
|----------|-----------|------|
| Vision.periodic() | RobotState | `getFieldRelativeVelocity().omegaRadiansPerSecond` |
| FlywheelCommands | RobotState | `getDistanceToHub()` → ShotCalculator |
| HoodCommands | RobotState | `getDistanceToHub()` → HoodPosCalculator |
| DriveCommands | RobotState | `getSpeakerAngleRad()`, `getPassAngleRad()` |
| Triggers | RobotState | `getBroadZone()`, `isAlignedToHub()`, `isAlignedToPass()` |
| Flywheel visualizer | Hood | `getPosition()` via supplier |
| IntakePivot visualizer | IntakePivot | `inputs.position`, `goalPosition` |
| PathPlanner | Drive | `getPose()`, `getChassisSpeeds()` |
| Auto preview | PathPlanner | Path waypoints (read from `.path` files) |
