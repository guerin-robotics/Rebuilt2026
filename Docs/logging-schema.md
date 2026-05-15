# AdvantageKit Logging Schema

Complete map of every logged signal in the Rebuilt2026 robot codebase. All signals verified against source files.

---

## 1. Logging Architecture Overview

### Framework
The robot uses **AdvantageKit** (`org.littletonrobotics.junction`), built on top of WPILib's `LoggedRobot`. All I/O flows through `Logger`, which is initialized in `Robot.java` before `RobotContainer` is constructed.

### Data Receivers (Robot.java:62-81)
| Mode | Receivers |
|------|-----------|
| `REAL` | `WPILOGWriter` (USB stick at `/U/logs`) + `NT4Publisher` |
| `SIM` | `NT4Publisher` only |
| `REPLAY` | `WPILOGReader` (source) + `WPILOGWriter` (output `_sim` suffix), timing disabled |

### Three Logging Mechanisms
1. **`Logger.processInputs(prefix, inputs)`** — serializes every field of an `@AutoLog`-annotated `*IOInputs` class under the given prefix. These are the replay-critical signals — every IO input must go through this path.
2. **`Logger.recordOutput(key, value)`** — logs derived/computed values, state strings, and visualization data. Not required for replay but useful for analysis.
3. **`@AutoLogOutput(key = "...")`** — method annotation scanned by `AutoLogOutputManager`. Every annotated method is called each loop and its return value is recorded. `RobotState` must be manually registered via `AutoLogOutputManager.addObject(RobotState.getInstance())` because it is a singleton not reachable through the normal object graph (Robot.java:90).

### `@AutoLog` Annotation
Placed on inner `*IOInputs` classes inside IO interfaces. The AKit annotation processor generates `*IOInputsAutoLogged` with serialization logic. Every field becomes a logged signal under the `processInputs` prefix.

---

## 2. Signal Naming Conventions

| Pattern | Example | Meaning |
|---------|---------|---------|
| `Subsystem/field` | `Drive/Gyro/yawPosition` | IO input from processInputs |
| `SubsystemGroup/Subsystem/field` | `Feeder/Upper/upperFeederVoltage` | Nested grouping |
| `SwerveStates/Setpoints` | — | Drive output arrays |
| `SwerveChassisSpeeds/Measured` | — | Velocity derived from module states |
| `Odometry/Robot` | — | Pose from the pose estimator |
| `Vision/Camera{i}/...` | `Vision/Camera0/TagPoses` | Per-camera vision outputs |
| `Vision/Summary/...` | — | Aggregated cross-camera vision |
| `RobotState/...` | `RobotState/EstimatedPose` | @AutoLogOutput from RobotState singleton |
| `AutoAim/...` | `AutoAim/TargetAngle` | Command-level heading control |
| `Auto/...` | `Auto/SelectedAuto` | Autonomous path selection and verification |
| `BatteryLogger/...` | `BatteryLogger/Current` | Aggregated power data |
| `Flywheel/Visualizer/...` | — | 3D trajectory visualization |
| `{Name}/Visualizer/...` | `Intake Pivot/Visualizer/Pose3d` | Mechanism 2D/3D visualization |

**Prefix used in `processInputs`** becomes the parent directory in the log. Every field in the `@AutoLog` class appears as `Prefix/fieldName`.

**Known inconsistency:** `intakeRollerIO` uses lowercase class/interface names, breaking Java naming conventions. The log prefix is `"Intake Roller"` (with a space), while all other subsystems use CamelCase prefixes or slashes with no spaces.

---

## 3. Drivetrain Signals

### 3a. Gyro — `Logger.processInputs("Drive/Gyro", gyroInputs)` (Drive.java:165)

Source: `src/main/java/frc/robot/subsystems/drive/GyroIO.java`

| Field | Log Key | Type | Purpose | Frequency |
|-------|---------|------|---------|-----------|
| `connected` | `Drive/Gyro/connected` | `boolean` | Pigeon 2 connection status | 50 Hz |
| `yawPosition` | `Drive/Gyro/yawPosition` | `Rotation2d` | Robot heading used for odometry | 50 Hz |
| `yawVelocityRadPerSec` | `Drive/Gyro/yawVelocityRadPerSec` | `double` | Turn rate — used in vision spin rejection filter | 50 Hz |
| `odometryYawTimestamps` | `Drive/Gyro/odometryYawTimestamps` | `double[]` | High-frequency odometry timestamps (Phoenix 250 Hz thread) | Up to 250 Hz |
| `odometryYawPositions` | `Drive/Gyro/odometryYawPositions` | `Rotation2d[]` | Yaw snapshots matching odometry timestamps | Up to 250 Hz |

Implementations: `GyroIOPigeon2.java` (real), `GyroIONavX.java` (NavX fallback).
**Replay-critical:** Yes — all fields required to reconstruct odometry.

### 3b. Swerve Modules — `Logger.processInputs("Drive/Module{0-3}", inputs)` (Module.java:57)

Four instances: 0 = FrontLeft, 1 = FrontRight, 2 = BackLeft, 3 = BackRight.
Source: `src/main/java/frc/robot/subsystems/drive/ModuleIO.java`

| Field | Log Key Pattern | Type | Purpose | Frequency |
|-------|-----------------|------|---------|-----------|
| `driveConnected` | `Drive/Module{i}/driveConnected` | `boolean` | TalonFX drive motor connection status | 50 Hz |
| `drivePositionRad` | `Drive/Module{i}/drivePositionRad` | `double` | Cumulative wheel position in radians | 50 Hz |
| `driveVelocityRadPerSec` | `Drive/Module{i}/driveVelocityRadPerSec` | `double` | Wheel velocity — compare to setpoint to detect slip | 50 Hz |
| `driveAppliedVolts` | `Drive/Module{i}/driveAppliedVolts` | `double` | Drive motor output voltage | 50 Hz |
| `driveCurrentAmps` | `Drive/Module{i}/driveCurrentAmps` | `double` | Drive supply current (also fed to BatteryLogger) | 50 Hz |
| `turnConnected` | `Drive/Module{i}/turnConnected` | `boolean` | TalonFX turn motor connection status | 50 Hz |
| `turnEncoderConnected` | `Drive/Module{i}/turnEncoderConnected` | `boolean` | CANcoder absolute encoder connection status | 50 Hz |
| `turnAbsolutePosition` | `Drive/Module{i}/turnAbsolutePosition` | `Rotation2d` | CANcoder absolute angle — used for steering zero | 50 Hz |
| `turnPosition` | `Drive/Module{i}/turnPosition` | `Rotation2d` | Relative encoder position — actual steering angle | 50 Hz |
| `turnVelocityRadPerSec` | `Drive/Module{i}/turnVelocityRadPerSec` | `double` | Steering angular rate (rad/s) | 50 Hz |
| `turnAppliedVolts` | `Drive/Module{i}/turnAppliedVolts` | `double` | Turn motor output voltage | 50 Hz |
| `turnCurrentAmps` | `Drive/Module{i}/turnCurrentAmps` | `double` | Turn supply current (also fed to BatteryLogger) | 50 Hz |
| `odometryTimestamps` | `Drive/Module{i}/odometryTimestamps` | `double[]` | Phoenix high-frequency odometry timestamps | Up to 250 Hz |
| `odometryDrivePositionsRad` | `Drive/Module{i}/odometryDrivePositionsRad` | `double[]` | Drive position snapshots for high-freq odometry | Up to 250 Hz |
| `odometryTurnPositions` | `Drive/Module{i}/odometryTurnPositions` | `Rotation2d[]` | Turn angle snapshots for high-freq odometry | Up to 250 Hz |

Implementations: `ModuleIOTalonFX.java` (COMP), `ModuleIOTalonFXS.java` (ALPHA), `ModuleIOSim.java`.
**Replay-critical:** Yes — all fields, especially the `odometry*` arrays from `PhoenixOdometryThread.java`.

### 3c. Drive @AutoLogOutput Signals (Drive.java)

| Signal | Log Key | Type | Line | Purpose |
|--------|---------|------|------|---------|
| Module states measured | `SwerveStates/Measured` | `SwerveModuleState[]` | 292 | Actual wheel velocity + angle per module |
| Chassis speeds measured | `SwerveChassisSpeeds/Measured` | `ChassisSpeeds` | 311 | Robot-frame velocity from measured module states |
| Odometry pose | `Odometry/Robot` | `Pose2d` | 335 | Fused pose estimator output (wheels + vision) |

### 3d. Drive recordOutput Signals (Drive.java)

| Signal | Log Key | Type | Line | Purpose |
|--------|---------|------|------|---------|
| Module setpoints (pre-optimize) | `SwerveStates/Setpoints` | `SwerveModuleState[]` | 180, 242 | Commanded wheel velocity + angle before optimization |
| Module setpoints (post-optimize) | `SwerveStates/SetpointsOptimized` | `SwerveModuleState[]` | 181, 251 | Commanded states after angle flip optimization |
| Chassis speed setpoints | `SwerveChassisSpeeds/Setpoints` | `ChassisSpeeds` | 243 | Desired chassis speeds (discretized for dt) |
| Path trajectory | `Odometry/Trajectory` | `Pose2d[]` | 137 | Active PathPlanner path for AdvantageScope field view |
| Path target pose | `Odometry/TrajectorySetpoint` | `Pose2d` | 141 | Current target waypoint on the active path |
| SysId state | `Drive/SysIdState` | `String` | 151 | Characterization routine phase |

**Key analysis comparison:** `SwerveStates/Setpoints` vs `SwerveStates/Measured` — gap indicates slip or steering lag. `Odometry/Trajectory` + `Odometry/TrajectorySetpoint` vs `Odometry/Robot` indicates path-following error.

---

## 4. Vision Signals

### 4a. Per-Camera IO Inputs — `Logger.processInputs("Vision/Camera{i}", inputs[i])` (Vision.java:85)

Four cameras (indices 0–3). Source: `src/main/java/frc/robot/subsystems/vision/io/VisionIO.java`

| Field | Log Key Pattern | Type | Purpose |
|-------|-----------------|------|---------|
| `connected` | `Vision/Camera{i}/connected` | `boolean` | PhotonVision camera connection status |
| `latestTargetObservation` | `Vision/Camera{i}/latestTargetObservation` | `TargetObservation` | Record: `tx` (Rotation2d) + `ty` (Rotation2d) for simple target tracking |
| `poseObservations` | `Vision/Camera{i}/poseObservations` | `PoseObservation[]` | Array of pose estimates: timestamp, pose, ambiguity, tagCount, averageTagDistance, type (MEGATAG_1/MEGATAG_2/PHOTONVISION) |
| `tagIds` | `Vision/Camera{i}/tagIds` | `int[]` | AprilTag IDs seen this frame |

Implementation: `VisionIOPhotonVision.java`

### 4b. Per-Camera recordOutput Signals

| Signal | Log Key Pattern | Type | File:Line | Purpose |
|--------|-----------------|------|-----------|---------|
| Multi-tag used | `Vision/{cameraName}/UsedMultiTag` | `boolean` | VisionIOPhotonVision.java:89 | Whether MegaTag 2 multi-tag solve was used (keyed by camera name string, not index) |
| Observation ambiguity | `Vision/{cameraName}/Ambiguity` | `double` | VisionIOPhotonVision.java:117 | Per-observation pose ambiguity (<0.3 accepted) |
| Detected tag poses (3D) | `Vision/Camera{i}/TagPoses` | `Pose3d[]` | Vision.java:189 | Field positions of detected tags for viz |
| All robot pose estimates | `Vision/Camera{i}/RobotPoses` | `Pose3d[]` | Vision.java:190 | All candidates before filtering |
| Accepted pose estimates | `Vision/Camera{i}/RobotPosesAccepted` | `Pose3d[]` | Vision.java:191 | Poses that passed all rejection filters |
| Rejected pose estimates | `Vision/Camera{i}/RobotPosesRejected` | `Pose3d[]` | Vision.java:193 | Poses that failed rejection filters |
| Tag count | `Vision/Camera{i}/TagCount` | `int` | Vision.java:195 | Number of tags detected |
| Is multi-tag | `Vision/Camera{i}/IsMultiTag` | `boolean` | Vision.java:196 | True when >1 tag visible |

### 4c. Cross-Camera Summary (Vision.java)

| Signal | Log Key | Type | Line | Purpose |
|--------|---------|------|------|---------|
| All tag poses | `Vision/Summary/TagPoses` | `Pose3d[]` | 204 | Union of all detected tags across cameras |
| All robot poses | `Vision/Summary/RobotPoses` | `Pose3d[]` | 205 | All candidate poses across cameras |
| All accepted poses | `Vision/Summary/RobotPosesAccepted` | `Pose3d[]` | 206 | All poses fused into estimator |
| All rejected poses | `Vision/Summary/RobotPosesRejected` | `Pose3d[]` | 208 | All rejected poses for diagnostics |

### 4d. Camera Configuration (VisionConstants.java — logged once at static init)

| Signal | Log Key | Type | Purpose |
|--------|---------|------|---------|
| Camera 0 name | `Vision/Camera0/name` | `String` | Physical camera name |
| Camera 0 transform | `Vision/Camera0/robot_position` | `Transform3d` | Camera-to-robot extrinsic transform |
| Camera 1 name | `Vision/Camera1/name` | `String` | — |
| Camera 1 transform | `Vision/Camera1/robot_position` | `Transform3d` | — |
| Camera 2 name | `Vision/Camera2/name` | `String` | — |
| Camera 2 transform | `Vision/Camera2/robot_position` | `Transform3d` | — |
| Camera 3 name | `Vision/Camera3/name` | `String` | — |
| Camera 3 transform | `Vision/Camera3/robot_position` | `Transform3d` | — |

**Rejection filter logic (Vision.java, not logged, determines Accepted vs Rejected):**
- Robot angular velocity > `maxAngularVelocityRadPerSec` → reject all observations this cycle
- tagCount == 0
- Single-tag ambiguity > `maxAmbiguity`
- Pose Z outside `[-floorError, maxZError]`
- `averageTagDistance` > `maxDistanceMeters`
- Pitch or roll > `maxPitchRollRadians`
- Pose outside field boundaries

---

## 5. Mechanism Signals

### 5a. Flywheel — `Logger.processInputs("Flywheel", inputs)` (Flywheel.java:68)

Source: `src/main/java/frc/robot/subsystems/flywheel/io/FlywheelIO.java`

| Field | Log Key | Type | Purpose |
|-------|---------|------|---------|
| `flywheelVelocity` | `Flywheel/flywheelVelocity` | `AngularVelocity` | Combined flywheel speed — compared to setpoint for shoot-ready check |
| `closedLoopError` | `Flywheel/closedLoopError` | `AngularVelocity` | Velocity error from setpoint |
| `closedLoopReference` | `Flywheel/closedLoopReference` | `AngularVelocity` | Active velocity setpoint from TalonFX closed-loop |
| `leaderVelocity` | `Flywheel/leaderVelocity` | `AngularVelocity` | Leader motor velocity |
| `leaderAppliedVolts` | `Flywheel/leaderAppliedVolts` | `Voltage` | Leader output voltage |
| `leaderSupplyCurrentAmps` | `Flywheel/leaderSupplyCurrentAmps` | `Current` | Leader supply current |
| `leaderStatorCurrentAmps` | `Flywheel/leaderStatorCurrentAmps` | `Current` | Leader stator current (torque proxy) |
| `leaderTemp` | `Flywheel/leaderTemp` | `Temperature` | Leader motor temperature |
| `leaderAngle` | `Flywheel/leaderAngle` | `Angle` | Leader cumulative position |
| `follower1Velocity` | `Flywheel/follower1Velocity` | `AngularVelocity` | Follower 1 velocity |
| `follower1AppliedVolts` | `Flywheel/follower1AppliedVolts` | `Voltage` | Follower 1 voltage |
| `follower1SupplyCurrentAmps` | `Flywheel/follower1SupplyCurrentAmps` | `Current` | Follower 1 supply current |
| `follower1StatorCurrentAmps` | `Flywheel/follower1StatorCurrentAmps` | `Current` | Follower 1 stator current |
| `follower1Temp` | `Flywheel/follower1Temp` | `Temperature` | Follower 1 temperature |
| `follower2Velocity` | `Flywheel/follower2Velocity` | `AngularVelocity` | Follower 2 velocity |
| `follower2AppliedVolts` | `Flywheel/follower2AppliedVolts` | `Voltage` | — |
| `follower2SupplyCurrentAmps` | `Flywheel/follower2SupplyCurrentAmps` | `Current` | — |
| `follower2StatorCurrentAmps` | `Flywheel/follower2StatorCurrentAmps` | `Current` | — |
| `follower2Temp` | `Flywheel/follower2Temp` | `Temperature` | — |
| `follower3Velocity` | `Flywheel/follower3Velocity` | `AngularVelocity` | Follower 3 velocity |
| `follower3AppliedVolts` | `Flywheel/follower3AppliedVolts` | `Voltage` | — |
| `follower3SupplyCurrentAmps` | `Flywheel/follower3SupplyCurrentAmps` | `Current` | — |
| `follower3StatorCurrentAmps` | `Flywheel/follower3StatorCurrentAmps` | `Current` | — |
| `follower3Temp` | `Flywheel/follower3Temp` | `Temperature` | — |
| `follower4Velocity` | `Flywheel/follower4Velocity` | `AngularVelocity` | Follower 4 velocity |
| `follower4AppliedVolts` | `Flywheel/follower4AppliedVolts` | `Voltage` | — |
| `follower4SupplyCurrentAmps` | `Flywheel/follower4SupplyCurrentAmps` | `Current` | — |
| `follower4StatorCurrentAmps` | `Flywheel/follower4StatorCurrentAmps` | `Current` | — |
| `follower4Temp` | `Flywheel/follower4Temp` | `Temperature` | — |

**recordOutput from Flywheel.java:**
| Signal | Log Key | Type | Line | Purpose |
|--------|---------|------|------|---------|
| Target RPM (in isAtSetpoint) | `Flywheel/targetRPM` | `double` | 105 | Logged every loop during setpoint check |
| Active RPM target (on set) | `Flywheel/currentRPMTarget` | `double` | 151 | Logged when a new velocity is commanded |
| Running velocity (IO impl) | `Flywheel running` | `AngularVelocity` | FlywheelIOPhoenix6.java:345 | **No namespace prefix** — appears at log root |

**Visualizer (FlywheelVisualizer.java):**
| Signal | Log Key | Type | Line | Purpose |
|--------|---------|------|------|---------|
| Launch origin | `Flywheel/Visualizer/LaunchOriginPose` | `Pose3d` | 115 | 3D pose of shooter exit point (static) |
| Ball trajectory | `Flywheel/Visualizer/Trajectory` | `Translation3d[]` | 119, 154 | Simulated ball flight path (empty when no shot) |

**NT-backed tuning object:**
| Object | NT Key | Type | Default |
|--------|--------|------|---------|
| `LoggedNetworkNumber tuningRPM` | `Tune/flywheel/tuningRPM` | `double` | 20.0 |

### 5b. Hood — `Logger.processInputs("Hood", inputs)` (Hood.java:27)

Source: `src/main/java/frc/robot/subsystems/hood/io/HoodIO.java`

| Field | Log Key | Type | Purpose |
|-------|---------|------|---------|
| `hoodVoltage` | `Hood/hoodVoltage` | `Voltage` | Applied voltage |
| `hoodSupplyCurrent` | `Hood/hoodSupplyCurrent` | `Current` | Supply current |
| `hoodStatorCurrent` | `Hood/hoodStatorCurrent` | `Current` | Stator current (torque proxy) |
| `hoodTemperature` | `Hood/hoodTemperature` | `Temperature` | Motor temperature |
| `hoodVelocity` | `Hood/hoodVelocity` | `AngularVelocity` | Pivot angular velocity (deg/s) |
| `hoodPosition` | `Hood/hoodPosition` | `Angle` | Current hood angle (degrees) |
| `hoodClosedLoopReference` | `Hood/hoodClosedLoopReference` | `Angle` | TalonFX position setpoint (degrees) |
| `hoodClosedLoopError` | `Hood/hoodClosedLoopError` | `Angle` | Position error from setpoint |

Implementations: `HoodIOReal.java`, `HoodIOSim.java`

### 5c. Intake Pivot — `Logger.processInputs("Intake Pivot", inputs)` (IntakePivot.java:39)

Source: `src/main/java/frc/robot/subsystems/intakePivot/io/IntakePivotIO.java`

| Field | Log Key | Type | Purpose |
|-------|---------|------|---------|
| `intakePivotVoltage` | `Intake Pivot/intakePivotVoltage` | `Voltage` | Applied voltage |
| `intakePivotSupplyCurrent` | `Intake Pivot/intakePivotSupplyCurrent` | `Current` | Supply current |
| `intakePivotStatorCurrent` | `Intake Pivot/intakePivotStatorCurrent` | `Current` | Stator current |
| `intakePivotTemperature` | `Intake Pivot/intakePivotTemperature` | `Temperature` | Motor temperature |
| `intakePivotVelocity` | `Intake Pivot/intakePivotVelocity` | `AngularVelocity` | Pivot velocity (RPS) |
| `intakePivotPosition` | `Intake Pivot/intakePivotPosition` | `Angle` | Current pivot angle (rotations) |
| `intakePivotClosedLoopReference` | `Intake Pivot/intakePivotClosedLoopReference` | `double` | Position setpoint (raw, unitless) |
| `intakePivotClosedLoopError` | `Intake Pivot/intakePivotClosedLoopError` | `double` | Position error (raw, unitless) |

**recordOutput from IntakePivotIOReal.java:199:**
| Signal | Log Key | Type | Purpose |
|--------|---------|------|---------|
| Torque control velocity | `Intake pivot torque controls` | `AngularVelocity` | **No namespace prefix** — appears at log root |

**Visualizer (IntakePivotVisualizer.java, prefix = "Intake Pivot"):**
| Signal | Log Key | Type | Line | Purpose |
|--------|---------|------|------|---------|
| Mechanism2d | `Intake Pivot/Visualizer/Mechanism2d` | `LoggedMechanism2d` | 140 | 2D mechanism diagram |
| 3D pivot pose | `Intake Pivot/Visualizer/Pose3d` | `Pose3d` | 156 | 3D visualization |
| Current angle | `Intake Pivot/Visualizer/CurrentAngle_deg` | `Angle` | 159 | Current pivot angle (degrees) |
| Goal angle | `Intake Pivot/Visualizer/GoalAngle_deg` | `Angle` | 160 | Target pivot angle (degrees) |
| At goal | `Intake Pivot/Visualizer/AtGoal` | `boolean` | 161 | Whether pivot is at goal position |

Implementations: `IntakePivotIOReal.java`, `IntakePivotIOSim.java`

### 5d. Intake Roller — `Logger.processInputs("Intake Roller", inputs)` (intakeRoller.java:25)

Source: `src/main/java/frc/robot/subsystems/intakeRoller/io/intakeRollerIO.java`

| Field | Log Key | Type | Purpose |
|-------|---------|------|---------|
| `intakeRollerVoltage` | `Intake Roller/intakeRollerVoltage` | `Voltage` | Leader motor voltage |
| `intakeRollerSupplyCurrent` | `Intake Roller/intakeRollerSupplyCurrent` | `Current` | Leader supply current |
| `intakeRollerStatorCurrent` | `Intake Roller/intakeRollerStatorCurrent` | `Current` | Leader stator current |
| `intakeRollerTemperature` | `Intake Roller/intakeRollerTemperature` | `Temperature` | Leader temperature |
| `intakeRollerVelocity` | `Intake Roller/intakeRollerVelocity` | `AngularVelocity` | Leader roller velocity (RPS) |
| `rollerClosedLoopReference` | `Intake Roller/rollerClosedLoopReference` | `AngularVelocity` | Leader velocity setpoint |
| `rollerClosedLoopError` | `Intake Roller/rollerClosedLoopError` | `AngularVelocity` | Leader velocity error |
| `rollerPos` | `Intake Roller/rollerPos` | `Angle` | Leader cumulative position |
| `intakeRollerFollowerVoltage` | `Intake Roller/intakeRollerFollowerVoltage` | `Voltage` | Follower motor voltage |
| `intakeRollerFollowerSupplyCurrent` | `Intake Roller/intakeRollerFollowerSupplyCurrent` | `Current` | Follower supply current |
| `intakeRollerFollowerStatorCurrent` | `Intake Roller/intakeRollerFollowerStatorCurrent` | `Current` | Follower stator current |
| `intakeRollerFollowerTemperature` | `Intake Roller/intakeRollerFollowerTemperature` | `Temperature` | Follower temperature |
| `intakeRollerFollowerVelocity` | `Intake Roller/intakeRollerFollowerVelocity` | `AngularVelocity` | Follower velocity (should match leader) |
| `rollerFollowerClosedLoopReference` | `Intake Roller/rollerFollowerClosedLoopReference` | `AngularVelocity` | Follower setpoint |
| `rollerFollowerClosedLoopError` | `Intake Roller/rollerFollowerClosedLoopError` | `AngularVelocity` | Follower error |
| `rollerFollowerPos` | `Intake Roller/rollerFollowerPos` | `Angle` | Follower cumulative position |

Implementations: `intakeRollerIOReal.java`, `intakeRollerIOSim.java`

### 5e. Transport — `Logger.processInputs("Transport", inputs)` (Transport.java:25)

Source: `src/main/java/frc/robot/subsystems/transport/io/TransportIO.java`

| Field | Log Key | Type | Purpose |
|-------|---------|------|---------|
| `TransportVoltage` | `Transport/TransportVoltage` | `Voltage` | Applied voltage (note: PascalCase — inconsistent) |
| `TransportStatorAmps` | `Transport/TransportStatorAmps` | `Current` | Stator current |
| `TransportSupplyAmps` | `Transport/TransportSupplyAmps` | `Current` | Supply current |
| `TransportMotorVelocity` | `Transport/TransportMotorVelocity` | `AngularVelocity` | Belt/roller velocity (RPS) |
| `TransportMotorTemperature` | `Transport/TransportMotorTemperature` | `Temperature` | Motor temperature |
| `transportClosedLoopReference` | `Transport/transportClosedLoopReference` | `AngularVelocity` | Velocity setpoint |
| `transportClosedLoopError` | `Transport/transportClosedLoopError` | `AngularVelocity` | Velocity error |
| `transportPos` | `Transport/transportPos` | `Angle` | Cumulative position |

Implementations: `TransportIOReal.java`, `TransportIOSim.java`

### 5f. Prestage — `Logger.processInputs("Prestage", inputs)` (Prestage.java:27)

Source: `src/main/java/frc/robot/subsystems/prestage/io/PrestageIO.java`

| Field | Log Key | Type | Purpose |
|-------|---------|------|---------|
| `prestageLeftVoltage` | `Prestage/prestageLeftVoltage` | `Voltage` | Left roller voltage |
| `prestageLeftStatorAmps` | `Prestage/prestageLeftStatorAmps` | `Current` | Left stator current |
| `prestageLeftSupplyAmps` | `Prestage/prestageLeftSupplyAmps` | `Current` | Left supply current |
| `prestageRightVoltage` | `Prestage/prestageRightVoltage` | `Voltage` | Right roller voltage |
| `prestageRightStatorAmps` | `Prestage/prestageRightStatorAmps` | `Current` | Right stator current |
| `prestageRightSupplyAmps` | `Prestage/prestageRightSupplyAmps` | `Current` | Right supply current |
| `prestageLeftVelocity` | `Prestage/prestageLeftVelocity` | `AngularVelocity` | Left roller velocity (RPS) |
| `prestageRightVelocity` | `Prestage/prestageRightVelocity` | `AngularVelocity` | Right roller velocity (RPS) |
| `prestageLeftTemperature` | `Prestage/prestageLeftTemperature` | `Temperature` | Left motor temperature |
| `prestageRightTemperature` | `Prestage/prestageRightTemperature` | `Temperature` | Right motor temperature |
| `prestageLeftClosedLoopReference` | `Prestage/prestageLeftClosedLoopReference` | `AngularVelocity` | Left velocity setpoint |
| `prestageRightClosedLoopReference` | `Prestage/prestageRightClosedLoopReference` | `AngularVelocity` | Right velocity setpoint |
| `prestageLeftClosedLoopError` | `Prestage/prestageLeftClosedLoopError` | `AngularVelocity` | Left velocity error |
| `prestageRightClosedLoopError` | `Prestage/prestageRightClosedLoopError` | `AngularVelocity` | Right velocity error |
| `prestageLeftPos` | `Prestage/prestageLeftPos` | `Angle` | Left cumulative position |
| `prestageRightPos` | `Prestage/prestageRightPos` | `Angle` | Right cumulative position |

Implementations: `PrestageIOReal.java`, `PrestageIOSim.java`

### 5g. Upper Feeder — `Logger.processInputs("Feeder/Upper", inputs)` (UpperFeeder.java:25)

Source: `src/main/java/frc/robot/subsystems/upperFeeder/io/UpperFeederIO.java`

| Field | Log Key | Type | Purpose |
|-------|---------|------|---------|
| `upperFeederVoltage` | `Feeder/Upper/upperFeederVoltage` | `Voltage` | Applied voltage |
| `upperFeederStatorAmps` | `Feeder/Upper/upperFeederStatorAmps` | `Current` | Stator current |
| `upperFeederSupplyAmps` | `Feeder/Upper/upperFeederSupplyAmps` | `Current` | Supply current |
| `upperFeederMotorVelocity` | `Feeder/Upper/upperFeederMotorVelocity` | `AngularVelocity` | Roller velocity (RPS) |
| `upperFeederMotorTemperature` | `Feeder/Upper/upperFeederMotorTemperature` | `Temperature` | Motor temperature |
| `upperFeederClosedLoopReference` | `Feeder/Upper/upperFeederClosedLoopReference` | `AngularVelocity` | Velocity setpoint |
| `upperFeederClosedLoopError` | `Feeder/Upper/upperFeederClosedLoopError` | `AngularVelocity` | Velocity error |
| `upperFeederPos` | `Feeder/Upper/upperFeederPos` | `Angle` | Cumulative position |

Implementations: `UpperFeederIOReal.java`, `UpperFeederIOSim.java`

### 5h. Lower Feeder — `Logger.processInputs("Feeder/Lower", inputs)` (LowerFeeder.java:25)

Source: `src/main/java/frc/robot/subsystems/lowerFeeder/io/LowerFeederIO.java`

| Field | Log Key | Type | Purpose |
|-------|---------|------|---------|
| `lowerFeederVoltage` | `Feeder/Lower/lowerFeederVoltage` | `Voltage` | Applied voltage |
| `lowerFeederStatorAmps` | `Feeder/Lower/lowerFeederStatorAmps` | `Current` | Stator current |
| `lowerFeederSupplyAmps` | `Feeder/Lower/lowerFeederSupplyAmps` | `Current` | Supply current |
| `lowerFeederMotorVelocity` | `Feeder/Lower/lowerFeederMotorVelocity` | `AngularVelocity` | Roller velocity (RPS) |
| `lowerFeederMotorTemperature` | `Feeder/Lower/lowerFeederMotorTemperature` | `Temperature` | Motor temperature |
| `lowerFeederClosedLoopReference` | `Feeder/Lower/lowerFeederClosedLoopReference` | `AngularVelocity` | Velocity setpoint |
| `lowerFeederClosedLoopError` | `Feeder/Lower/lowerFeederClosedLoopError` | `AngularVelocity` | Velocity error |
| `lowerFeederPos` | `Feeder/Lower/lowerFeederPos` | `Angle` | Cumulative position |

Implementations: `LowerFeederIOReal.java`, `LowerFeederIOSim.java`

---

## 6. Autonomous Signals

### 6a. Path Preview and Selection (RobotContainer.java — `disabledPeriodic`)

| Signal | Log Key | Type | Line | Purpose |
|--------|---------|------|------|---------|
| Selected auto name | `Auto/SelectedAuto` | `String` | 1012 | Name of the chosen auto routine |
| Preview status message | `Auto/PreviewStatus` | `String` | 1022, 1047, 1051 | "No paths found for: X" / "Loaded N paths" / "Error loading: X" |
| Auto start pose | `Auto/StartPose` | `Pose2d` | 1048 | First waypoint of the selected path |

### 6b. Pre-Match Start Position Check (RobotContainer.java — `disabledPeriodic`)

| Signal | Log Key | Type | Line | Purpose |
|--------|---------|------|------|---------|
| Distance from start | `Auto/StartCheck/DistanceInches` | `double` | 1096 | How far the robot is from the expected auto start pose |
| Rotation error | `Auto/StartCheck/RotationDiffDegrees` | `double` | 1097 | Heading error from expected start rotation |
| Position within tolerance | `Auto/StartCheck/PositionOK` | `boolean` | 1077, 1098 | True when robot is close enough to start position |
| Rotation within tolerance | `Auto/StartCheck/RotationOK` | `boolean` | 1078, 1099 | True when robot heading is correct |

### 6c. PathPlanner A* — `Logger.processInputs("LocalADStarAK", io)` (LocalADStarAK.java:42,60)

The `LocalADStarAK` utility wraps PathPlanner's local A* pathfinder using an `ADStarIO` that implements `LoggableInputs`. This logs pathfinding state at each planning call. Replay-critical for autos that use PathPlanner dynamic pathfinding.

---

## 7. Command Scheduler Signals

### 7a. Auto-Aim Heading Control (DriveCommands.java — `joystickDriveAtAngle`)

| Signal | Log Key | Type | Line | Active When |
|--------|---------|------|------|-------------|
| Commanded heading | `AutoAim/TargetAngle` | `Rotation2d` | 218 | While auto-aim is running |
| Current heading | `AutoAim/CurrentAngle` | `Rotation2d` | 219 | While auto-aim is running |
| Angle error | `AutoAim/AngleErrorRad` | `double` | 220 | While auto-aim is running |
| Cleanup zeroes | `AutoAim/TargetAngle`, `AutoAim/CurrentAngle`, `AutoAim/AngleErrorRad` | `double` 0.0 | 244–246 | On command end or interrupt |

Analysis: `AutoAim/TargetAngle - AutoAim/CurrentAngle` should equal `AutoAim/AngleErrorRad`. Sustained error > tolerance with `RobotState/IsAlignedToHub = false` indicates PID tuning or vision angle issue.

### 7b. State Marker Strings

One-shot `recordOutput` calls used as event markers in the log:

| Signal | Log Key | Value | File:Line | Purpose |
|--------|---------|-------|-----------|---------|
| Tower side | `RobotState/towerAlign` | `"farSide"` or `"nearSide"` | DriveCommands.java:337,340 | Which side of the tower the robot is aligning to |
| X-brake | `RobotState/Drive` | `"Stopping with X"` | DriveCommands.java:427 | X-brake pattern engaged |
| Intake jostle | `RobotState/IntakePivot` | `"JostleCalled"` | IntakePivotCommands.java:69 | Jostle routine triggered |
| Shot started | `RobotState/shooting` | `true` | ShootSequences.java:29 | Shot sequence began |

**Bug:** `RobotState/shooting` is set to `true` at shot start and never reset to `false`. It is permanently `true` after the first shot.

---

## 8. Diagnostics Signals

### 8a. RobotState @AutoLogOutput (RobotState.java — registered in Robot.java:90)

All polled at 50 Hz:

| Signal | Log Key | Type | Line | Purpose |
|--------|---------|------|------|---------|
| Estimated pose | `RobotState/EstimatedPose` | `Pose2d` | 137 | Robot pose from Drive's SwerveDrivePoseEstimator |
| Field-relative velocity | `RobotState/FieldRelativeVelocity` | `ChassisSpeeds` | 169 | vx/vy/omega in field frame — used in vision rejection |
| Robot-relative velocity | `RobotState/RobotRelativeVelocity` | `ChassisSpeeds` | 188 | vx/vy/omega in robot frame |
| Distance to hub | `RobotState/DistanceToAllianceHub_m` | `Distance` | 209 | Alliance hub distance in meters (auto-flips for red) |
| Angle to hub | `RobotState/AngleToAllianceHub` | `Rotation2d` | 283 | Target heading to aim at alliance hub |
| Hub alignment | `RobotState/IsAlignedToHub` | `boolean` | 341 | True when within `hubAlignmentToleranceDegrees` |
| Pass alignment | `RobotState/IsAlignedToPass` | `boolean` | 368 | True when within `passAlignmentToleranceDegrees` |

### 8b. Game State Signals

| Signal | Log Key | Type | File:Line | Active Mode |
|--------|---------|------|-----------|-------------|
| Hub shift active | `RobotState/HubShift` | `boolean` | Robot.java:230 | Teleop only |
| First active alliance | `RobotState/firstActiveAlliancer` | (enum/string) | Robot.java:231 | Teleop only |
| Time remaining in shift | `RobotState/timeRemainingInShift` | `double` | Robot.java:232 | Teleop only |
| Disabled flag | `RobotState/disabled` | `boolean` | HubShiftUtil.java:428 | Any mode |

### 8c. Build Metadata (Logger.recordMetadata — before Logger.start())

| Key | Source | Purpose |
|-----|--------|---------|
| `ProjectName` | `BuildConstants.MAVEN_NAME` | Identifies the project |
| `BuildDate` | `BuildConstants.BUILD_DATE` | When the robot code was compiled |
| `GitSHA` | `BuildConstants.GIT_SHA` | Exact commit deployed |
| `GitDate` | `BuildConstants.GIT_DATE` | When that commit was made |
| `GitBranch` | `BuildConstants.GIT_BRANCH` | Which branch was deployed |
| `GitDirty` | `BuildConstants.DIRTY` (0/1/other) | "All changes committed" / "Uncommitted changes" / "Unknown" |

---

## 9. Performance / Power Signals

### 9a. BatteryLogger (BatteryLogger.java — runs after scheduler in Robot.robotPeriodic)

**Robot-wide totals (50 Hz):**

| Signal | Log Key | Type | Line | Purpose |
|--------|---------|------|------|---------|
| Total current | `BatteryLogger/Current` | `double` | 102 | Sum of all subsystem supply currents (amps) |
| Drive current | `BatteryLogger/DriveCurrent` | `double` | 103 | Current from drive-flagged subsystems |
| Total power | `BatteryLogger/Power` | `double` | 104 | Watts (current × battery voltage) |
| Cumulative energy | `BatteryLogger/Energy` | `double` | 105 | Watt-hours since robot enabled |

**Per-subsystem breakdowns** (dynamic keys — one per `reportCurrentUsage` call):
Three variants for each key: `BatteryLogger/Current/{key}`, `BatteryLogger/Power/{key}`, `BatteryLogger/Energy/{key}`

| Subsystem Key | isDrive | Notes |
|---------------|---------|-------|
| `Drive/Module0-Drive` through `Drive/Module3-Drive` | true | 4 drive motors |
| `Drive/Module0-Turn` through `Drive/Module3-Turn` | true | 4 turn motors |
| `Flywheel/Leader` | false | Leader TalonFX |
| `Flywheel/Follower1` through `Flywheel/Follower4` | false | 4 follower TalonFX |
| `Hood` | false | Hood TalonFX |
| `Intake/Pivot` | false | Pivot TalonFX |
| `Intake/Roller-Leader` | false | Roller leader |
| `Intake/Roller-Follower` | false | Roller follower |
| `Feeder/Upper` | false | Upper feeder |
| `Feeder/Lower` | false | Lower feeder |
| `Transport` | false | Transport motor |
| `Prestage/Left` | false | Left prestage |
| `Prestage/Right` | false | Right prestage |
| `Controls/roboRIO` | false | roboRIO current (from RobotController) |
| `Controls/CANcoders` | false | Estimated: 4 × 50 mA |
| `Controls/Pigeon` | false | Estimated: 40 mA |
| `Controls/CANivore` | false | Estimated: 30 mA |
| `Controls/Radio` | false | Estimated: 500 mA |

Parent keys are auto-aggregated — e.g., `Drive` = sum of all `Drive/Module*-Drive` + `Drive/Module*-Turn`.

**Not logged by BatteryLogger:** Battery voltage itself. Used internally for watt calculations but never published to the log.

---

## 10. Replay-Critical Signals

These signals must be present in a `.wpilog` file for replay to produce useful output.

| Signal Group | Log Keys | Why Critical |
|-------------|----------|--------------|
| Module high-freq odometry | `Drive/Module{0-3}/odometryTimestamps`, `odometryDrivePositionsRad`, `odometryTurnPositions` | High-frequency pose integration from `PhoenixOdometryThread` — without these, replay uses 50 Hz odometry and diverges |
| Gyro high-freq odometry | `Drive/Gyro/odometryYawTimestamps`, `odometryYawPositions` | Gyro fused with wheels at 250 Hz — missing = encoder-only dead reckoning |
| Vision inputs | `Vision/Camera{0-3}/poseObservations`, `tagIds`, `connected` | Vision measurements fused into pose estimator — replay must replay the same fusion decisions |
| PathPlanner inputs | `LocalADStarAK/*` | Pathfinding decisions that shaped auto command execution |
| All other `processInputs` groups | All `*IOInputs` fields | Every subsystem IO is logged via `processInputs` — this is the AKit replay contract |

**Safe to omit in replay** (derived/computed, not required):
- `SwerveStates/*`, `SwerveChassisSpeeds/*`, `Odometry/*` — recomputed from inputs
- `RobotState/*` (@AutoLogOutput) — recomputed each loop
- `BatteryLogger/*` — diagnostic only, not used by any control code
- `Vision/*/TagPoses`, `RobotPoses*`, `Summary/*` — recomputed from vision inputs
- `AutoAim/*`, state strings — command output signals

---

## 11. Issues Found

### Duplicate Logs
| Problem | Signals | Location |
|---------|---------|----------|
| `Flywheel/targetRPM` and `Flywheel/currentRPMTarget` both log `currentRPMTarget` | First on Flywheel.java:105 (in `isAtSetpoint()`), second on Flywheel.java:151 (in `setVelocity()`) | Flywheel.java |
| Vision ambiguity key uses camera name string vs camera index | `Vision/{cameraName}/Ambiguity` (VisionIOPhotonVision.java:117) uses the camera's name string; other vision outputs use `Vision/Camera{i}/...` with index — they won't align in AdvantageScope | VisionIOPhotonVision.java vs Vision.java |

### Misnamed / Unnamespaced Signals
| Signal | Problem | Fix |
|--------|---------|-----|
| `"Flywheel running"` | No namespace prefix — appears at the log root instead of under `Flywheel/` | Rename to `"Flywheel/measuredVelocity"` in FlywheelIOPhoenix6.java:345 |
| `"Intake pivot torque controls"` | No namespace prefix, logged from IO implementation not subsystem | Rename to `"Intake Pivot/torqueControlVelocity"` in IntakePivotIOReal.java:199 |

### Missing Signals
| Missing Signal | Why It Matters | Suggested Fix |
|----------------|---------------|---------------|
| Battery voltage | BatteryLogger uses voltage for watt calculations but never logs it — voltage sag is a key diagnostic | Add `Logger.recordOutput("BatteryLogger/Voltage", batteryVoltage)` in BatteryLogger |
| `RobotState/shooting = false` | Signal is set to `true` at shot start and never reset — useless for shot duration analysis | Add `recordOutput("RobotState/shooting", false)` in ShootSequences end/cancel |
| Wheel slip indicator | `driveVelocityRadPerSec` vs setpoint is loggable but no derived slip boolean exists | Add derived `Drive/Module{i}/slipping` boolean in Module.java periodic |
| `isAtSetpoint` booleans for feeders/transport/prestage | Reference + error are logged but no ready-state boolean | Add `recordOutput("Feeder/Upper/atSetpoint", ...)` etc. |
| Loop cycle time | Not explicitly published for easy alerting | AdvantageKit logs internal loop timing — check `Logger/LoopCycleMs` in the log |
| Vision pose jump magnitude | No logged delta between accepted vision pose and current estimated pose | Add `recordOutput("Vision/Summary/PoseJumpMeters", ...)` before calling `addVisionMeasurement` |

### Excessive / Unnecessary Signals
| Signal | Issue |
|--------|-------|
| `Flywheel/Visualizer/LaunchOriginPose` | Static geometry — doesn't change between matches. Only needs to be logged once at init, not at 50 Hz |
| Follower 1–4 voltages and currents (20 extra fields) | All followers are slaved to the leader. Logging all 4 × 5 follower fields every loop adds ~20 signals that mostly echo the leader. If followers are true Phoenix 6 followers, consider logging only velocity to confirm sync |
| Intake roller leader + follower both with full field sets | 16 fields for a 2-motor roller — same argument as flywheel followers |

### Replay-Breaking Gaps
| Gap | Impact |
|-----|--------|
| `RobotState` singleton not auto-discovered | Already fixed: `AutoLogOutputManager.addObject(RobotState.getInstance())` on Robot.java:90. Without this, none of the `@AutoLogOutput` signals on RobotState are logged |
| `LocalADStarAK` uses custom `LoggableInputs` | Non-standard pattern — verify `ADStarIO.toLog()` and `fromLog()` are correctly implemented for replay to work |
| `BatteryLogger` has no IO layer | Diagnostic utility only — not replayable by design. Acceptable |

### Desired vs Measured Comparison Gaps
| Mechanism | Desired | Measured | Gap |
|-----------|---------|----------|-----|
| Drive modules | `SwerveStates/Setpoints[i].speedMetersPerSecond` | `SwerveStates/Measured[i].speedMetersPerSecond` | No pre-computed error — must compute in AdvantageScope |
| Drive heading (auto-aim) | `AutoAim/TargetAngle` | `AutoAim/CurrentAngle` | `AutoAim/AngleErrorRad` is logged — **no gap** |
| Flywheel | `Flywheel/closedLoopReference` | `Flywheel/flywheelVelocity` | Error available via `Flywheel/closedLoopError` — **no gap** |
| Hood | `Hood/hoodClosedLoopReference` | `Hood/hoodPosition` | Error available via `Hood/hoodClosedLoopError` — **no gap** |
| Intake pivot | `Intake Pivot/intakePivotClosedLoopReference` | `Intake Pivot/intakePivotPosition` | Error logged as unitless double — no physical units |
| Vision pose vs odometry | `Vision/Summary/RobotPosesAccepted` | `Odometry/Robot` | **Gap** — no logged delta between vision estimate and fused pose |
