# Logging and Telemetry

AdvantageKit 3.x is the logging backbone. Every subsystem follows the same
pattern: an `@AutoLog`-annotated inputs struct populated by the IO layer,
flushed to the log every cycle via `Logger.processInputs()`, plus manual
`Logger.recordOutput()` calls for derived state.

---

## Initialization Sequence (Robot.java)

Order matters — `Logger.start()` must be called before any subsystem is
constructed, because `Logger.processInputs()` will silently no-op if logging
hasn't started.

```
Robot.java constructor:
  1. Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME)
     Logger.recordMetadata("BuildDate",   BuildConstants.BUILD_DATE)
     Logger.recordMetadata("GitSHA",      BuildConstants.GIT_SHA)
     Logger.recordMetadata("GitDate",     BuildConstants.GIT_DATE)
     Logger.recordMetadata("GitBranch",   BuildConstants.GIT_BRANCH)
     Logger.recordMetadata("GitDirty",    String.valueOf(BuildConstants.DIRTY))

  2. Mode-dependent receiver setup:
     REAL:   WPILOGWriter() + NT4Publisher()
     SIM:    NT4Publisher() only
     REPLAY: WPILOGReader(logPath) + WPILOGWriter(logPath + "_sim")

  3. Logger.start()

  4. new RobotContainer()  ← subsystems constructed here

  5. AutoLogOutputManager.addObject(RobotState.getInstance())
     ↑ RobotState is a singleton not in the constructor graph, so it must
       be manually registered for @AutoLogOutput methods to be discovered.
```

---

## Metadata Logged

| Key | Value |
|-----|-------|
| `ProjectName` | Maven artifact name |
| `BuildDate` | Timestamp of build |
| `GitSHA` | Full commit hash |
| `GitDate` | Commit timestamp |
| `GitBranch` | Branch name |
| `GitDirty` | Whether tree had uncommitted changes |

These appear in every `.wpilog` file header. When debugging a match log, you
can identify exactly which commit was running on the robot.

---

## NT Publishing

In REAL and SIM modes, `NT4Publisher` is added before `Logger.start()`. This
causes every logged value (all `@AutoLog` inputs fields, all `@AutoLogOutput`
method return values, all manual `Logger.recordOutput()` calls) to be
simultaneously broadcast over NetworkTables 4. AdvantageScope, Shuffleboard,
and any custom dashboard can consume this stream.

In REPLAY mode, no NT4Publisher is added — replay runs as fast as possible
offline and there is no live dashboard.

---

## Per-Subsystem Logging

### Drive

**processInputs key:** `"Drive/Gyro"` (gyro), `"Drive/Module{0-3}"` (modules)

**@AutoLog fields — Gyro (`GyroIOInputs`):**

| Field | Type | What it is |
|-------|------|-----------|
| `connected` | `boolean` | Pigeon2/NavX communication alive |
| `yawPosition` | `Rotation2d` | Robot heading |
| `yawVelocityRadPerSec` | `double` | Turn rate |
| `odometryYawTimestamps[]` | `double[]` | Timestamps for high-freq samples |
| `odometryYawPositions[]` | `Rotation2d[]` | Heading at each high-freq sample |

**@AutoLog fields — Module (`ModuleIOInputs`):**

| Field | Type | What it is |
|-------|------|-----------|
| `driveConnected` | `boolean` | Drive TalonFX comms |
| `drivePositionRad` | `double` | Integrated wheel angle |
| `driveVelocityRadPerSec` | `double` | Wheel speed |
| `driveAppliedVolts` | `double` | Drive motor voltage |
| `driveCurrentAmps` | `double` | Drive motor supply current |
| `turnConnected` | `boolean` | Turn TalonFX comms |
| `turnEncoderConnected` | `boolean` | CANcoder comms |
| `turnAbsolutePosition` | `Rotation2d` | CANcoder absolute reading |
| `turnPosition` | `Rotation2d` | Fused turn position |
| `turnVelocityRadPerSec` | `double` | Turn rate |
| `turnAppliedVolts` | `double` | Turn motor voltage |
| `turnCurrentAmps` | `double` | Turn motor supply current |
| `odometryTimestamps[]` | `double[]` | High-freq odometry timestamps |
| `odometryDrivePositionsRad[]` | `double[]` | Drive positions at each sample |
| `odometryTurnPositions[]` | `Rotation2d[]` | Turn positions at each sample |

**@AutoLogOutput methods (Drive.java):**

| Key | Type | Source |
|-----|------|--------|
| `"SwerveStates/Measured"` | `SwerveModuleState[4]` | Current module state array |
| `"SwerveChassisSpeeds/Measured"` | `ChassisSpeeds` | Kinematics inverse from module states |
| `"Odometry/Robot"` | `Pose2d` | `poseEstimator.getEstimatedPosition()` |

**Manual recordOutput (Drive.java):**

| Key | Type | When |
|-----|------|------|
| `"Odometry/Trajectory"` | `Pose2d[]` | While PathPlanner path is active |
| `"Odometry/TrajectorySetpoint"` | `Pose2d` | While PathPlanner path is active |
| `"SwerveStates/Setpoints"` | `SwerveModuleState[4]` | Before optimization each cycle |
| `"SwerveStates/SetpointsOptimized"` | `SwerveModuleState[4]` | After optimization each cycle |
| `"SwerveChassisSpeeds/Setpoints"` | `ChassisSpeeds` | Discretized target speeds |
| `"Drive/SysIdState"` | `String` | During SysId routines |

---

### Vision

**processInputs key:** `"Vision/Camera{0-3}"`

**@AutoLog fields (`VisionIOInputs`):**

| Field | Type | What it is |
|-------|------|-----------|
| `connected` | `boolean` | Camera reachable |
| `latestTargetObservation` | `TargetObservation` | Best target tx/ty for servoing |
| `poseObservations[]` | `PoseObservation[]` | All robot pose estimates this cycle |
| `tagIds[]` | `int[]` | IDs of all visible tags |

`PoseObservation` record fields: `timestamp`, `pose` (Pose3d), `ambiguity`,
`tagCount`, `averageTagDistance`, `type` (MEGATAG\_1 / MEGATAG\_2 / PHOTONVISION)

**Manual recordOutput (Vision.java) — per camera:**

| Key | Type | Content |
|-----|------|---------|
| `"Vision/Camera{i}/TagPoses"` | `Pose3d[]` | Positions of detected tags |
| `"Vision/Camera{i}/RobotPoses"` | `Pose3d[]` | All candidate robot poses |
| `"Vision/Camera{i}/RobotPosesAccepted"` | `Pose3d[]` | Observations passed all filters |
| `"Vision/Camera{i}/RobotPosesRejected"` | `Pose3d[]` | Observations that were filtered out |
| `"Vision/Camera{i}/TagCount"` | `int` | Number of visible tags |
| `"Vision/Camera{i}/IsMultiTag"` | `boolean` | Multi-tag solve used |

**Manual recordOutput — summary across all cameras:**

| Key | Content |
|-----|---------|
| `"Vision/Summary/TagPoses"` | All visible tag poses |
| `"Vision/Summary/RobotPoses"` | All candidate poses |
| `"Vision/Summary/RobotPosesAccepted"` | All accepted poses |
| `"Vision/Summary/RobotPosesRejected"` | All rejected poses |

Vision is the most thoroughly logged subsystem. Accepted and rejected
observations are both visible, making filter debugging straightforward.

---

### Flywheel

**processInputs key:** `"Flywheel"`

**@AutoLog fields (`ShooterIOInputs`):**

| Field | Type |
|-------|------|
| `flywheelVelocity` | `AngularVelocity` |
| `closedLoopError` | `AngularVelocity` |
| `closedLoopReference` | `AngularVelocity` |
| `leaderVelocity` | `AngularVelocity` |
| `leaderAppliedVolts` | `Voltage` |
| `leaderSupplyCurrentAmps` | `Current` |
| `leaderStatorCurrentAmps` | `Current` |
| `leaderTemp` | `Temperature` |
| `leaderAngle` | `Angle` |
| `follower1Velocity` – `follower4Velocity` | `AngularVelocity` |
| `follower1AppliedVolts` – `follower4AppliedVolts` | `Voltage` |
| `follower1SupplyCurrentAmps` – `follower4SupplyCurrentAmps` | `Current` |
| `follower1StatorCurrentAmps` – `follower4StatorCurrentAmps` | `Current` |
| `follower1Temp` – `follower4Temp` | `Temperature` |

**Manual recordOutput (Flywheel.java):**

| Key | Content | Location |
|-----|---------|---------|
| `"Flywheel/targetRPM"` | `currentRPMTarget` | `periodic()` |
| `"Flywheel/currentRPMTarget"` | `currentRPMTarget` | inside `isSpunUp()` |

**Note:** `currentRPMTarget` is logged twice: once in `periodic()` and once
inside `isSpunUp()`. Since `isSpunUp()` is called from the `isFlywheelSpunUp`
`LoggedTrigger` every cycle (via the scheduler's Trigger polling), this
produces a duplicate log entry under a different key each cycle.

**LoggedNetworkNumber:** `"Tune/flywheel/tuningRPM"` — default 20 RPM.

**LoggedTrigger:** `"isFlywheelSpunUp"` — logs true/false on every state change.

---

### Hood

**processInputs key:** `"Hood"`

**@AutoLog fields (`HoodIOInputs`):**

| Field | Type |
|-------|------|
| `hoodVoltage` | `Voltage` |
| `hoodSupplyCurrent` | `Current` |
| `hoodStatorCurrent` | `Current` |
| `hoodTemperature` | `Temperature` |
| `hoodVelocity` | `AngularVelocity` |
| `hoodPosition` | `Angle` |
| `hoodClosedLoopReference` | `Angle` |
| `hoodClosedLoopError` | `Angle` |

---

### Prestage

**processInputs key:** `"Prestage"`

**@AutoLog fields (`PrestageIOInputs`):**

| Field | Type |
|-------|------|
| `prestageLeftVoltage` | `Voltage` |
| `prestageLeftStatorAmps` | `Current` |
| `prestageLeftSupplyAmps` | `Current` |
| `prestageLeftVelocity` | `AngularVelocity` |
| `prestageLeftTemperature` | `Temperature` |
| `prestageLeftClosedLoopReference` | `AngularVelocity` |
| `prestageLeftClosedLoopError` | `AngularVelocity` |
| `prestageLeftPos` | `Angle` |
| *(right motor mirrors left)* | |

---

### Upper Feeder

**processInputs key:** `"Feeder/Upper"`

**@AutoLog fields (`UpperFeederIOInputs`):**

| Field | Type |
|-------|------|
| `upperFeederVoltage` | `Voltage` |
| `upperFeederStatorAmps` | `Current` |
| `upperFeederSupplyAmps` | `Current` |
| `upperFeederMotorVelocity` | `AngularVelocity` |
| `upperFeederMotorTemperature` | `Temperature` |
| `upperFeederClosedLoopReference` | `AngularVelocity` |
| `upperFeederClosedLoopError` | `AngularVelocity` |
| `upperFeederPos` | `Angle` |

---

### Lower Feeder

**processInputs key:** `"Feeder/Lower"`

Identical field set to UpperFeeder with `lowerFeeder` prefix.

---

### Transport

**processInputs key:** `"Transport"`

**@AutoLog fields (`TransportIOInputs`):**

| Field | Type |
|-------|------|
| `TransportVoltage` | `Voltage` |
| `TransportStatorAmps` | `Current` |
| `TransportSupplyAmps` | `Current` |
| `TransportMotorVelocity` | `AngularVelocity` |
| `TransportMotorTemperature` | `Temperature` |
| `transportClosedLoopReference` | `AngularVelocity` |
| `transportClosedLoopError` | `AngularVelocity` |
| `transportPos` | `Angle` |

**Note:** `Transport*` fields use inconsistent capitalization (`TransportVoltage`
vs `transportClosedLoopReference`). AKit logs both correctly but they appear
under mixed case in AdvantageScope, which is confusing.

---

### Intake Pivot

**processInputs key:** `"Intake Pivot"` (space in key — legal in AKit, but
produces a different path separator in some dashboard tools)

**@AutoLog fields (`IntakePivotIOInputs`):**

| Field | Type | Note |
|-------|------|------|
| `intakePivotVoltage` | `Voltage` | |
| `intakePivotSupplyCurrent` | `Current` | |
| `intakePivotStatorCurrent` | `Current` | |
| `intakePivotTemperature` | `Temperature` | |
| `intakePivotVelocity` | `AngularVelocity` | |
| `intakePivotPosition` | `Angle` | |
| `intakePivotClosedLoopReference` | `double` | Not `Angle` — inconsistent type |
| `intakePivotClosedLoopError` | `double` | Not `Angle` — inconsistent type |

---

### Intake Roller

**processInputs key:** `"Intake Roller"` (space in key)

**@AutoLog fields (`intakeRollerIOInputs`):**

Leader and follower fields with `intakeRoller*` / `*Follower` suffixes.
Both motors log: voltage, supply current, stator current, temperature,
velocity, closed-loop reference and error, position.

---

### RobotState

Registered via `AutoLogOutputManager.addObject(RobotState.getInstance())`.
All `@AutoLogOutput`-annotated methods are auto-logged every cycle.

| Key | Type | Computation |
|-----|------|------------|
| `"RobotState/EstimatedPose"` | `Pose2d` | `poseSupplier.get()` |
| `"RobotState/FieldRelativeVelocity"` | `ChassisSpeeds` | Derived from module states |
| `"RobotState/RobotRelativeVelocity"` | `ChassisSpeeds` | Field velocity rotated by heading |
| `"RobotState/DistanceToAllianceHub_m"` | `Distance` | Pose → hub geometry |
| `"RobotState/AngleToAllianceHub"` | `Rotation2d` | Pose → hub bearing |
| `"RobotState/IsAlignedToHub"` | `boolean` | Heading error < 1.5° |
| `"RobotState/IsAlignedToPass"` | `boolean` | Heading error < 7.0° |

**Manual recordOutput in Robot.teleopPeriodic():**

| Key | Content |
|-----|---------|
| `"RobotState/HubShift"` | Whether hub shift is active |
| `"RobotState/firstActiveAlliancer"` | First-active alliance state |
| `"RobotState/timeRemainingInShift"` | Seconds remaining in hub shift |

---

### BatteryLogger

Aggregates supply current from all subsystems and logs power budgeting data.
Called from `Robot.robotPeriodic()` after `CommandScheduler.getInstance().run()`.

| Key | Content |
|-----|---------|
| `"BatteryLogger/Current"` | Total estimated current draw (A) |
| `"BatteryLogger/DriveCurrent"` | Drive-only current (A) |
| `"BatteryLogger/Power"` | Total power (W) |
| `"BatteryLogger/Energy"` | Cumulative energy (J) |
| `"BatteryLogger/Current/{subsystem}"` | Per-subsystem current breakdown |
| `"BatteryLogger/Power/{subsystem}"` | Per-subsystem power breakdown |
| `"BatteryLogger/Energy/{subsystem}"` | Per-subsystem energy breakdown |

Fixed overheads added each cycle: roboRIO, CANcoders, Pigeon2, CANivore, Radio.

---

### Auto Diagnostics

| Key | Type | Set in |
|-----|------|--------|
| `"Auto/SelectedAuto"` | `String` | `updateAutoPreview()` |
| `"Auto/PreviewStatus"` | `String` | `updateAutoPreview()` |
| `"Auto/StartPose"` | `Pose2d` | `updateAutoPreview()` |
| `"Auto/StartCheck/PositionOK"` | `boolean` | `checkStartPose()` |
| `"Auto/StartCheck/RotationOK"` | `boolean` | `checkStartPose()` |
| `"Auto/StartCheck/DistanceInches"` | `double` | `checkStartPose()` |
| `"Auto/StartCheck/RotationDiffDegrees"` | `double` | `checkStartPose()` |

---

## Missing Logs

### 1. Explicit goal/setpoint output for most subsystems

Flywheel has `"Flywheel/targetRPM"` logged manually. No other subsystem logs
its commanded setpoint as a top-level output. The closed-loop reference *is*
inside the `@AutoLog` inputs struct (e.g., `hoodClosedLoopReference`), but this
is the motor controller's internal reference — not the command the subsystem
method received.

For Hood, IntakePivot, Prestage, Feeders, Transport: there is no explicit
`"Subsystem/GoalAngle"` or `"Subsystem/GoalVelocity"` log. To diagnose "did
the command set the right target?" you must read the closed-loop reference from
the inputs struct, which is one level of indirection.

**Affected subsystems:** Hood, Prestage, UpperFeeder, LowerFeeder, Transport,
IntakePivot, intakeRoller

### 2. IntakePivot goalPosition field not logged

`IntakePivot.java` tracks `goalPosition` for the visualizer but never logs it
to AdvantageKit. In replay, the visualizer doesn't run (no display), so the
goal is invisible.

### 3. Drive module temperatures not logged

`ModuleIOInputs` has no temperature fields for either drive or turn TalonFX
motors. Flywheel, Hood, Feeders, and all other subsystems log temperature — but
drive motors do not. Drive modules run the hardest and overheat silently.

### 4. LoggedTrigger key is not namespaced

`Flywheel.isFlywheelSpunUp` logs to `"isFlywheelSpunUp"` (root level, no
subsystem prefix). In AdvantageScope, this appears at the top of the signal
tree rather than under `Flywheel/`. Minor, but inconsistent with the rest of
the logging hierarchy.

### 5. currentRPMTarget logged under two keys

`"Flywheel/targetRPM"` (in `periodic()`) and `"Flywheel/currentRPMTarget"`
(inside `isSpunUp()`) log the same field every cycle. Neither key is wrong,
but having both creates confusion about which is canonical.

---

## Excessive / Redundant Logs

### isSpunUp() side-effect logging

`Flywheel.isSpunUp()` (called as a getter) calls `Logger.recordOutput()` as a
side effect (Flywheel.java:151). Getters with side effects are a code smell.
`isSpunUp()` is called from:
- `isFlywheelSpunUp` LoggedTrigger (every cycle via scheduler polling)
- `ShootSequences.autoShootToHub()` waitUntil condition (every cycle during auto)
- Potentially other command conditions

Each call logs `"Flywheel/currentRPMTarget"` again. The value doesn't change
between calls in the same cycle, so this is redundant writes per cycle.

### Summary vision keys duplicate per-camera keys

`"Vision/Summary/*"` concatenates data already logged under
`"Vision/Camera{i}/*"`. For match debugging this is convenient, but it doubles
the number of Vision pose arrays written to the log each cycle (8 arrays per
camera cycle + 4 summary arrays = 36 Pose3d[] writes per cycle when all cameras
have data).

---

## Naming Inconsistencies

| Location | Issue |
|----------|-------|
| `TransportIO.java` | Field prefix capitalized (`TransportVoltage`) vs. lower-camel (`transportClosedLoopReference`) |
| `IntakePivotIO.java` | `closedLoopReference/Error` typed as `double` instead of `Angle` (all other subsystems use typed units) |
| `"Intake Pivot"` processInputs key | Space in key works in AKit but may cause issues in some NT path parsers |
| `"Intake Roller"` processInputs key | Same space issue |
| `"isFlywheelSpunUp"` LoggedTrigger key | No subsystem prefix; appears at root in AdvantageScope |
