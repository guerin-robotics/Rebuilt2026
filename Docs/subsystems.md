# Subsystems

10 subsystems total. All follow the IO abstraction pattern described in
`architecture.md`. Source files live under `src/main/java/frc/robot/subsystems/`.

---

## 1. Drive

**Files:** `subsystems/drive/Drive.java`, `drive/DriveConstants.java`,
`drive/Module.java`, `drive/GyroIO*`, `drive/ModuleIO*`

**Hardware:** 4-module swerve drivetrain using Talon FX (Kraken X60) motors and
CANcoder absolute encoders. Pigeon 2 gyro (primary) or NavX (fallback).

**IO Interfaces:**
- `GyroIO` → `GyroIOPigeon2`, `GyroIONavX`
- `ModuleIO` → `ModuleIOTalonFX`, `ModuleIOTalonFXS`, `ModuleIOSim`

**State:**
- `SwerveDrivePoseEstimator` — single source of robot pose (wheels + gyro + vision)
- `SwerveModulePosition[]` — per-module distance + angle
- `Rotation2d rawGyroRotation` — raw gyro reading
- `boolean areWheelsXed` — tracks X-lock state

**Key methods:**
- `runVelocity(ChassisSpeeds)` — execute velocity command (used by path follower and teleop)
- `addVisionMeasurement(Pose3d, long, Matrix)` — ingest vision pose update
- `getPose()` — returns estimated pose
- `setPose(Pose2d)` — reset odometry
- `getChassisSpeeds()` — current robot velocity

**Odometry:** `PhoenixOdometryThread` runs a high-frequency loop (250 Hz on
CAN FD, 100 Hz standard) to capture encoder/gyro deltas before the main 50 Hz
loop. All captured samples are applied to `SwerveDrivePoseEstimator` in
`periodic()`.

**Path following:** Configured via `AutoBuilder.configure()` with
`PPHolonomicDriveController` (kP = 5.0 for both translation and rotation).
`LocalADStarAK` handles dynamic obstacle avoidance and replanning.

**SysId:** `sysIdQuasistatic()` and `sysIdDynamic()` routines exist for drive
characterization. Commented out of auto chooser to prevent accidental match use.

---

## 2. Vision

**Files:** `subsystems/vision/Vision.java`, `vision/VisionConstants.java`,
`vision/io/VisionIO*`

**Hardware:** 4 PhotonVision cameras

| Index | Name | Location |
|-------|------|----------|
| 0 | RobotRight | Front-right, X=1.0", Y=-12.17", Z=6.44", pitch=-15° |
| 1 | RobotLeft | Front-left, X=1.0", Y=+12.42", Z=6.44", pitch=-15° |
| 2 | ShooterRight | Rear-right, X=-13.58", Y=-6.13", Z=12.53", pitch=-15° |
| 3 | ShooterLeft | Rear-left, X=-13.58", Y=+6.13", Z=12.53", pitch=-15° |

**IO Interfaces:** `VisionIO` → `VisionIOPhotonVision`, `VisionIOPhotonVisionSim`

**Constructor:** `Vision(VisionConsumer consumer, VisionIO... io)`. The consumer
is `drive::addVisionMeasurement`. This decouples Vision from Drive — Vision does
not hold a reference to the Drive subsystem.

**Filtering (applied each cycle):**
1. Angular velocity pre-filter: if robot spinning above `maxAngularVelocityRadPerSec`,
   skip all cameras for this cycle (motion blur)
2. Tag distance filter: reject observations where average tag distance exceeds
   `maxDistanceMeters`
3. Pitch/roll filter: reject observations where estimated pitch/roll exceeds
   `maxPitchRollRadians` (robot should be flat on carpet)
4. Standard deviations scale with distance and inversely with tag count — closer
   tags and more tags get more trust

**Targeting:** `getTargetX(cameraIndex)` returns the yaw angle to the best
visible target for driver-assist alignment.

**Field layout:** 2026 Rebuilt AprilTag layout from WPILib standard JSON.

---

## 3. Flywheel

**Files:** `subsystems/flywheel/Flywheel.java`, `flywheel/FlywheelConstants.java`,
`flywheel/ShotCalculator.java`, `flywheel/io/FlywheelIO*`

**Hardware:** 1 Talon FX leader (ID 30) + 4 followers (IDs 31–34). All Kraken X60
motors. Velocity closed-loop via `VelocityTorqueCurrentFOC`.

**IO Interfaces:** `FlywheelIO` → `FlywheelIOPhoenix6`, `FlywheelIOSim`

**State:**
- `currentRPMTarget` — last commanded velocity (for spinup check)
- `isFlywheelSpunUp` — boolean trigger, logged via `@AutoLogOutput`
- `hoodAngleSupplier` — supplier wired to Hood.getPosition() for visualizer

**Key methods:**
- `setFlywheelVelocity(AngularVelocity)` — velocity closed-loop
- `setFlywheelVoltage(Voltage)` — direct voltage (SysId / characterization)
- `setFlywheelIdle()` — runs at `HardwareConstants.CompConstants.Velocities.idleRPM`
- `isSpunUp()` — true when velocity within `spinupThreshold` RPM of setpoint

**Shot lookup:** `ShotCalculator` holds a distance-to-RPM interpolation table.
`FlywheelCommands.setVelocityForHub()` reads distance from `RobotState` and looks
up the target velocity.

**Tuning override:** `LoggedNetworkNumber tuningRPM` exposes a NetworkTable entry
(`Tune/flywheel/tuningRPM`) for real-time adjustment. Active only when
`TuningConstants.TUNING_MODE` is true.

**Visualization:** `FlywheelVisualizer` renders a 3D trajectory preview using the
hood angle supplier.

---

## 4. Hood

**Files:** `subsystems/hood/Hood.java`, `hood/HoodConstants.java`,
`hood/HoodPosCalculator.java`, `hood/io/HoodIO*`

**Hardware:** 1 Talon FX (ID 35), 1 CANcoder (ID 50). Position closed-loop via
`PositionVoltage` with CANcoder as feedback source.

**IO Interfaces:** `HoodIO` → `HoodIOReal`, `HoodIOSim`

**Key positions:**

| Position | Angle |
|----------|-------|
| Down (stowed) | 0° |
| Tower shot | 2.5° |
| Pass shot | 35° |
| Hub (variable) | Interpolated from `HoodPosCalculator` |

**Key methods:**
- `setHoodPos(Angle)` — direct angle command
- `setHoodPosForHub()` — distance-based lookup via `HoodPosCalculator`
- `setHoodPosForPass()` — fixed pass angle
- `incrementHoodPos()` — +5° manual adjustment
- `getPosition()` — current angle (fed to FlywheelVisualizer)

**Default command:** `hoodIdle()` — holds last position, prevents stale PID
reference when no command is active.

---

## 5. Prestage

**Files:** `subsystems/prestage/Prestage.java`, `prestage/PrestageConstants.java`,
`prestage/io/PrestageIO*`

**Hardware:** 2 Talon FX motors (IDs 37–38, leader + follower). Velocity
closed-loop via `VelocityTorqueCurrentFOC`.

**IO Interfaces:** `PrestageIO` → `PrestageIOReal`, `PrestageIOSim`

**Velocities (from `HardwareConstants`):**

| Mode | RPM |
|------|-----|
| Shoot | 3000 |
| Idle | 1300 |

**Key methods:**
- `setPrestageVelocity(AngularVelocity)` — velocity control
- `setPrestageVoltage(Voltage)` — direct voltage

---

## 6. Upper Feeder

**Files:** `subsystems/upperFeeder/UpperFeeder.java`, `upperFeeder/io/UpperFeederIO*`

**Hardware:** 1 Talon FX (ID 36). Velocity closed-loop.

**Shoot velocity:** −3000 RPM (negative = correct direction for feeding toward shooter)

**Key methods:**
- `setUpperFeederVelocity(AngularVelocity)`
- `setUpperFeederVoltage(Voltage)`

---

## 7. Lower Feeder

**Files:** `subsystems/lowerFeeder/LowerFeeder.java`, `lowerFeeder/io/LowerFeederIO*`

**Hardware:** 1 Talon FX (ID 39). Velocity closed-loop.

Mirrors Upper Feeder in structure and constants.

---

## 8. Transport

**Files:** `subsystems/transport/Transport.java`, `transport/TransportConstants.java`,
`transport/io/TransportIO*`

**Hardware:** 1 Talon FX (ID 40). Velocity closed-loop.

**Transport velocity (move fuel):** −1800 RPM

**Key methods:**
- `setTransportVelocity(AngularVelocity)`
- `setTransportVoltage(Voltage)`

---

## 9. Intake Pivot

**Files:** `subsystems/intakePivot/IntakePivot.java`,
`intakePivot/IntakePivotConstants.java`, `intakePivot/IntakePivotVisualizer.java`,
`intakePivot/io/IntakePivotIO*`

**Hardware:** 1 Talon FX (ID 41), 1 CANcoder (ID 44). Position closed-loop via
`PositionVoltage`.

**IO Interfaces:** `IntakePivotIO` → `IntakePivotIOReal`, `IntakePivotIOSim`

**Key positions (in rotations):**

| Position | Rotations |
|----------|-----------|
| Down (deployed) | 0.0 |
| Up (stowed) | 0.3 |
| Jostle 1 | 0.115 |
| Jostle 2 | 0.16 |

**Key methods:**
- `setPivotPosition(Angle)` — position command
- `zeroPivotEncoder()` — reset encoder to 0
- `setPivotVoltage(Voltage)` — direct voltage

**Visualization:** `IntakePivotVisualizer` renders current and goal positions in
WPILib Mechanism2d.

---

## 10. Intake Roller

**Files:** `subsystems/intakeRoller/intakeRoller.java` (note: lowercase class name),
`intakeRoller/io/intakeRollerIO*`

**Hardware:** 2 Talon FX motors (IDs 42–43, leader + follower).

**Voltages (from `HardwareConstants`):**

| Mode | Voltage |
|------|---------|
| Intake | +12 V |
| Agitate (slow eject) | +3 V |

**Key methods:**
- `setRollerVoltage(Voltage)` — direct voltage (primary control mode)
- `setRollerVelocity(AngularVelocity)` — velocity control

---

## Subsystem Relationships

```
Intake Roller ──┐
Intake Pivot    ├──> Feed fuel into robot
Transport ──────┘

Lower Feeder ──┐
Upper Feeder   ├──> Move fuel toward shooter
Prestage ──────┘

Flywheel ──┐
Hood       ├──> Launch fuel
           │
           └── FlywheelVisualizer (reads Hood angle)

Drive ──────────> RobotState (pose) ──> Flywheel/Hood (distance-based shot)
Vision ─────────> Drive (addVisionMeasurement)
```

The shooter pipeline (Transport → Lower Feeder → Upper Feeder → Prestage →
Flywheel) runs in sequence during a shot. Individual subsystems have no direct
references to each other — coordination happens in `ShootSequences.java`
commands.
