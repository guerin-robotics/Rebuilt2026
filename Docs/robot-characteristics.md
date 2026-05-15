# Robot Characteristics — Rebuilt2026

Complete reverse-engineered specification of the Rebuilt2026 FRC robot. All values sourced directly from repository constants, IO implementations, and generated tuner files.

---

## 1. Robot Overview

| Property | Value | Source |
|----------|-------|--------|
| Active robot configuration | `COMP` | Constants.java:22 |
| Secondary robot | `ALPHA` (development/practice bot) | Constants.java:35-39 |
| Framework | AdvantageKit on WPILib Java | Robot.java |
| Drive architecture | 4-wheel holonomic swerve | COMP_TunerConstants.java |
| Motor controller family | CTRE Phoenix 6 TalonFX (Kraken X60) | All IO implementations |
| Gyroscope | Pigeon 2 (CAN ID 0) | COMP_TunerConstants.java:106 |
| Vision system | PhotonVision, 4 cameras | VisionConstants.java |
| Path planning | PathPlanner with LocalADStar | Drive.java:134 |
| Autonomous mode default | `"2.5-Left-Comp"` | HardwareConstants.java:119 |
| Simulation mode (when not real) | `SIM` (physics) | Constants.java:18 |
| Total CAN devices | 29 | Robot.java pre-flight checklist |

### Game context (inferred from constants)
The zone system, hub targets, tower positions, pass targets, hub shift logic, and shot-distance tables indicate this robot is built for **2026 Rebuilt** — a game with:
- Alliance hubs that teams score into
- A hub ownership "shift" mechanic (HubShiftUtil)
- Trench zones, bump zones, alliance/opponent zones
- Alliance towers that require alignment to score
- Pass shots across the field
- Autonomous paths labeled Left/Right/Center/Champs

---

## 2. Physical Configuration

### Frame and Bumpers
| Property | Value | Units | Source |
|----------|-------|-------|--------|
| Robot width (PathPlanner) | 0.9 | m | pathplanner/settings.json:2 |
| Robot length (PathPlanner) | 0.9 | m | pathplanner/settings.json:3 |
| Robot mass | 63.503 | kg (139.9 lbs) | Drive.java:68, settings.json:21 |
| Moment of inertia | 5.162 | kg·m² | Drive.java:69, settings.json:22 |
| Wheel coefficient of friction | 1.2 | dimensionless | Drive.java:70, settings.json:28 |

### Module Positions (COMP robot)
Module wheelbase: 22 inches square (11" from center in each axis).

| Module | X (forward) | Y (left) | CAN IDs (Drive/Steer/Encoder) |
|--------|-------------|----------|-------------------------------|
| Front Left | +11 in (+0.2794 m) | +11 in (+0.2794 m) | 4 / 5 / 6 |
| Front Right | +11 in (+0.2794 m) | -11 in (−0.2794 m) | 7 / 8 / 9 |
| Back Left | -11 in (−0.2794 m) | +11 in (+0.2794 m) | 1 / 2 / 3 |
| Back Right | -11 in (−0.2794 m) | -11 in (−0.2794 m) | 10 / 11 / 12 |

Source: COMP_TunerConstants.java:155-189

Drive base radius (center to module): `sqrt(11² + 11²)` = **15.56 inches (0.3953 m)**
Source: Drive.java:58-65 (computed dynamically from TunerConstants positions)

### Shooter Placement
- Shooter exit point (visualizer): X = −6 in behind robot center, Y = 0 (centered), Z = 20 in above floor
- Source: FlywheelConstants.java:198-201

### Intake Placement (PathPlanner feature)
- Intake feature center: X = +0.6 m (forward), size = 0.7 m × 0.3 m (rounded rectangle)
- Source: pathplanner/settings.json:41

---

## 3. Drivetrain Characteristics

### COMP Robot Swerve Configuration
| Parameter | Value | Units | Source |
|-----------|-------|-------|--------|
| Drive motor type | TalonFX (Kraken X60 FOC) | — | COMP_TunerConstants.java:50-51 |
| Steer motor type | TalonFX (Kraken X60 FOC) | — | COMP_TunerConstants.java:53-54 |
| Drive gear ratio | 7.03125 : 1 | — | COMP_TunerConstants.java:99 |
| Steer gear ratio | 26.0909 : 1 | — | COMP_TunerConstants.java:100 |
| Coupling ratio | 4.5 | drive turns / steer turn | COMP_TunerConstants.java:97 |
| Wheel radius | 2.0 | inches (0.0508 m) | COMP_TunerConstants.java:101 |
| Free speed at 12 V | 4.39 | m/s | COMP_TunerConstants.java:93 |
| CAN bus (drivetrain) | `"Canivore"` | CANivore FD | COMP_TunerConstants.java:89 |
| Odometry frequency | 250 | Hz (CANivore FD) | Drive.java:57 |
| Steer feedback type | FusedCANcoder | — | COMP_TunerConstants.java:58 |
| Drive control type | TorqueCurrentFOC | — | COMP_TunerConstants.java:46-47 |
| Steer control type | TorqueCurrentFOC | — | COMP_TunerConstants.java:42-43 |
| Slip current threshold | 60 | A | COMP_TunerConstants.java:62 |
| Drive supply limit | 70 | A | COMP_TunerConstants.java:70 |
| Steer stator limit | 40 | A | COMP_TunerConstants.java:79 |
| Steer supply limit | 40 | A | COMP_TunerConstants.java:81 |

### COMP Drive PID/FF Gains (Slot0, TorqueCurrentFOC)
| Parameter | Value | Meaning |
|-----------|-------|---------|
| kP | 35 | A/RPS error |
| kS | 3.18 | A static friction |
| kV | 1.2 | A/RPS feedforward |
| kI | 0 | — |
| kD | 0 | — |

Source: COMP_TunerConstants.java:36-37

### COMP Steer PID/FF Gains (Slot0, TorqueCurrentFOC)
| Parameter | Value | Meaning |
|-----------|-------|---------|
| kP | 3750 | A/rotation error |
| kD | 50 | A·s/rotation |
| kS | 0.1 | A static friction |
| kV | 1.94 | A/RPS |
| kI | 0 | — |
| kA | 0 | — |

Source: COMP_TunerConstants.java:25-33

### CANcoder Offsets (COMP)
| Module | Offset (rotations) |
|--------|--------------------|
| Front Left | +0.139404 |
| Front Right | −0.123779 |
| Back Left | −0.373047 |
| Back Right | −0.212646 |

Source: COMP_TunerConstants.java:151-184

### ALPHA Robot Comparison
The ALPHA (practice) robot has a different gear set:
- Drive ratio: 6.122:1, steer ratio: 21.43:1, max speed: 5.04 m/s
- Steer closed loop: **Voltage** (not TorqueCurrentFOC)
- CAN bus: default `""` (roboRIO CAN, not CANivore)
- Slip current: 120 A (double the COMP value — not tuned for ALPHA)

Source: ALPHA_TunerConstants.java

### PathPlanner Motion Constraints (defaults)
| Parameter | Value | Units | Source |
|-----------|-------|-------|--------|
| Max velocity | 4.0 | m/s | settings.json:16 |
| Max acceleration | 5.0 | m/s² | settings.json:17 |
| Max angular velocity | 540.0 | °/s (9.42 rad/s) | settings.json:18 |
| Max angular acceleration | 1020.0 | °/s² (17.8 rad/s²) | settings.json:19 |
| Nominal voltage | 12.0 | V | settings.json:20 |

### PathPlanner Controller Gains
| Controller | kP | kI | kD | Source |
|------------|-----|-----|-----|--------|
| Translation | 5.0 | 0.0 | 0.0 | Drive.java:130 |
| Rotation | 5.0 | 0.0 | 0.0 | Drive.java:130 |

---

## 4. Electrical Characteristics

### CAN Bus Architecture
Two CAN buses are used:

**CANivore bus (`"Canivore"`)** — Drivetrain only (CAN FD, 250 Hz odometry)
| Device | Type | CAN ID |
|--------|------|--------|
| Pigeon 2 gyroscope | IMU | 0 |
| Back Left Drive | TalonFX | 1 |
| Back Left Steer | TalonFX | 2 |
| Back Left CANcoder | CANcoder | 3 |
| Front Left Drive | TalonFX | 4 |
| Front Left Steer | TalonFX | 5 |
| Front Left CANcoder | CANcoder | 6 |
| Front Right Drive | TalonFX | 7 |
| Front Right Steer | TalonFX | 8 |
| Front Right CANcoder | CANcoder | 9 |
| Back Right Drive | TalonFX | 10 |
| Back Right Steer | TalonFX | 11 |
| Back Right CANcoder | CANcoder | 12 |

**roboRIO CAN bus (`"rio"`)** — All mechanisms
| Device | Type | CAN ID | Subsystem |
|--------|------|--------|-----------|
| Flywheel Leader | TalonFX | 30 | Flywheel |
| Flywheel Follower 1 | TalonFX | 31 | Flywheel |
| Flywheel Follower 2 | TalonFX | 32 | Flywheel |
| Flywheel Follower 3 | TalonFX | 33 | Flywheel |
| Flywheel Follower 4 | TalonFX | 34 | Flywheel |
| Hood Motor | TalonFX | 35 | Hood |
| Upper Feeder | TalonFX | 36 | Upper Feeder |
| Prestage Follower | TalonFX | 37 | Prestage |
| Prestage Leader | TalonFX | 38 | Prestage |
| Lower Feeder | TalonFX | 39 | Lower Feeder |
| Transport | TalonFX | 40 | Transport |
| Intake Pivot Motor | TalonFX | 41 | Intake Pivot |
| Intake Roller Leader | TalonFX | 42 | Intake Roller |
| Intake Roller Follower | TalonFX | 43 | Intake Roller |
| Intake Pivot CANcoder | CANcoder | 44 | Intake Pivot |
| Hood CANcoder | CANcoder | 50 | Hood |

Source: HardwareConstants.java:18-46

**Total: 29 devices** (13 on CANivore + 16 on RIO CAN)

### Estimated Peak Current Draw
| Subsystem | Supply Limit | Motors | Peak (A) |
|-----------|-------------|--------|----------|
| Drive (4 × drive) | 70 A each | 4 TalonFX | 280 A |
| Drive (4 × steer) | 40 A each | 4 TalonFX | 160 A |
| Flywheel | 40 A each | 5 TalonFX | 200 A |
| Hood | 40 A | 1 TalonFX | 40 A |
| Intake Pivot | 50 A | 1 TalonFX | 50 A |
| Intake Roller | 40 A each | 2 TalonFX | 80 A |
| Transport | 40 A | 1 TalonFX | 40 A |
| Prestage | 40 A each | 2 TalonFX | 80 A |
| Upper Feeder | 40 A | 1 TalonFX | 40 A |
| Lower Feeder | 40 A | 1 TalonFX | 40 A |
| **Theoretical Maximum** | | **22 TalonFX** | **≈1010 A** |

In practice, the typical simultaneous draw during shooting is substantially lower: drivetrain at moderate load (~100-200 A), flywheel spinning (~40-80 A), feeders/transport (~20-40 A). Brownout risk exists during aggressive acceleration of all drive motors simultaneously with a full flywheel spin-up.

---

## 5. Vision Characteristics

### Camera Placement
| Camera | Name | X (forward) | Y (left) | Z (up) | Pitch | Yaw |
|--------|------|-------------|----------|--------|-------|-----|
| Camera 0 | `RobotRight` | +1.0 in | −12.171 in | +6.438 in | −15° (downward) | 270° (right-facing) |
| Camera 1 | `RobotLeft` | +1.0 in | +12.421 in | +6.438 in | −15° (downward) | 90° (left-facing) |
| Camera 2 | `ShooterRight` | −12.572 in | −6.125 in | +12.509 in | −15° (downward) | 180° (rear-facing) |
| Camera 3 | `ShooterLeft` | −12.572 in | +5.375 in | +12.509 in | −15° (downward) | 180° (rear-facing) |

Source: VisionConstants.java:39-80

Camera 0/1 are mounted at the front of the robot at 6.4 inches height, facing sideways. Camera 2/3 are mounted at the rear at 12.5 inches height, facing backward toward the hub. The −15° pitch means all cameras tilt slightly downward.

### AprilTag Layout
`AprilTagFields.k2026RebuiltWelded` — 2026 Rebuilt (welded) field
Source: VisionConstants.java:24

### Vision Filtering Parameters
| Parameter | Value | Units | Purpose | Source |
|-----------|-------|-------|---------|--------|
| Max single-tag ambiguity | 0.2 | dimensionless | Reject ambiguous single-tag PnP solves | VisionConstants.java:87 |
| Max Z error | 2.0 | m | Reject physically impossible pose heights | VisionConstants.java:90 |
| Floor error (min Z) | 6 in (0.152 m) | m | Reject below-floor poses | VisionConstants.java:93 |
| Max tag distance | 6.0 | m | Reject low-quality long-range detections | VisionConstants.java:97 |
| Max angular velocity | 4.0 | rad/s | Reject all vision when spinning fast | VisionConstants.java:101 |
| Max pitch/roll | 25° (0.436 rad) | rad | Reject tilted poses (wrong solve) | VisionConstants.java:106 |

### Standard Deviation Parameters
| Parameter | Value | Units | Source |
|-----------|-------|-------|--------|
| Linear std dev baseline | 0.03 | m (at 1 m, 1 tag) | VisionConstants.java:115 |
| Angular std dev baseline | 0.03 | rad (at 1 m, 1 tag) | VisionConstants.java:116 |
| Single-tag multiplier | 4.0× | — | VisionConstants.java:122 |
| MegaTag 2 linear factor | 0.5× | — (more trusted) | VisionConstants.java:135 |
| MegaTag 2 angular factor | ∞ | — (no rotation data) | VisionConstants.java:136-137 |

Std devs scale as `baseline × (distance² / tagCount)`. At 3 m with a single tag, linear std dev = 0.03 × 9 × 4 = 1.08 m — very low trust at long range.

### Pose Estimation
Single `SwerveDrivePoseEstimator` in Drive.java, shared with RobotState via a supplier reference. Vision measurements are added via `Drive.addVisionMeasurement()`. The dual-estimator divergence bug (from a prior version) was eliminated by wiring RobotState to Drive's estimator.

Source: Drive.java:102, 158; RobotState.java:84

---

## 6. Mechanism Characteristics

### 6a. Flywheel (Shooter)

| Property | Value | Units | Source |
|----------|-------|-------|--------|
| Motor count | 5 (1 leader + 4 followers) | — | FlywheelIOPhoenix6.java:42-101 |
| Motor type | TalonFX (Kraken X60 FOC) | — | FlywheelConstants.java:152 |
| CAN bus | `"rio"` | — | FlywheelIOPhoenix6.java:44 |
| CAN IDs | 30 (leader), 31, 32, 33, 34 | — | HardwareConstants.java:20-24 |
| Follower config | 1, 2 = Aligned; 3, 4 = Opposed | — | FlywheelIOPhoenix6.java:107-110 |
| Neutral mode | Coast | — | FlywheelIOPhoenix6.java:196 |
| Gear ratio (sensor→mechanism) | 36/24 = 1.5 : 1 | — | FlywheelConstants.java:110 |
| Flywheel wheel diameter | 4 inches (0.1016 m) | — | FlywheelConstants.java:111 |
| Control mode | MotionMagicVelocityTorqueCurrentFOC | — | FlywheelIOPhoenix6.java:55-56 |
| Supply limit | 40 A | A | FlywheelConstants.java:39 |
| Supply trigger | 35 A after 1 s | A | FlywheelConstants.java:40-41 |
| Stator limit | 45 A | A | FlywheelConstants.java:43 |
| Min RPM | 100 | RPM | FlywheelConstants.java:29 |
| Max RPM | 5600 | RPM | FlywheelConstants.java:30 |
| Max acceleration | 160 | RPS/s | FlywheelConstants.java:31 |
| MotionMagic acceleration | 100 | RPS/s² | FlywheelConstants.java:79 |
| Spin-up threshold | 200 | RPM below target | HardwareConstants.java:107 |
| Spin-up timeout | 0.5 | s | HardwareConstants.java:98 |
| Idle velocity | 1200 | RPM | HardwareConstants.java:68 |
| Idle velocity (high) | 60 | RPM | HardwareConstants.java:70 |
| Spit velocity | 17 | RPS (1020 RPM) | HardwareConstants.java:78 |
| Tower shot velocity | 1625 | RPM | HardwareConstants.java:190 |
| Pass velocity | 2700 | RPM | HardwareConstants.java:195 |

**Distance-to-RPM lookup table** (FlywheelConstants.java:129-138):
| Distance from hub | Required RPM |
|-------------------|-------------|
| 75 in (1.905 m) | 1450 |
| 85 in (2.159 m) | 1525 |
| 110 in (2.794 m) | 1625 (tower shot) |
| 130 in (3.302 m) | 1700 |
| 145 in (3.683 m) | 1750 |
| 160 in (4.064 m) | 1875 |
| 175 in (4.445 m) | 1925 |
| 180 in (4.572 m) | 1950 |
| 190 in (4.826 m) | 1975 |

### 6b. Hood

| Property | Value | Units | Source |
|----------|-------|-------|--------|
| Motor type | TalonFX | — | HoodIOReal.java:27 |
| Motor CAN ID | 35 | — | HardwareConstants.java:28 |
| Encoder type | CANcoder | — | HoodIOReal.java:28 |
| Encoder CAN ID | 50 | — | HardwareConstants.java:29 |
| CAN bus | `"rio"` (default) | — | HoodIOReal.java (no explicit bus) |
| Neutral mode | Brake | — | HoodIOReal.java:81 |
| Feedback mode | FusedCANcoder | — | HoodIOReal.java:103 |
| Motor-to-shaft ratio | 5.33 : 1 (30T → 20T belt, note: comments say 30/20=1.5 but constant is 5.33) | — | HoodConstants.java:37 |
| Shaft-to-hood ratio | 122/12 ≈ 10.167 : 1 (12T lantern → 122T hood) | — | HoodConstants.java:27 |
| Total gear ratio | ~54.2 : 1 | — | Derived |
| Max hood angle | 234° | degrees | HoodConstants.java:18 |
| Software upper limit | 62° | degrees | HoodConstants.java:43 |
| Software lower limit | 0° | degrees | HoodConstants.java:42 |
| Supply limit | 40 A | A | HoodConstants.java:50 |
| Supply trigger | 35 A after 1 s | A | HoodConstants.java:51-52 |
| Stator limit | 20 A | A | HoodConstants.java:53 |
| Magnet offset | −0.16 | rotations | HoodConstants.java:16 |
| Control mode | MotionMagicTorqueCurrentFOC | — | HoodIOReal.java:30 |
| MotionMagic acceleration | 10 | rot/s² | HoodConstants.java:63 |
| MotionMagic cruise velocity | 1 | rot/s | HoodConstants.java:64 |

**Hood positions in use** (HardwareConstants.java):
| Shot type | Position |
|-----------|---------|
| Down (stowed) | 0° |
| Tower shot | 2.5° |
| Pass shot | 35.0° |

**Hood angle-to-distance table** (HoodConstants.java HoodMap):
| Distance | Hood angle (degrees) |
|----------|---------------------|
| 75 in | 1.0° |
| 85 in | 1.5° |
| 110 in | 2.5° |
| 130 in | 3.5° |
| 145 in | 5.25° |
| 160 in | 11.0° |
| 175 in | 11.5° |
| 180 in | 12.25° |
| 190 in | 12.0° |

### 6c. Intake Pivot

| Property | Value | Units | Source |
|----------|-------|-------|--------|
| Motor type | TalonFX (Kraken X60 FOC) | — | IntakePivotConstants.java:78 |
| Motor CAN ID | 41 | — | HardwareConstants.java:42 |
| Encoder CAN ID | 44 | — | HardwareConstants.java:45 |
| CAN bus | `"rio"` | — | IntakePivotIOReal.java:40 |
| Neutral mode | Brake | — | IntakePivotIOReal.java:102 |
| Feedback mode | RemoteCANcoder | — | IntakePivotIOReal.java:137 |
| Gear ratio (rotor→mechanism) | 45 : 1 | — | IntakePivotConstants.java:55 |
| Software upper limit | 0.4 rotations (144°) | rotations | IntakePivotConstants.java:35 |
| Software lower limit | 0.0 rotations | rotations | IntakePivotConstants.java:37 |
| Software limits enabled | **Disabled** (false) | — | IntakePivotIOReal.java:140 |
| Gravity compensation | Arm_Cosine | — | IntakePivotIOReal.java:112 |
| Supply limit | 50 A | A | IntakePivotConstants.java:23 |
| Supply trigger | 45 A after 1 s | A | IntakePivotConstants.java:24-25 |
| Stator limit | 80 A | A | IntakePivotConstants.java:27 |
| Magnet offset | −0.58 | rotations | IntakePivotConstants.java:59 |
| Control mode | MotionMagicTorqueCurrentFOC | — | IntakePivotIOReal.java:48 |
| MotionMagic acceleration | 2.0 | rot/s² | IntakePivotConstants.java:70 |
| MotionMagic cruise velocity | 1.0 | rot/s | IntakePivotConstants.java:72 |
| Jostle current threshold | 70 | A stator | IntakePivotConstants.java:57 |

**Intake Pivot positions** (HardwareConstants.java:82-88):
| Position | Value |
|----------|-------|
| Up (stowed) | 0.3 rotations |
| Down (deployed) | 0.0 rotations |
| Jostle Up | 0.25 rotations |
| Jostle First | 0.115 rotations |
| Jostle Second | 0.16 rotations |

### 6d. Intake Roller

| Property | Value | Units | Source |
|----------|-------|-------|--------|
| Motor count | 2 (leader + follower) | — | intakeRollerIOReal.java |
| Motor CAN IDs | 42 (leader), 43 (follower) | — | HardwareConstants.java:43-44 |
| CAN bus | `"rio"` | — | intakeRollerIOReal.java:28 |
| Gear ratio | 24/11 ≈ 2.18 : 1 | — | intakeRollerConstants.java:23 |
| Supply limit | 40 A | A | intakeRollerConstants.java:12 |
| Supply trigger | 35 A after 1 s | A | intakeRollerConstants.java:13-14 |
| Stator limit | 55 A | A | intakeRollerConstants.java:15 |
| Intake voltage | 12.0 | V | HardwareConstants.java:52 |
| Agitate voltage | 3.0 | V | HardwareConstants.java:53 |
| Intake velocity (closed-loop) | 2400 | RPM | HardwareConstants.java:71 |
| Spit voltage | 10.0 | V | HardwareConstants.java:59 |

### 6e. Transport

| Property | Value | Units | Source |
|----------|-------|-------|--------|
| Motor CAN ID | 40 | — | HardwareConstants.java:39 |
| CAN bus | `"rio"` | — | TransportIOReal.java:25 |
| Gear ratio | 33/11 = 3.0 : 1 | — | TransportConstants.java:24 |
| Supply limit | 40 A | A | TransportConstants.java:12 |
| Stator limit | 40 A | A | TransportConstants.java:14 |
| Run voltage | −7.0 | V | HardwareConstants.java:51 |
| Spit voltage | +12.0 | V | HardwareConstants.java:59 |
| Run velocity (closed-loop) | −1800 | RPM | HardwareConstants.java:72 |

### 6f. Prestage (2 rollers)

| Property | Value | Units | Source |
|----------|-------|-------|--------|
| Motor CAN IDs | 38 (leader), 37 (follower) | — | HardwareConstants.java:31-32 |
| CAN bus | `"rio"` | — | PrestageIOReal.java:27 |
| Gear ratio | 24/11 ≈ 2.18 : 1 | — | PrestageConstants.java:23 |
| Supply limit | 40 A | A | PrestageConstants.java:12 |
| Stator limit | 45 A | A | PrestageConstants.java:15 |
| Run velocity | 3000 | RPM | HardwareConstants.java:65 |
| Idle velocity | 1300 | RPM | HardwareConstants.java:67 |
| Idle voltage | −1.0 | V | HardwareConstants.java:54 |
| Run voltage | 8.0 | V | HardwareConstants.java:55 |
| Spit velocity | 50 | RPS (3000 RPM) | HardwareConstants.java:76 |

### 6g. Upper Feeder

| Property | Value | Units | Source |
|----------|-------|-------|--------|
| Motor CAN ID | 36 | — | HardwareConstants.java:35 |
| CAN bus | `"rio"` | — | UpperFeederIOReal.java:25 |
| Gear ratio | 24/11 ≈ 2.18 : 1 | — | UpperFeederConstants.java:24 |
| Supply limit | 40 A | A | UpperFeederConstants.java:12 |
| Stator limit | 40 A | A | UpperFeederConstants.java:14 |
| Run velocity | −3000 | RPM | HardwareConstants.java:66 |
| Spit velocity | 50 | RPS | HardwareConstants.java:77 |

### 6h. Lower Feeder

| Property | Value | Units | Source |
|----------|-------|-------|--------|
| Motor CAN ID | 39 | — | HardwareConstants.java:36 |
| CAN bus | `"rio"` | — | LowerFeederIOReal.java:25 |
| Gear ratio | 24/11 ≈ 2.18 : 1 | — | LowerFeederConstants.java:25 |
| Supply limit | 40 A | A | LowerFeederConstants.java:12 |
| Stator limit | 35 A | A | LowerFeederConstants.java:14 |

---

## 7. Autonomous Characteristics

### Available Autonomous Routines
| Auto Name | Inferred Description |
|-----------|---------------------|
| `2.5-Left-Comp` | **Default**. 2.5-ball left-side competition auto |
| `2.5-Right-Comp` | 2.5-ball right-side competition auto |
| `Champs-Left` | Championship left-side auto |
| `Champs-Right` | Championship right-side auto |
| `Champs-Safe-Left` | Conservative championship left auto |
| `Champs-Safe-Right` | Conservative championship right auto |
| `Champs-Follow-Left` | Left follower pattern (defense-oriented?) |
| `Champs-Follow-Right` | Right follower pattern |
| `Center-Bump` | Center start, bump zone path |
| `Left-Double` | Left start, 2-ball path |
| `Right-Double` | Right start, 2-ball path |
| `Left-Depot` | Left start, depot pickup |
| `Left-Disruptor` | Left start, disruptor strategy |
| `Right-Disruptor` | Right start, disruptor strategy |
| `Safe-2-Left-Comp` | Conservative 2-ball left |
| `Safe-2-Right-Comp` | Conservative 2-ball right |
| `State-Elims` | State championship eliminations auto |
| `qual2`, `Qual55` | Specific qualifying match autos |

Source: `src/main/deploy/pathplanner/autos/`

### Path Folders
`Center-Paths`, `Comp-Paths`, `Left-Paths`, `Champs-Paths`, `Right-Paths`, `States-Paths`
Source: pathplanner/settings.json:5-11

### Auto Start Position Verification
During `disabledPeriodic`, the robot continuously checks whether it is placed at the auto start pose:
- Distance tolerance: logged as `Auto/StartCheck/PositionOK`
- Rotation tolerance: logged as `Auto/StartCheck/RotationOK`

Source: RobotContainer.java:1077-1099

### Auto Timing Constants (HardwareConstants.java)
| Constant | Value | Purpose |
|----------|-------|---------|
| Flywheel spin-up timeout | 0.5 s | Max wait for flywheel before shooting anyway |
| Alignment timeout | 1.5 s | Max wait for heading alignment before shooting regardless |
| Wait to compress | 0.85 s (auto) | Delay after path before scoring sequence |
| Wait to drop | 0.5 s | Delay in scoring |

---

## 8. Performance Constraints

### Speed and Acceleration
| Constraint | Value | Units | Source |
|------------|-------|-------|--------|
| Max drive speed (free) | 4.39 | m/s | COMP_TunerConstants.java:93 |
| PathPlanner default max vel | 4.0 | m/s | settings.json:16 |
| PathPlanner default max accel | 5.0 | m/s² | settings.json:17 |
| Max angular velocity | 9.42 | rad/s (540°/s) | settings.json:18 |
| Max angular acceleration | 17.8 | rad/s² (1020°/s²) | settings.json:19 |
| Vision rejection spin threshold | 4.0 | rad/s (~229°/s) | VisionConstants.java:101 |

### Alignment Tolerances
| Mode | Tolerance | Source |
|------|-----------|--------|
| Hub shot (tight) | ±1.5° | HardwareConstants.java:110 |
| Hub shot (loose, keep shooting) | ±7.0° | HardwareConstants.java:111 |
| Pass shot | ±7.0° | HardwareConstants.java:114 |
| Pass shot (loose) | ±7.0° | HardwareConstants.java:115 |

### Shooting Range
From the distance-RPM table: minimum distance ~75 inches (1.905 m), maximum characterized ~190 inches (4.826 m). The `ShotCalculator` interpolates between points and clamps to [100 RPM, 5600 RPM].

---

## 9. Control Loop Tuning

### All Mechanism Gains (Real Robot)

| Mechanism | Controller | kS | kV | kP | kD | kG | Notes |
|-----------|------------|-----|-----|-----|-----|-----|-------|
| Flywheel | MotionMagicVelocityTCFOC | 8.0 A | 0.12 A/RPS | 15.0 A/RPS | 0 | — | All 5 motors same gains |
| Hood | MotionMagicTCFOC | 9 A | — | 4000 A/rot | 0 | — | Very high kP for position |
| Intake Pivot | MotionMagicTCFOC | — | — | 750 A/rot | 5 A·s/rot | 8 A | Arm_Cosine gravity comp |
| Intake Roller | MotionMagicVelocityTCFOC | 1.5 A | 0 | 0 | 0 | — | FF-only, no PID |
| Transport | MotionMagicVelocityTCFOC | 5.0 A | 0 | 4.2 A/RPS | 0 | — | |
| Prestage | MotionMagicVelocityTCFOC | 8.0 A | 0 | 8.0 A/RPS | 0 | — | |
| Upper Feeder | MotionMagicVelocityTCFOC | 2.0 A | 0 | 13.0 A/RPS | 0 | — | |
| Lower Feeder | MotionMagicVelocityTCFOC | 2.0 A | 0 | 16.0 A/RPS | 0 | — | LF has highest kP |

Sources: FlywheelConstants.java, HoodConstants.java, IntakePivotConstants.java, intakeRollerConstants.java, TransportConstants.java, PrestageConstants.java, UpperFeederConstants.java, LowerFeederConstants.java

### MotionMagic Profiles

| Mechanism | Cruise Velocity | Acceleration |
|-----------|-----------------|--------------|
| Flywheel | (none — velocity control) | 100 RPS/s² |
| Hood | 1 rot/s | 10 rot/s² |
| Intake Pivot | 1 rot/s | 2 rot/s² |
| Transport | (velocity control only) | 120 RPS/s² |
| Prestage | (velocity control only) | 80 RPS/s² |
| Upper Feeder | (velocity control only) | 120 RPS/s² |
| Lower Feeder | (velocity control only) | 120 RPS/s² |
| Intake Roller | (velocity control only) | 100 RPS/s² |

### Swerve Drive Tuning Notes
- COMP robot uses **TorqueCurrentFOC** for both drive and steer — units for gains are Amps, not volts.
- Drive kS=3.18 A is the torque needed to overcome friction (high, tuned for Kraken X60 FOC).
- Steer kP=3750 A/rotation with kD=50 A·s/rotation is very aggressive — appropriate for FusedCANcoder accuracy.
- Slip current 60 A was tuned from 65 A (comment in code) — moderate for a robot this mass.

---

## 10. Current Limiting Strategy

### Strategy Summary
All mechanisms use **supply current limiting** (limits CAN/battery draw) plus **stator current limiting** (limits torque/heat). Drive steer motors use stator limiting only (low torque requirement). Drive motors use supply limiting only.

| Subsystem | Supply Limit | Supply Trigger | Stator Limit | Notes |
|-----------|-------------|----------------|--------------|-------|
| Drive (per motor) | 70 A | — | — | 4 motors = 280 A max drive supply |
| Steer (per motor) | 40 A | — | 40 A | Low since azimuth is low-torque |
| Flywheel (per motor) | 40 A | 35 A @ 1 s | 45 A | 5 motors = 200 A max supply |
| Hood | 40 A | 35 A @ 1 s | 20 A | Low stator because hood is lightly loaded |
| Intake Pivot | 50 A | 45 A @ 1 s | 80 A | Highest stator limit — heavy arm with gravity |
| Intake Roller | 40 A | 35 A @ 1 s | 55 A | Two motors |
| Transport | 40 A | 35 A @ 1 s | 40 A | Equal supply/stator |
| Prestage | 40 A | 35 A @ 1 s | 45 A | Two motors |
| Upper Feeder | 40 A | 35 A @ 1 s | 40 A | — |
| Lower Feeder | 40 A | 35 A @ 1 s | 35 A | Lowest stator limit |

The "supply trigger" pattern (supply → lower limit after time) is used by most mechanisms to allow brief overcurrent during spin-up while preventing sustained high draw.

### Brownout Risk Assessment
FRC circuit breaker trips at 120 A (40 A main breaker = typical 120 A trip for robot) and PDP/PDH fuses at 40 A per port.

A full-intensity scenario (drive sprinting + flywheel spin-up + intake lowering):
- Drive: 4 × 70 A = 280 A supply
- Flywheel: 5 × 40 A = 200 A supply
- Intake Pivot: 50 A

This exceeds practical battery capacity. In practice the 40 A PDP fuses per circuit + 120 A main limit total draw. The code does not currently implement dynamic current budgeting, but the trigger supply limits reduce sustained overcurrent.

---

## 11. Logging Architecture

See [logging-schema.md](logging-schema.md) for the complete signal catalog.

### Summary
| Component | Count | Mechanism |
|-----------|-------|-----------|
| @AutoLog IO classes | 11 | processInputs |
| processInputs call sites | 18 | All subsystem periodics + LocalADStarAK |
| @AutoLogOutput methods | 7 | RobotState singleton |
| recordOutput call sites | ~80 unique keys | Commands, subsystems, BatteryLogger |
| Logger.recordMetadata keys | 6 | ProjectName, BuildDate, GitSHA, GitDate, GitBranch, GitDirty |
| NT-backed logged objects | 2 | LoggedDashboardChooser (auto), LoggedNetworkNumber (tuning RPM) |

### Data Receivers by Mode
| Mode | Receiver 1 | Receiver 2 |
|------|-----------|-----------|
| REAL | WPILOGWriter (USB `/U/logs`) | NT4Publisher |
| SIM | NT4Publisher | — |
| REPLAY | WPILOGReader (source log) | WPILOGWriter (`_sim` suffix) |

Source: Robot.java:62-81

---

## 12. Simulation Support

All subsystems have simulation IO implementations:

| Subsystem | Sim Class | Physics Model | Motor Model |
|-----------|-----------|---------------|-------------|
| Drivetrain drive | ModuleIOSim | DCMotorSim | Kraken X60 FOC (1 per module) |
| Drivetrain steer | ModuleIOSim | DCMotorSim | Kraken X44 FOC (1 per module) |
| Gyro | GyroIOPigeon2 (using sim state) | — | — |
| Flywheel | FlywheelIOSim | DCMotorSim + BatterySim | Kraken X60 FOC × 5 |
| Hood | HoodIOSim | DCMotorSim | (inferred) |
| Intake Pivot | IntakePivotIOSim | SingleJointedArmSim | Kraken X60 FOC × 1 |
| Intake Roller | intakeRollerIOSim | DCMotorSim | Kraken X60 FOC × 2 |
| Transport | TransportIOSim | DCMotorSim | Kraken X60 FOC × 1 |
| Prestage | PrestageIOSim | DCMotorSim | Kraken X60 FOC × 1 per side |
| Upper Feeder | UpperFeederIOSim | DCMotorSim | Kraken X60 FOC × 1 |
| Lower Feeder | LowerFeederIOSim | DCMotorSim | Kraken X60 FOC × 1 |
| Vision | VisionIOPhotonVision | PhotonVision sim | — |

### Simulation-Specific Notes
- Flywheel uses BatterySim to model voltage sag under load
- Intake Pivot uses SingleJointedArmSim with gravity simulation enabled
- Drive sim uses separate KP/KV gains from real robot (ModuleIOSim.java:29-35)
- All CTRE TalonFX objects are created in sim using the same CAN IDs as real hardware — CTRE firmware runs natively in simulation
- The constants files provide separate sim gain sets (`Sim.KS`, `Sim.KP`, etc.) and the getters switch automatically based on `Constants.currentMode`

### Simulation Moments of Inertia
| Mechanism | MOI (kg·m²) |
|-----------|------------|
| Flywheel (combined 5-motor) | 0.01 |
| Intake Pivot arm | 0.01 |
| Intake Roller | 0.001 |
| Transport | 0.001 |
| Prestage roller | 0.001 |
| Upper Feeder | 0.001 |
| Lower Feeder | 0.001 |
| Drive (steer per module) | 0.01 |
| Drive (drive per module) | 0.01 |

---

## 13. Replay Support

AdvantageKit replay is supported on all subsystems that use `@AutoLog` + `Logger.processInputs`. Replay reads from a `.wpilog` file and re-runs all robot code deterministically.

### Replay-Critical Signals
| Required Group | Why |
|----------------|-----|
| `Drive/Module{0-3}/odometry*` arrays | High-frequency Phoenix odometry at 250 Hz |
| `Drive/Gyro/odometry*` arrays | Gyro fused with wheels for rotation |
| `Vision/Camera{0-3}/poseObservations` | Vision measurements fused into estimator |
| `LocalADStarAK/*` | PathPlanner A* decisions |
| All other `*IOInputs` fields | AKit replay contract |

### Replay Configuration
To replay:
1. Set `Constants.simMode = Mode.REPLAY` in Constants.java
2. Place the `.wpilog` file where `LogFileUtil.findReplayLog()` can find it (typically a fixed path or USB)
3. The replay output writes to `originalname_sim.wpilog`
4. `setUseTiming(false)` is set in Robot.java — replay runs as fast as the CPU allows

Source: Robot.java:74-80

---

## 14. Robot Strengths and Weaknesses (Inferred)

### Likely Strengths
- **High-fidelity pose estimation** — 4 cameras with 250 Hz Phoenix odometry and multi-camera MegaTag 2 support. Vision is filtered aggressively to avoid bad solves.
- **Distance-adaptive shooting** — The flywheel and hood both use interpolating distance maps, enabling auto-adjustment based on pose estimate.
- **Full simulation support** — Every mechanism has a physics sim. Development and testing can proceed without hardware.
- **Comprehensive logging** — 29+ log streams with full AdvantageKit replay support. Post-match debugging is strong.
- **CANivore isolation** — Drive electronics on a dedicated CANivore bus at 250 Hz, completely separate from mechanism CAN. Drivetrain communication is reliable regardless of mechanism CAN load.
- **Autonomous variety** — 18+ named autos across 6 path folders, with start-pose verification.
- **Power monitoring** — BatteryLogger tracks per-subsystem current, power, and cumulative energy in real time.
- **Shooting while aligned** — Auto-aim with a 1.5° hub tolerance, 1.5 s timeout before shooting anyway.

### Likely Weaknesses / Risks
- **CAN bus risk (RIO CAN)** — 16 devices on the RIO CAN bus. All mechanism motors, both CANcoders (hood, intake pivot), and the Pigeon 2 are NOT on this bus, but 16 signals is still busy. The flywheel broadcasts 5 motors × 4 signals at 50 Hz = 20 signals per 20 ms loop. `optimizeBusUtilization()` is called on all devices, which helps.
- **Flywheel follower current** — 5 TalonFX on flywheel with 40 A supply each = 200 A peak theoretical. If all spin up simultaneously from 0 during auto, this is a significant battery load alongside 4 drive motors also accelerating.
- **Hood software limit is narrower than mechanical** — `HoodConstants.SoftwareConstants.softwareUpperLimit = 62°` but `hoodMaxPos = 234°`. This suggests the software limit is significantly conservative or the mechanism doesn't use its full range.
- **Intake pivot software limits disabled** — `ForwardSoftLimitEnable = false` in IntakePivotIOReal.java. The software limits are configured but not enabled. An encoder failure or zeroing error could allow the intake to drive past its physical limits.
- **`RobotState/shooting` never cleared** — A logging bug means post-first-shot the flag is always `true`. Not a performance issue but confusing during match analysis.
- **Vision angular filter is permissive** — maxAngularVelocityRadPerSec = 4.0 rad/s (~229°/s) is quite high. At 4 rad/s the robot is spinning rapidly; vision estimates at that speed may have significant motion-blur error.
- **Single pose estimator** — Correct approach (no dual-estimator bug), but if Drive disconnects or is reconstructed, RobotState's poseSupplier would point to stale data until re-wired.
- **No wheel slip detection** — Despite having `driveVelocityRadPerSec` vs. setpoints in the log, no derived slip boolean or alert exists. Slipping can go undetected during match analysis unless manually compared in AdvantageScope.
- **Pass shot RPM table uses guesses** — `FlywheelConstants.java:141` comment explicitly says "Passing numbers are guesses." Pass accuracy may be inconsistent.
