# Hardware Layout Reference

Source of truth for physical robot hardware. Keep this updated when hardware changes.

---

## CAN Bus: CANivore (`"Canivore"`)

| CAN ID | Device | Type | Subsystem | Notes |
|---|---|---|---|---|
| 0 | Pigeon 2 | IMU | Drive | Gyro, yaw used by odometry thread |
| 1 | Back Left Drive | TalonFX (Kraken X60) | Drive | Inverted: no |
| 2 | Back Left Steer | TalonFX (Kraken X60) | Drive | Inverted: no |
| 3 | Back Left Encoder | CANcoder | Drive | Offset: −0.373046875 rot |
| 4 | Front Left Drive | TalonFX (Kraken X60) | Drive | Inverted: no |
| 5 | Front Left Steer | TalonFX (Kraken X60) | Drive | Inverted: no |
| 6 | Front Left Encoder | CANcoder | Drive | Offset: +0.139404296875 rot |
| 7 | Front Right Drive | TalonFX (Kraken X60) | Drive | Inverted: yes |
| 8 | Front Right Steer | TalonFX (Kraken X60) | Drive | Inverted: no |
| 9 | Front Right Encoder | CANcoder | Drive | Offset: −0.123779296875 rot |
| 10 | Back Right Drive | TalonFX (Kraken X60) | Drive | Inverted: yes |
| 11 | Back Right Steer | TalonFX (Kraken X60) | Drive | Inverted: no |
| 12 | Back Right Encoder | CANcoder | Drive | Offset: −0.212646484375 rot |
| 41 | Intake Pivot Motor | TalonFX | Intake Pivot | On CANivore — reason not documented |
| 44 | Intake Pivot Encoder | CANcoder | Intake Pivot | On CANivore — same note |

---

## CAN Bus: RIO (`"rio"`)

| CAN ID | Device | Type | Subsystem | Notes |
|---|---|---|---|---|
| 30 | Flywheel Leader | TalonFX (Kraken X60) | Flywheel | |
| 31 | Flywheel Follower 1 | TalonFX (Kraken X60) | Flywheel | Opposes leader |
| 32 | Flywheel Follower 2 | TalonFX (Kraken X60) | Flywheel | Opposes leader |
| 33 | Flywheel Follower 3 | TalonFX (Kraken X60) | Flywheel | Opposes leader |
| 34 | Flywheel Follower 4 | TalonFX (Kraken X60) | Flywheel | Same direction as leader |
| 35 | Hood Motor | TalonFX | Hood | |
| 36 | Upper Feeder | TalonFX | Upper Feeder | |
| 37 | Prestage Right | TalonFX | Prestage | Follower — opposes leader (ID 38) |
| 38 | Prestage Left | TalonFX | Prestage | Leader |
| 39 | Lower Feeder | TalonFX | Lower Feeder | |
| 40 | Transport | TalonFX | Transport | Open-loop only |
| 42 | Intake Roller Leader | TalonFX | Intake Roller | Open-loop only |
| 43 | Intake Roller Follower | TalonFX | Intake Roller | Same direction as leader |
| 50 | Hood CANcoder | CANcoder | Hood | Absolute position for hood angle |

---

## Vision Cameras (PhotonVision over NetworkTables)

| Name | Position (from robot center) | Facing | Purpose |
|---|---|---|---|
| `RobotRight` | X=+1.0", Y=−12.2", Z=+6.4" | Yaw 270° (right) | Front-right AprilTags |
| `RobotLeft` | X=+1.0", Y=+12.4", Z=+6.4" | Yaw 90° (left) | Front-left AprilTags |
| `ShooterRight` | X=−12.6", Y=−6.1", Z=+12.5" | Yaw 180° (rear) | Shooter-side tags |
| `ShooterLeft` | X=−12.6", Y=+5.4", Z=+12.5" | Yaw 180° (rear) | Shooter-side tags |

All cameras: Pitch −15°, Roll 0°

Camera coprocessor: Orange Pi (PhotonVision)

---

## Swerve Module Geometry

Wheel positions relative to robot center:

| Module | X | Y |
|---|---|---|
| Front Left | +11 in | +11 in |
| Front Right | +11 in | −11 in |
| Back Left | −11 in | +11 in |
| Back Right | −11 in | −11 in |

Drive gear ratio: 7.03125:1
Steer gear ratio: 26.09:1
Wheel radius: 2 in (0.0508 m)
Coupling ratio: 4.5 (drive motor rotations per steer rotation)

---

## Robot Physical Specs (PathPlanner)

| Property | Value |
|---|---|
| Mass | 63.503 kg |
| Moment of inertia | 5.162 kg·m² |
| Wheel COF | 1.2 |
| Max drive speed | ~4.39 m/s at 12V |

---

## Electrical

| Item | Notes |
|---|---|
| Log storage | USB drive at `/U/` — must be present for match logs |
| PDH | Powers all subsystems; voltage logged via `BatteryLogger` |
| RoboRIO | Main control; `"rio"` CAN bus terminates here |
| CANivore | Separate USB-CAN adapter; `"Canivore"` bus |

---

## Vendor Library Versions

| Library | Version | Config file |
|---|---|---|
| WPILib | 2026.2.1 | `build.gradle` |
| CTRE Phoenix 6 | 26.2.0 | `vendordeps/Phoenix6.json` |
| PhotonVision | v2026.3.4 | `vendordeps/photonlib.json` |
| AdvantageKit | 26.0.2 | `vendordeps/AdvantageKit.json` |
| PathPlannerLib | current | `vendordeps/PathplannerLib.json` |

Update vendor libraries at the start of each season. Do not update mid-season
unless fixing a known critical bug.
