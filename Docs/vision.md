# Vision

---

## System Overview

4 PhotonVision cameras perform AprilTag detection. Estimated robot poses are
fed into the drivetrain's `SwerveDrivePoseEstimator` as vision measurements,
improving localization accuracy especially when wheel odometry drifts.

All vision code: `src/main/java/frc/robot/subsystems/vision/`

---

## Architecture

```
VisionIOPhotonVision (x4)       Wraps PhotonCamera, reads pipeline results
         |
         v
VisionIOInputsAutoLogged        AdvantageKit-logged inputs struct per camera
         |
         v
Vision.periodic()               Filters observations, computes pose estimates
         |
         v
consumer.accept(pose, ts, stdDev)    Callback = drive::addVisionMeasurement
         |
         v
SwerveDrivePoseEstimator         Fuses vision with wheel/gyro odometry
```

Vision does not hold a reference to Drive. The dependency is inverted via a
`VisionConsumer` functional interface passed at construction time:

```java
vision = new Vision(
    drive::addVisionMeasurement,
    new VisionIOPhotonVision(camera0Name, robotToCamera0),
    new VisionIOPhotonVision(camera1Name, robotToCamera1),
    new VisionIOPhotonVision(camera2Name, robotToCamera2),
    new VisionIOPhotonVision(camera3Name, robotToCamera3)
);
```

---

## Camera Mounts

All transforms defined in `VisionConstants.java` as `Transform3d` (meters, radians).
All cameras pitched −15° (lens angled upward toward field AprilTags).

| Camera | Name | Forward | Left | Up | Pitch |
|--------|------|---------|------|-----|-------|
| 0 | RobotRight | 1.0" | −12.17" | 6.44" | −15° |
| 1 | RobotLeft | 1.0" | +12.42" | 6.44" | −15° |
| 2 | ShooterRight | −13.58" | −6.13" | 12.53" | −15° |
| 3 | ShooterLeft | −13.58" | +6.13" | 12.53" | −15° |

Front cameras (0, 1) are on the intake side. Rear cameras (2, 3) are on the
shooter side. This layout provides wide coverage and redundancy.

---

## IO Abstraction

`VisionIO` interface (in `vision/io/VisionIO.java`):

```java
@AutoLog
public static class VisionIOInputs {
  // Per-camera: latest pipeline results (targets, timestamps, poses)
}
public default void updateInputs(VisionIOInputs inputs) {}
```

Implementations:
- `VisionIOPhotonVision` — real hardware, reads from `PhotonCamera` over NetworkTables
- `VisionIOPhotonVisionSim` — simulation, uses `PhotonCameraSim` with a virtual field

---

## Observation Filtering

Applied in `Vision.periodic()` before passing observations to the pose estimator.

### 1. Angular velocity pre-filter

If the robot is rotating above `maxAngularVelocityRadPerSec`, **all** cameras are
skipped for that cycle. Fast rotation causes motion blur and unreliable tag
detection.

```java
boolean robotSpinningTooFast =
    Math.abs(RobotState.getInstance()
        .getFieldRelativeVelocity().omegaRadiansPerSecond)
    > VisionConstants.maxAngularVelocityRadPerSec;
```

### 2. Tag distance filter

If the average distance from the robot to visible tags exceeds `maxDistanceMeters`,
the observation is rejected. Far tags have larger pixel uncertainty.

### 3. Pitch/roll filter

If the estimated robot pose has a pitch or roll magnitude above
`maxPitchRollRadians`, the observation is rejected. On flat carpet, the robot
should never be meaningfully tilted; large pitch/roll indicates a bad estimate.

### 4. Standard deviation scaling

Accepted observations are passed with a `Matrix<N3, N1>` of standard deviations
(x, y, theta). These scale:
- **Higher** with distance (farther tags → less certain)
- **Lower** with more visible tags (more tags → better triangulation)

This lets the Kalman filter weight vision vs. odometry appropriately.

---

## Pose Estimation Integration

`Drive.addVisionMeasurement()` receives the filtered pose and timestamp:

```java
poseEstimator.addVisionMeasurement(
    robotPose.toPose2d(),
    timestampMicros / 1e6,  // Convert to seconds
    stdDevs
);
```

The `SwerveDrivePoseEstimator` internally uses a time-sorted buffer of wheel
odometry measurements, allowing it to retroactively apply the vision measurement
at the correct timestamp even when it arrives late.

---

## Driver Targeting Assist

`Vision.getTargetX(cameraIndex)` returns the yaw angle to the best visible target
on a specific camera. Used by drive commands for auto-alignment:

```java
DriveCommands.alignOrXForShoot(drive,
    () -> vision.getTargetX(0),  // Camera 0 angle offset
    ...
)
```

---

## AprilTag Field Layout

2026 Rebuilt layout loaded from WPILib standard JSON via
`AprilTagFieldLayout.loadField(AprilTagFields.k2026Reefscape)` (or equivalent
2026 field constant). Tags are on:
- Stage structures (IDs 1–4)
- Human player stations (IDs 5–8)
- Amp structures (IDs 9–10)

Field origin: blue alliance corner. Red alliance coordinates are automatically
mirrored via `AllianceFlipUtil`.

---

## Simulation

When `Constants.currentMode == SIM`, `VisionIOPhotonVisionSim` is used. It
takes the robot's simulated pose from `Drive.getPose()` and projects tag
visibility / poses through a virtual camera model. This allows testing the full
vision-to-odometry pipeline without physical hardware.
