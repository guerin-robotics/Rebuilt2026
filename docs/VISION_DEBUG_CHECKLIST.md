# Vision Debug Checklist

Practical checklist for debugging the PhotonVision + AdvantageKit vision system.
Work top to bottom — most vision problems are configuration, not code.

The full data chain:

```
PhotonVision (Orange Pi) → NetworkTables → VisionIOPhotonVision → VisionIOInputsAutoLogged
→ Vision.periodic() filtering → std dev calculation → Drive.addVisionMeasurement()
→ SwerveDrivePoseEstimator → RobotState.getEstimatedPose() → AdvantageScope
```

---

## 1. PhotonVision UI checks (Orange Pi)

Open `http://photonvision.local:5800` (or the Orange Pi's IP, port 5800).

- [ ] PhotonVision version matches photonlib vendordep (`vendordeps/photonlib.json`, currently **v2026.3.4**). Mismatched versions silently break NetworkTables data.
- [ ] All 4 cameras appear and stream video.
- [ ] Team number is set correctly (10021) so the Pi connects to the robot's NT server.
- [ ] CPU usage is reasonable (<80%) and FPS is stable on every camera.

## 2. Camera name checks

Code expects these **exact** names (see `VisionConstants.java`):

| Index | Code name      | Location on robot              |
|-------|----------------|--------------------------------|
| 0     | `RobotRight`   | front-right, facing right      |
| 1     | `RobotLeft`    | front-left, facing left        |
| 2     | `ShooterRight` | rear-right, facing backward    |
| 3     | `ShooterLeft`  | rear-left, facing backward     |

- [ ] Names in the PhotonVision UI match exactly (case-sensitive, no spaces).
- [ ] Each physical camera is plugged into the slot its name claims — cover one lens with your hand and confirm the right stream goes dark.

## 3. AprilTag pipeline checks

For each camera in the PhotonVision UI:

- [ ] Pipeline type is **AprilTag**, family **36h11**.
- [ ] **Driver mode is OFF** (driver mode disables target processing entirely).
- [ ] **3D mode / solvePnP enabled** so pose data is published.
- [ ] **Multi-tag enabled** (code prefers the coprocessor multi-tag solve and falls back to lowest-ambiguity single-tag).
- [ ] Field layout on the Pi is the 2026 Rebuilt (welded) layout, matching `AprilTagFields.k2026RebuiltWelded` in code.
- [ ] Camera is **calibrated** at the resolution the pipeline uses (uncalibrated = no 3D pose).

## 4. Exposure / FPS / resolution checks

- [ ] Exposure is LOW (manual, short). Motion blur from auto-exposure is the #1 cause of bad tags while moving.
- [ ] Tags detect reliably at 4–5 m while the robot is moving.
- [ ] FPS ≥ 25 on each camera; latency under ~50 ms.
- [ ] Brightness/gain tuned at the venue, not just in the shop.

## 5. Camera transform measurement checklist

Transforms live in `VisionConstants.java` (`robotToCamera0..3`), measured from **robot center, floor level** to the **camera lens**, in inches:

| Camera       | X (fwd) | Y (left) | Z (up) | Pitch | Yaw  |
|--------------|---------|----------|--------|-------|------|
| RobotRight   | +1.0    | -12.171  | +6.438 | -15°  | 270° |
| RobotLeft    | +1.0    | +12.421  | +6.438 | -15°  | 90°  |
| ShooterRight | -12.572 | -6.125   | +12.509| -15°  | 180° |
| ShooterLeft  | -12.572 | +5.375   | +12.509| -15°  | 180° |

- [ ] X positive = toward intake/front, Y positive = robot left, Z positive = up.
- [ ] Pitch is **negative** for a camera tilted upward (WPILib rotates about +Y; positive pitch points the camera down).
- [ ] Yaw 90 = facing robot-left, 270 = facing robot-right, 180 = facing backward.
- [ ] Verify with a tape measure after any camera remount, and use AdvantageScope 3D view (`Vision/CameraN/robot_position`) to confirm the cameras render where they physically are.

## 6. NetworkTables checks

Use AdvantageScope's NT view, Elastic, or Glass while connected to the robot:

- [ ] `/photonvision/` table exists and contains one subtable per camera with the exact names above.
- [ ] Each camera's `heartbeat` value is incrementing (camera alive).
- [ ] `Vision/CameraN/Connected` is true in the AdvantageKit log for all 4 cameras.

## 7. AdvantageScope log checks

Key log fields (`Vision/Camera0..3`, plus summary):

| Field | What it tells you |
|-------|-------------------|
| `Vision/CameraN/RobotPoses` | Raw poses before filtering |
| `Vision/CameraN/RobotPosesAccepted` | Poses sent to the pose estimator |
| `Vision/CameraN/RobotPosesRejected` | Poses that failed a filter |
| `Vision/CameraN/RejectionReason` | Why the last observation was rejected ("" = accepted) |
| `Vision/CameraN/TagIds` / `TagCount` | Which tags are seen |
| `Vision/CameraN/Ambiguity` | Single-tag ambiguity (reject > 0.4) |
| `Vision/CameraN/AverageTagDistance` | Distance scaling input (reject > 6.0 m) |
| `Vision/CameraN/LinearStdDev` / `AngularStdDev` | Trust given to the last accepted pose |
| `Vision/Summary/AcceptedObservationCount` / `RejectedObservationCount` | Per-loop accept/reject totals |
| `Odometry/Robot` (Drive) / `RobotState/EstimatedPose` | Fused pose output |

- [ ] Overlay `RobotPosesAccepted` and the estimated pose on the 3D field — accepted poses should cluster around the robot.
- [ ] If everything is rejected, read `RejectionReason` before touching code.

## 8. On-robot test procedure

1. Robot on blocks or in open space, DS connected, robot **disabled** (vision still runs).
2. Confirm all 4 `Connected` flags are true.
3. Hold an AprilTag (or face a field tag) in front of each camera in turn; confirm `TagIds` updates for the right camera.
4. Check `RejectionReason` is empty and accepted poses appear.
5. Push the robot by hand — the estimated pose should track smoothly without jumps.

## 9. Static AprilTag test

1. Place the robot at a measured field position facing a tag (e.g., bumpers against a known line).
2. Compare `RobotState/EstimatedPose` to the tape-measured position.
3. Accept: within ~5 cm and ~2° at 2–3 m from the tag.
4. Repeat per camera (cover the others) to isolate a bad transform — a consistent offset on one camera means its `robotToCamera` is wrong.

## 10. Slow-drive field test

1. Drive slowly (~1 m/s) around the field with vision enabled.
2. Watch the estimated pose in AdvantageScope — it should move smoothly; vision corrections should be small nudges, not teleports.
3. Spin in place fast — observations should be rejected (`AngularVelocityTooHigh`) and the pose should not jump.
4. Drive far from all tags, then back into view — pose should converge smoothly.

## 11. Auto-align validation test

1. Run the auto-align / aim command from several positions and angles.
2. Confirm final alignment error is repeatable and within tolerance.
3. If alignment is consistently off in one direction, suspect a camera transform or hub pose constant, not the controller.

## 12. What bad data looks like

- **Pose teleports across the field** → ambiguity flips on a single tag; lower `maxAmbiguity` or rely on multi-tag.
- **Pose oscillates between two spots** → two cameras disagree; one transform is wrong.
- **Pose lags behind the robot** → timestamp/latency problems; check Pi clock sync and NT connection.
- **Pose drifts only when moving** → exposure too long (motion blur) or angular velocity filter too loose.
- **Z, pitch, or roll far from zero in `RobotPoses`** → bad solve or bad transform (these are rejected, but lots of them means a config problem).

## 13. How to tune std devs

In `VisionConstants.java`:

- Baselines: `linearStdDevBaseline = 0.01 m`, `angularStdDevBaseline = 0.03 rad`, scaled by `distance² / tagCount`, single-tag ×2.
- Vision too jittery (pose buzzes around)? **Increase** baselines — trust vision less.
- Vision too slow to correct odometry drift? **Decrease** baselines — trust vision more.
- One camera noisier than the rest (worse mount/calibration)? Raise its entry in `cameraStdDevFactors`.
- Tune with the log: compare `LinearStdDev` values against how much accepted poses actually scatter.

## 14. How to temporarily disable one camera

Preferred (no rewiring): raise that camera's `cameraStdDevFactors` entry to a huge value (e.g., `1e5`) — observations are still logged but effectively ignored by the estimator.

Full disable: in `RobotContainer`, replace its `new VisionIOPhotonVision(...)` with `new VisionIO() {}` (keep 4 IO entries so camera indices and replay stay consistent).

## 15. Pre-competition vision checklist

- [ ] PhotonVision version matches vendordep on all cameras.
- [ ] Field layout on Pi = `k2026RebuiltWelded` (verify the venue is welded, not AndyMark — switch both code and Pi together if needed).
- [ ] All 4 camera names match code.
- [ ] All cameras calibrated at competition resolution.
- [ ] Exposure/brightness tuned on the actual field during calibration time.
- [ ] Static tag test (section 9) passed on the practice field.
- [ ] `Connected` = true ×4 with the FMS-style network setup.
- [ ] A recent match/practice log reviewed: accepted counts healthy, rejection reasons sensible.
- [ ] Spare camera + cables in the pit; know section 14 in case one camera goes bad mid-event.
