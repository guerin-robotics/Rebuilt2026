# Replay Compatibility

AdvantageKit replay lets you re-run robot logic against a recorded `.wpilog`
file, producing a new log that shows what the code *would have done* with
different logic or constants. This is the primary tool for post-match debugging.

---

## How Replay Works

At runtime, every call to `Logger.processInputs("Key", inputs)` does two things:

- **Real robot / sim:** reads `io.updateInputs(inputs)` (hardware), then writes
  `inputs.*` to the log
- **Replay:** reads `inputs.*` from the replay log source (ignoring hardware),
  then re-runs all logic downstream

This means the IO layer is bypassed entirely during replay. Every decision the
robot makes must come from `inputs.*` fields — any value read directly from
hardware (or any `io.getXxx()` call in `periodic()`) would bypass the replay
source and produce undefined behavior.

---

## Replay Initialization (Robot.java:74–80)

```java
case REPLAY:
    setUseTiming(false);    // Run as fast as possible (no 20ms sleep)
    String logPath = LogFileUtil.findReplayLog();
    Logger.setReplaySource(new WPILOGReader(logPath));
    Logger.addDataReceiver(
        new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))
    );
    break;
```

- `setUseTiming(false)` disables the 20ms loop timer so replay completes in
  seconds rather than match-length minutes.
- `LogFileUtil.findReplayLog()` searches for the most recent `.wpilog` on
  connected USB drives, falling back to a hardcoded path.
- The output log is written to `{originalName}_sim.wpilog` in the same
  directory, enabling side-by-side comparison in AdvantageScope.

---

## Replay Compatibility Per Subsystem

### Pass: subsystems that are fully replay-safe

All subsystems in this codebase follow the correct pattern:

```java
@Override
public void periodic() {
    io.updateInputs(inputs);                    // IO layer fills struct
    Logger.processInputs("Key", inputs);        // AKit logs (or replays) struct
    // ALL logic reads only from inputs.*
    someField = inputs.velocity;
    if (inputs.position.gt(threshold)) { ... }
}
```

During replay, `io.updateInputs(inputs)` is called but AKit's replay system
**overwrites** `inputs.*` with recorded values before returning. So even if the
`IOReal` or `IOSim` is instantiated, its hardware reads are discarded.

Confirmed replay-safe (periodic reads only `inputs.*`):

| Subsystem | processInputs key | Confirmed inputs-only? |
|-----------|-------------------|----------------------|
| Drive (gyro) | `"Drive/Gyro"` | Yes |
| Drive (modules) | `"Drive/Module{0-3}"` | Yes |
| Vision | `"Vision/Camera{0-3}"` | Yes |
| Flywheel | `"Flywheel"` | Yes |
| Hood | `"Hood"` | Yes |
| Prestage | `"Prestage"` | Yes |
| UpperFeeder | `"Feeder/Upper"` | Yes |
| LowerFeeder | `"Feeder/Lower"` | Yes |
| Transport | `"Transport"` | Yes |
| IntakePivot | `"Intake Pivot"` | Yes |
| intakeRoller | `"Intake Roller"` | Yes |

### Pass: RobotState is replay-safe

`RobotState.getEstimatedPose()` reads from `poseSupplier.get()`, which is
`drive::getPose`. `Drive.getPose()` returns `poseEstimator.getEstimatedPosition()`,
which is updated exclusively from `inputs.*` values (module positions and gyro
angles). No hardware reads bypass the inputs struct.

### Pass: setUseTiming(false) is set

`Robot.java:76` correctly disables loop timing in REPLAY mode. Without this,
replay would take as long as the original match.

### Pass: Output log is written

`Robot.java:79` adds a `WPILOGWriter` output receiver in REPLAY mode. This
produces the `_sim` log for comparison.

### Pass: No IO calls in periodic() that bypass inputs

No subsystem calls `io.getSomething()` directly in `periodic()`. All state reads
go through `inputs.*`. The IO layer is only used for setpoints (output calls
like `io.setVoltage()`, `io.setVelocity()`), which are safe to re-execute during
replay because they don't affect the input side.

---

## Replay Limitations

### 1. Output setpoints re-execute against real hardware

If you run replay on the actual robot (rare, but possible), the motor setpoint
calls (`io.setVelocity(...)`) will execute against real hardware using the
replayed logic. This is almost never done — replay is typically run in sim mode
where the IO layer is a stub or sim implementation.

To avoid this: use `Constants.Mode.REPLAY` which in `RobotContainer` selects
the stub IO (`new SubsystemIO() {}`) for all subsystems. With stub IO, output
calls are no-ops.

### 2. RobotState hub-shift values come from Robot.teleopPeriodic()

`Robot.java:230–233` logs three values to AdvantageKit in `teleopPeriodic()`:

```java
Logger.recordOutput("RobotState/HubShift", HubShiftUtil.isHubShiftActive());
Logger.recordOutput("RobotState/firstActiveAlliancer", ...);
Logger.recordOutput("RobotState/timeRemainingInShift", ...);
```

These are logged *after* they are computed — they are outputs, not inputs. In
replay, `HubShiftUtil.isHubShiftActive()` is called fresh each replay cycle
from the robot clock (which in replay is sourced from the replay log). Whether
this is exactly correct depends on `HubShiftUtil`'s internal implementation —
if it has internal state that depends on real wall-clock time rather than the
match timer, it may compute different values during replay than the original run.

**Risk:** Medium — if hub shift timing differs in replay, `isShootSafeTime`
and `isShootClear` triggers may evaluate differently, changing shoot command
activation timing in the replayed log.

### 3. Vision simulation state is not replayed

`VisionIOPhotonVisionSim` is selected in SIM mode, not REPLAY mode. In REPLAY
mode, the stub IO `VisionIO {}` is used, which returns no observations. However,
because `Vision.periodic()` calls `Logger.processInputs("Vision/Camera{i}",
inputs[i])`, the replay source overwrites `inputs[i]` with the recorded camera
observations. So vision *observations* are correctly replayed — the sim
`VisionSystemSim` is not involved.

### 4. SmartDashboard auto delay not replayed

`RobotContainer.getAutonomousCommand()` reads `SmartDashboard.getNumber(autoDelayKey, 0.0)`. SmartDashboard reads are not logged by AdvantageKit (they go through WPILib's NT layer, not through `Logger.processInputs`). If the auto delay was nonzero during the match, replay will use 0 seconds.

**Workaround:** Log the delay at autonomous init with `Logger.recordOutput("Auto/Delay", delay)` and compare against replay behavior.

### 5. LoggedNetworkNumber is replay-safe but NetworkTable values are live

`LoggedNetworkNumber` (used for `"Tune/flywheel/tuningRPM"`) wraps the NT
value through AdvantageKit's logging pipeline, so it **is** replayed correctly.
The value recorded in the original log is used during replay.

Plain `SmartDashboard.getNumber()` calls are NOT logged and would read from
whatever value is currently in NT during replay (typically 0 or default).

---

## Running a Replay

1. Copy a match `.wpilog` from the USB drive to your development machine.
2. Set `Constants.currentMode = Mode.REPLAY` in `Constants.java`.
3. Place the `.wpilog` file where `LogFileUtil.findReplayLog()` will find it
   (project root, or a USB drive attached to the dev machine).
4. Build and run the code (on a roboRIO in replay mode, or in simulation via
   WPILib simulation with `gradlew simulateJava`).
5. The replay will run at full speed (no 20ms sleep).
6. Open both `original.wpilog` and `original_sim.wpilog` in AdvantageScope
   for side-by-side comparison.

---

## Replay-Assisted Debugging Workflow

### "Why didn't we shoot?"

```
1. Open match log in AdvantageScope
2. Find the timestamp when shoot was expected
3. Check "RobotState/IsAlignedToHub" — was it true?
4. Check "isFlywheelSpunUp" — was the flywheel up to speed?
5. Check "RobotState/IsAlignedToPass" vs zone — was zone correct?
6. If a trigger was stuck: check "BatteryLogger" for voltage sag
   (low battery → brownout → CAN gaps → stale inputs)
```

### "Why did the shot miss?"

```
1. Find shot timestamp in log
2. Check "RobotState/DistanceToAllianceHub_m" at shot moment
3. Check "Flywheel/targetRPM" vs "Flywheel/ShooterIOInputs/leaderVelocity"
   (were we actually at speed?)
4. Check "Hood/HoodIOInputs/hoodPosition" vs "hoodClosedLoopReference"
   (was hood at target angle?)
5. Check "Odometry/Robot" — was our pose estimate accurate?
   (compare accepted vs rejected vision poses)
```

### "Why did auto fail at this step?"

```
1. Check "Odometry/Robot" vs "Odometry/Trajectory" at failure point
   (pose error too large → path deviation)
2. Check "Auto/StartCheck/PositionOK" (was robot placed correctly?)
3. Check "Vision/Camera{i}/RobotPosesRejected" in auto
   (was vision working at start to initialize pose?)
```
