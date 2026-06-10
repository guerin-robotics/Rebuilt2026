# Prompt: Debug a Match Log

Use this prompt when reviewing an AdvantageKit `.hoot` log file to find a problem.
Describe what went wrong in the match before asking for analysis.

---

```
Analyze this match behavior:

Match: [match number / event name]
What went wrong: [description of the problem — be specific about timing and what the robot did vs should have done]
When it happened: [e.g., "during the 3rd auto shot", "first time shoot button was pressed in teleop", "around 1:30 match time"]

In Advantage Scope, I can see:
- [key signal you observed, e.g.: "RobotState/IsAlignedToHub was false when the feeder ran"]
- [another signal if available]

Help me find the root cause by suggesting which signals to check and what the
AdvantageKit log would show if each hypothesis were true.
```

---

## Key Signals to Check for Common Problems

### Robot didn't shoot when button was pressed
```
Triggers/isShootClear              — was the composite trigger true?
Triggers/isShootSafeZone           — was robot in alliance zone?
Triggers/isShootSafeTime           — was the hub active?
RobotState/IsAlignedToHub          — was heading within tolerance?
Flywheel/[leader]/closedLoopRef    — was flywheel targeting a velocity?
Flywheel/isSpunUp                  — did spinup condition pass?
```

### Robot aligned wrong direction
```
RobotState/AngleToAllianceHub      — what heading was being targeted?
RobotState/EstimatedPose           — was the pose correct?
Drive/Odometry/Robot               — field visualization
Vision/Camera[0-3]/AcceptedPose   — were vision updates being applied?
```

### Auto ran wrong path / started from wrong position
```
Auto/SelectedAuto                  — what auto was selected?
Auto/StartPose                     — expected starting pose
Auto/StartCheck/TranslationError   — how far off was the robot?
Auto/StartCheck/RotationError      — rotation error at start
Odometry/Trajectory                — what path was being followed?
```

### Vision caused pose jump
```
Vision/Camera[N]/AcceptedPoses     — which camera sent a pose?
Vision/Camera[N]/RejectedPoses     — what was rejected and why?
RobotState/FieldRelativeVelocity   — was the robot moving fast when the jump happened?
Drive/Odometry/Robot               — visualization of the jump
```

### Brownout during match
```
PowerDistribution/Voltage          — when did voltage drop below 7V?
BatteryLogger/[Subsystem]/Current  — which subsystem drew the most current?
[Subsystem]/StatorAmps             — stall detection per subsystem
```

### Intake didn't compress
```
IntakePivot/GoalPosition           — what position was being commanded?
IntakePivot/ActualPosition         — what position was reached?
Commands/Active                    — was the compress command scheduled?
Triggers/compressCancelled         — was the cancellation flag set?
```

---

## Replay Instructions

To replay a log and add new instrumentation:
1. Set `Constants.getMode()` to `REPLAY`
2. Point `WPILOGReader` at the `.hoot` file
3. Add `Logger.recordOutput()` calls for additional values
4. Run `./gradlew simulateJava`
5. Open Advantage Scope and connect to the replay output
