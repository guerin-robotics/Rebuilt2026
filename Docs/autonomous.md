# Autonomous

Full lifecycle, scheduler interactions, pose estimation interactions, and failure
handling. All values confirmed by source reading.

---

## Auto Selection System

### Chooser Construction (RobotContainer.java:237–244)

```java
autoChooser = new LoggedDashboardChooser<>(
    "Auto Choices",
    AutoBuilder.buildAutoChooser());

autoChooser.addDefaultOption(
    HardwareConstants.CompConstants.Autos.DefaultAutoName,
    new PathPlannerAuto(HardwareConstants.CompConstants.Autos.DefaultAutoName));

SmartDashboard.putData("Auto Preview", autoPreviewField);
SmartDashboard.putNumber(autoDelayKey, defaultAutoDelay);
```

`AutoBuilder.buildAutoChooser()` scans `src/main/deploy/pathplanner/autos/`
and creates a `SendableChooser` entry for every `.auto` file found. The
`LoggedDashboardChooser` wrapper logs every selection change through
AdvantageKit so selection history appears in match logs.

19 auto routines are available. The default is set by
`HardwareConstants.CompConstants.Autos.DefaultAutoName`.

### Auto Delay

`SmartDashboard.getNumber(autoDelayKey, defaultAutoDelay)` is read at
`autonomousInit()` time inside `getAutonomousCommand()`. The key is `"Auto Delay"`,
default value is `defaultAutoDelay` (0.0 s). Drive team adjusts this on the
dashboard to stagger alliance partners.

**Important:** The auto delay is read from SmartDashboard, **not** from
AdvantageKit. It is not replayed correctly — see `docs/replay.md` for details.

### getAutonomousCommand() (RobotContainer.java:984–987)

```java
public Command getAutonomousCommand() {
    double delay = SmartDashboard.getNumber(autoDelayKey, defaultAutoDelay);
    return Commands.sequence(
        Commands.waitSeconds(delay),
        autoChooser.get().asProxy()
    );
}
```

`.asProxy()` wraps the selected `PathPlannerAuto` in a `ProxyCommand`. This
creates a composition boundary: the proxy's requirements are declared as the
union of whatever the auto group requires at schedule time, but the auto runs
in its own composition scope. This prevents the outer `sequence` from cancelling
subsystem default commands (like `hoodIdle`) during the delay phase while
nothing in the auto group is running yet.

---

## Autonomous Execution Lifecycle

### Phase 1: Disabled — preview and placement check

Every disabled cycle (Robot.java:167–168):

```
disabledPeriodic()
  ├── robotContainer.updateAutoPreview()
  └── robotContainer.checkStartPose()
```

**updateAutoPreview()** (RobotContainer.java:1001–1054):
1. Reads current auto name from `autoChooser.get().getName()`
2. Calls `PathPlannerAuto.getPathGroupFromAutoFile(autoName)` to load all path
   segments from the `.auto` file
3. If red alliance, applies `AllianceFlipUtil.flip()` to all waypoints
4. Flattens all paths into a `List<Pose2d>` of sampled poses
5. Writes to `autoPreviewField.getObject("path").setPoses(poses)` for dashboard
6. Stores `autoStartPose` from the first path's first waypoint
7. Logs: `"Auto/SelectedAuto"`, `"Auto/PreviewStatus"`, `"Auto/StartPose"`

If path loading fails (file not found, parse error), logs an error string to
`"Auto/PreviewStatus"` and leaves `autoStartPose` null. `checkStartPose()`
guards against null `autoStartPose`.

**checkStartPose()** (RobotContainer.java:1077–1099):

| Check | Tolerance | AKit key |
|-------|-----------|---------|
| Position | ≤ 6.0 inches | `"Auto/StartCheck/PositionOK"` |
| Heading | ≤ 5.0 degrees | `"Auto/StartCheck/RotationOK"` |

Values also logged: `"Auto/StartCheck/DistanceInches"`,
`"Auto/StartCheck/RotationDiffDegrees"`

The robot's current pose comes from `RobotState.getInstance().getEstimatedPose()`.
If the robot has been sitting still for a while, this pose is anchored to the
last vision observation or the last odometry position from the previous match.
The drive team should ensure the robot is placed accurately before the start
check turns green.

### Phase 2: autonomousInit() (Robot.java:173–180)

```java
autonomousCommand = robotContainer.getAutonomousCommand();
if (autonomousCommand != null) {
    CommandScheduler.getInstance().schedule(autonomousCommand);
}
```

The CommandScheduler immediately runs `initialize()` on the scheduled command.
For the `sequence` wrapper, this means the `waitSeconds(delay)` starts
counting. The auto's `initialize()` is deferred until the delay expires.

**Pose reset at auto start:** PathPlanner calls `Drive.setPose()` at the
beginning of each auto if `resetOdom: true` is set in the `.auto` file (which
it is for all 19 routines). `Drive.setPose()` calls
`poseEstimator.resetPosition(rawGyroRotation, modulePositions, pose)`, which:
1. Clears the estimator's internal history buffer
2. Seeds it with the given pose (from the path's starting waypoint)
3. Preserves the current gyro rotation to maintain heading continuity

This is the primary source of initial pose truth. Vision has typically been
running in disabled mode and may have already refined the pose, but `resetOdom`
overrides whatever the estimator had converged to. This can cause a discontinuity
if vision had a better estimate than the path's hard-coded starting pose.

### Phase 3: Auto running

After the delay, `autoChooser.get().asProxy()` runs. PathPlanner parses the
`.auto` file's command tree and constructs a WPILib `Command` object. The
scheduler runs this command's `execute()` every cycle.

**Every cycle during auto:**
```
CommandScheduler.run()
  ├── Poll all Triggers (teleop bindings still active)
  ├── Run all subsystem periodic() calls
  │     Drive.periodic()    → odometry update, vision processing
  │     Vision.periodic()   → camera observations → addVisionMeasurement
  │     Flywheel.periodic() → velocity tracking
  │     [all other subsystems]
  ├── Run active command execute() calls
  │     Auto group execute() → PathPlanner trajectory tracking
  │       → Drive.runVelocity(ChassisSpeeds) each cycle
  │     Named commands execute if scheduled
  └── Check command isFinished() conditions
```

Teleop button bindings remain registered and their Triggers continue polling.
Physical button presses cannot happen during auto, but condition-based Triggers
(zone checks, alignment checks) could theoretically fire. See `architecture/
dependency-graph.md` for the scheduler conflict risk.

### Phase 4: Shot sequence within auto

When the auto group reaches a `"Shoot"` named command:

```
ShootSequences.autoShootToHub() requires:
  Flywheel, Hood, Prestage, UpperFeeder, LowerFeeder,
  Transport, intakeRoller, IntakePivot

Timeline:
  t=0.0s  initialize(): flywheel and hood begin ramping, prestage starts
  t=0.0s  waitUntil(isFlywheelSpunUp) begins polling every cycle
  t≤0.5s  flywheel reaches target RPM → waitUntil() satisfied
  t=0.5s  (timeout fires if not spun up) → feeders engage anyway
  t+εs    feeders + transport + agitate + autoPivotCompress all start

  autoPivotCompress:
    waits autoWaitToCompressSeconds → then jostle pivot to pivotJostleUpPos
```

The `"Shoot"` named command also runs `DriveCommands.joystickDriveAtAngle`
in parallel (RobotContainer.java:311–322), keeping the robot pointed at the hub
while stationary. This requires `Drive`, which means the `Shoot` named command
also requires `Drive` and will prevent any concurrent path following.

The auto structure uses a `race` group for shots:
```json
{ "type": "race", "data": { "commands": [
    { "type": "named", "data": { "name": "Shoot" } },
    { "type": "wait", "data": { "waitTime": 2.7 } }
]}}
```
The race ends when either the `Shoot` command finishes **or** 2.7 seconds
elapses, whichever comes first. This hard timeout prevents the auto from
hanging indefinitely if the shot sequence stalls.

### Phase 5: Pose correction during auto

While any path is following, `Vision.periodic()` continues to run and
`addVisionMeasurement()` is called for every accepted observation. The
`SwerveDrivePoseEstimator` applies these retroactively at the correct timestamp.

During fast auto movements, the angular velocity filter often suppresses vision:
```java
// Vision.java:91-93
boolean robotSpinningTooFast =
    Math.abs(RobotState.getFieldRelativeVelocity().omegaRadiansPerSecond)
        > maxAngularVelocityRadPerSec;
```
When `true`, **all four cameras are skipped** for that cycle. During high-speed
path segments, vision may be suppressed for multiple consecutive cycles.

During the `Shoot` named command, the robot is stationary (joystickDriveAtAngle
targeting the hub). Angular velocity is low, so vision can recover pose estimate
quality before the shot distance calculation is read.

### Phase 6: teleopInit() transition (Robot.java:193–208)

```java
if (autonomousCommand != null) {
    autonomousCommand.cancel();
}
CommandScheduler.getInstance().schedule(robotContainer.getAutoStopCommand());
```

`autonomousCommand.cancel()` cascades: the proxy cancels the auto group, which
cancels every composed command within it, calling `end(interrupted=true)` on
each. Motor setpoints from the auto are cleared.

`getAutoStopCommand()` immediately schedules `ShootSequences.stopAll(...)` to
zero all shooter subsystem outputs. This prevents flywheel coasting from
persisting into teleop.

Default commands (`joystickDrive`, `hoodIdle`) resume as soon as their
requirements are freed.

---

## Pose Reset Flow

```
Path's starting pose (from .path file, alliance-flipped if red)
           │
           ▼ (called by AutoBuilder at auto start)
Drive.setPose(Pose2d)
           │
           ▼
poseEstimator.resetPosition(
    rawGyroRotation,        ← current gyro reading (preserves heading)
    modulePositions[],      ← current encoder positions
    pose                    ← target pose from path
)
           │
           ▼ (internal to SwerveDrivePoseEstimator)
Clears history buffer
Seeds buffer with new pose
           │
           ▼
RobotState.getEstimatedPose() → new pose takes effect immediately

```

**After reset:** Vision measurements continue streaming in. The first accepted
vision observation after the reset will Kalman-blend with the reset pose. If the
robot was placed accurately and `resetOdom: true` is reliable, vision acts as
a drift corrector. If the robot was placed poorly, the path will start wrong
and vision will fight the odometry for correction — with limited authority
because standard deviations are distance-based.

---

## Failure Handling

### Flywheel spinup timeout

```java
Commands.waitUntil(flywheel.isFlywheelSpunUp)
    .withTimeout(spinUpTimeOut)  // 0.5 s
```

If the flywheel does not reach target RPM within 0.5 seconds, the timeout
fires and the feeder commands start anyway. The shot will be under-speed. This
is a deliberate tradeoff: a slow shot is better than hanging in a shot command
for several seconds during auto.

Battery sag (heavy current draw on other subsystems simultaneously) is the most
common cause of spinup timeout. The BatteryLogger can be used to correlate.

### Path following drift

PathPlanner has no built-in pose recovery mechanism. If the robot drifts
significantly off the planned trajectory (collision, wheel slip), the path
continues from the planned trajectory — the robot just has larger tracking error.
The `PPHolonomicDriveController` will apply larger correction forces, but with
kP=5.0 there is finite correction authority.

Vision measurements help: if a camera sees tags at the deviation point, pose
will converge back toward truth and the path error will visually decrease. But
the path itself continues at its scheduled time.

No explicit fallback auto or path abort is implemented.

### Path file load failure

If `PathPlannerAuto.getPathGroupFromAutoFile()` throws during `updateAutoPreview()`:
- The exception is caught and logged to `"Auto/PreviewStatus"`
- `autoStartPose` remains null
- `checkStartPose()` skips the check (null guard)
- At `autonomousInit()`, constructing the `PathPlannerAuto` for the same name
  will throw again and the auto command will be null, skipping autonomous

The dashboard chooser uses the `.auto` file names that were successfully found
at deploy time, so a file missing after deploy is the main load failure risk.

### Named command not found

If `AutoBuilder.buildAutoChooser()` loads an `.auto` file referencing a named
command that was not registered with `NamedCommands.registerCommand()`, PathPlanner
logs a warning and substitutes a no-op `Commands.none()`. The auto continues
silently without executing that step.

This is the most dangerous silent failure: the `"Shoot"` named command being
replaced by a no-op means the robot drives the path without shooting. The only
indication is the absence of flywheel spin-up in the match log.

**Guard:** Named commands must be registered **before** `AutoBuilder.buildAutoChooser()`.
This is enforced by the code structure (registerNamedCommands() on line 234,
buildAutoChooser() on line 237).

### Shot command timing (race timeout)

The `race` group caps each shot at 2.7–3.0 seconds (varies per auto). If both
the flywheel timeout and race timeout expire without a clean shot, `stopAll` is
called and the auto moves to the next path. This degrades score but prevents
the auto from freezing.

---

## Vision Usage in Auto

Vision is **not disabled or throttled** during auto. It runs identically to
teleop. The angular velocity filter is the primary limiter:

| Robot state | Vision behavior |
|-------------|----------------|
| Fast path following (high ω) | All cameras suppressed |
| Slow/stopped (shot position) | All cameras active |
| Transitioning between segments | Intermittent suppression |

The shot distance calculation (`RobotState.getDistanceToAllianceHub()`) reads
the vision-corrected pose from `poseEstimator.getEstimatedPosition()`. If the
robot spent several consecutive cycles on a fast segment without vision, the
pose at the shot point is purely wheel-odometry. Wheel odometry at FRC distances
and speeds accumulates ~5–10 cm of error per second of un-corrected running.

The 4 cameras provide good coverage near the hub because hub-adjacent AprilTags
are visible from the alliance zone shooting positions. Vision corrections
typically converge well during the static shot window.
