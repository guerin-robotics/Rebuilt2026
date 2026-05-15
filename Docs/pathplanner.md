# PathPlanner Integration

PathPlanner 2026 handles all path following and autonomous routine execution.
Source: Drive.java, RobotContainer.java, and files under
`src/main/deploy/pathplanner/`.

---

## AutoBuilder Configuration (Drive.java:124–142)

PathPlanner's `AutoBuilder` is configured once in the `Drive` constructor.
It must run before any `PathPlannerAuto` or `AutoBuilder.buildAutoChooser()` call.

```java
AutoBuilder.configure(
    this::getPose,
    this::setPose,
    this::getChassisSpeeds,
    this::runVelocity,
    new PPHolonomicDriveController(
        new PIDConstants(5.0, 0.0, 0.0),   // Translation: Kp=5.0, Ki=0, Kd=0
        new PIDConstants(5.0, 0.0, 0.0)    // Rotation:    Kp=5.0, Ki=0, Kd=0
    ),
    PP_CONFIG,
    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
    this   // subsystem requirement for the path following command
);
Pathfinding.setPathfinder(new LocalADStarAK());
```

**Alliance flip lambda:** Uses `DriverStation.getAlliance()` with `Blue` as the
default if no alliance is known. Every path is authored in blue alliance
coordinates. When the lambda returns `true`, PathPlanner mirrors all waypoints,
rotation targets, and event marker positions across the field centerline
(`x = fieldLength / 2`).

---

## RobotConfig (PP_CONFIG) (Drive.java:68–83)

`RobotConfig` is the physics model PathPlanner uses to compute motor torque
commands and enforce actuator limits.

| Parameter | Value | Source |
|-----------|-------|--------|
| Robot mass | 63.503 kg | `Drive.ROBOT_MASS_KG` |
| Moment of inertia | 5.162 kg·m² | `Drive.ROBOT_MOI` |
| Wheel coefficient of friction | 1.2 | `Drive.WHEEL_COF` |
| Wheel radius | `TunerConstants.FrontLeft.WheelRadius` | TunerConstants |
| Max speed at 12V | `TunerConstants.kSpeedAt12Volts` | TunerConstants |
| Motor model | `DCMotor.getKrakenX60Foc(1)` with gear reduction | TunerConstants ratio |
| Slip current | `TunerConstants.FrontLeft.SlipCurrent` | TunerConstants |
| Module locations | ±0.273 m in X and Y | `DriveConstants` |
| Drive base radius | 0.386 m | computed from module locations |

These values directly affect how aggressively PathPlanner commands each module.
If `ROBOT_MASS_KG` or `ROBOT_MOI` are wrong, PathPlanner will under- or
over-correct for robot inertia and overshoot waypoints.

---

## PPHolonomicDriveController

`PPHolonomicDriveController` computes `ChassisSpeeds` corrections from pose
error every cycle:

```
translation_correction = translationPID.calculate(currentPose, targetPose)
rotation_correction    = rotationPID.calculate(currentHeading, targetHeading)
```

Both PIDs are pure proportional: **Kp=5.0, Ki=0, Kd=0**. No integral windup,
no derivative kick. At 5.0 Kp, a 0.1 m pose error produces 0.5 m/s correction,
which at typical auto speeds (3–4 m/s) is adequate for path tracking.

The controller output is added on top of the feedforward from the trajectory
velocities. The trajectory itself specifies the bulk of the speed — PID only
corrects for accumulated error.

---

## LocalADStarAK (Drive.java:134)

```java
Pathfinding.setPathfinder(new LocalADStarAK());
```

`LocalADStarAK` wraps PathPlanner's built-in `LocalADStar` pathfinder with
AdvantageKit logging (`LoggableInputs`). This means the obstacle grid state
and computed path points are serialized to the log every cycle via
`Logger.processInputs("LocalADStar", inputs)` inside the wrapper.

**Effect in replay:** The same pathfinding results from the original match are
replayed exactly — the `fromLog()` method restores path points from the log
rather than re-running the A* algorithm. This makes replay deterministic even
if pathfinding is non-deterministic.

**Active pathfinding:** `LocalADStar` dynamically replans around transient
obstacles (other robots) during auto. It updates the path if the robot deviates
significantly from the planned route. This is used for on-the-fly replanning
when the robot's actual path deviates from the pre-planned one.

---

## Named Commands (RobotContainer.java:285–334)

Registered before `buildAutoChooser()`. Every `.auto` file that references a
name in this table will execute the corresponding command.

| Name | Requires | Command tree |
|------|---------|-------------|
| `DeployIntake` | IntakePivot | `IntakePivotCommands.setPivotPosition(pivotDownPos)` |
| `RetractIntake` | IntakePivot | `IntakePivotCommands.setPivotPosition(pivotUpPos)` |
| `RunIntake` | intakeRoller, Transport | `parallel(setRollerVoltage(intakeVoltage), setTransportVoltage(transportVoltage))` |
| `Shoot` | Drive + all shooter subsystems | `parallel(joystickDriveAtAngle→hub, autoShootToHub(...))` |
| `stopAll` | Flywheel, Hood, Prestage, UpperFeeder, LowerFeeder, Transport, intakeRoller | `ShootSequences.stopAll(...)` |
| `HoodDownNamed` | Hood | `HoodCommands.setHoodPos(hoodDownPos)` |

**`Shoot` requires Drive.** This means whenever a `"Shoot"` step runs in an
auto, Drive is required by the named command. PathPlanner's path-following
command also requires Drive. Because they cannot run concurrently under the
scheduler's single-requirement rule, the `"Shoot"` command must be in its own
sequential step, not interleaved with a path. All 19 autos follow this pattern:
`path → race(Shoot, wait) → stopAll → path → ...`

---

## Event Triggers (RobotContainer.java:349–384)

Event triggers fire based on robot position along the path (not by time).
PathPlanner checks whether the robot's current `waypointRelativePos` has passed
the marker's position.

| Trigger | Position type | Action | Has requirement? |
|---------|--------------|--------|-----------------|
| `DeployIntake` | Point | `intakePivot.setPivotPosition(pivotDownPos)` | No |
| `RetractIntake` | Point | `intakePivot.setPivotPosition(pivotUpPos)` | No |
| `RunIntake` | Zone (start–end) | Start: `setRollerVoltage(12V)` / End: stop | No |
| `HoodDown` | Point | `HoodCommands.setHoodPos(hood, 0°)` with requirement | **Yes** |

**Why event triggers have no subsystem requirements:**

`HoodDown` is the exception — it does declare Hood as a requirement. The other
three intentionally omit requirements using `Commands.runOnce(() -> method())`
without passing the subsystem. The rationale (documented in RobotContainer.java:337–348):

> If an event trigger's command shares a requirement with a command in the auto
> group that is currently running, the scheduler will interrupt the auto group
> to resolve the conflict. Using `runOnce` without the subsystem avoids this.

**Risk:** When `RunIntake` zone trigger fires while `autoShootToHub` is running
(which requires `intakeRoller`), the trigger's `runOnce` directly calls
`intakeRoller.setRollerVoltage()` while `autoShootToHub`'s
`intakeRollerCommands.setRollerVoltage(agitateVoltage)` is also running. The
last `setVoltage()` call in the cycle wins. This is benign if event markers are
timed carefully to not overlap with shot windows, but could produce unexpected
roller behavior if they do overlap.

---

## Path File Format

All paths use PathPlanner 2025.0 JSON format. Stored in
`src/main/deploy/pathplanner/paths/*.path`.

### Waypoints

```json
"waypoints": [
  {
    "anchor": { "x": 3.512, "y": 4.015 },
    "prevControl": null,
    "nextControl": { "x": 3.2, "y": 4.0 }
  }
]
```

Waypoints define the Bézier control points. PathPlanner samples the spline
at small intervals to build a dense trajectory with velocity and acceleration
profiles.

### Global Constraints

```json
"globalConstraints": {
  "maxVelocity": 4.0,          // m/s
  "maxAcceleration": 5.0,      // m/s²
  "maxAngularVelocity": 540.0, // deg/s
  "maxAngularAcceleration": 1020.0  // deg/s²
}
```

These match `settings.json` defaults. PathPlanner enforces these as hard limits
when computing the trajectory's velocity profile.

### Constraint Zones

Local overrides for specific path segments, identified by `waypointRelativePos`
range:

```json
"constraintZones": [
  {
    "minWaypointRelativePos": 2.63,
    "maxWaypointRelativePos": 4.24,
    "constraints": {
      "maxVelocity": 2.7,
      "maxAcceleration": 3.5,
      "maxAngularVelocity": 540.0,
      "maxAngularAcceleration": 1020.0
    }
  }
]
```

Constraint zones are used to slow the robot down near intake zones (to reduce
note miss probability) or near the scoring position (to reduce heading error
at shot time). The `maxVelocity: 2.7` example reduces path speed by ~33% at
the intake pickup segment.

### Rotation Targets

```json
"rotationTargets": [
  { "waypointRelativePos": 2.28, "rotationDegrees": -38.16 },
  { "waypointRelativePos": 4.23, "rotationDegrees": -129.15 }
]
```

Specifies the robot's heading at specific points along the path. PathPlanner
interpolates heading between these targets independently of the translation
trajectory. This allows the robot to pre-rotate toward the hub before arriving
at the shot position.

### Event Markers

```json
"eventMarkers": [
  { "name": "DeployIntake", "waypointRelativePos": 0.0 },
  { "name": "RunIntake",
    "waypointRelativePos": 2.12,
    "endWaypointRelativePos": 4.41 }
]
```

Point markers (`waypointRelativePos` only) fire once when passed. Zone markers
(`waypointRelativePos` + `endWaypointRelativePos`) fire on enter and stop on
exit.

### Goal End State

```json
"goalEndState": { "velocity": 0, "rotation": 40.77 }
```

The trajectory decelerates to `velocity: 0` at the endpoint and aims the robot
at `rotation: 40.77` degrees. When chaining paths, the next path's
`idealStartingState` should match to avoid a discontinuous velocity profile.

---

## Auto File Format (`.auto`)

Stored in `src/main/deploy/pathplanner/autos/*.auto`.

### Command tree node types

| Type | Behavior |
|------|---------|
| `sequential` | Runs children in order |
| `parallel` | Runs all children simultaneously, ends when all finish |
| `race` | Runs all children simultaneously, ends when first finishes |
| `path` | Follows a named `.path` file |
| `named` | Executes a registered named command |
| `wait` | Waits for a fixed duration |

### Top-level fields

```json
{
  "version": "2025.0",
  "resetOdom": true,       // Call Drive.setPose() with path's starting pose
  "choreoAuto": false,     // Not a Choreo trajectory
  "command": { ... }       // Root command tree node
}
```

`"resetOdom": true` is set on all 19 autos. PathPlanner calls the `resetPose`
lambda from `AutoBuilder.configure()` at the start of the first path in the
auto, using the path's `idealStartingState` pose.

### Representative auto structure: 2.5-Right-Comp.auto

```
sequential:
  ├── path: "Right-Comp-Path-1"
  │     (event markers embedded: DeployIntake at 0.0, RunIntake zone)
  ├── race:
  │     ├── named: "Shoot"
  │     └── wait: 2.7s
  ├── named: "stopAll"
  ├── named: "DeployIntake"
  ├── path: "Right-Comp-Path-2"
  ├── race:
  │     ├── named: "Shoot"
  │     └── wait: 3.0s
  ├── named: "stopAll"
  ├── named: "DeployIntake"
  └── path: "Right-Comp-Path-3"
```

Each `race(Shoot, wait)` provides a hard timeout so a failed shot doesn't
freeze the auto. `stopAll` after each shot clears flywheel coast before the
next path. `DeployIntake` after stopAll pre-deploys for the next note pickup.

---

## Global PathPlanner Settings (settings.json)

```json
{
  "robotWidth": 0.9,            // m (bumper-to-bumper)
  "robotLength": 0.9,           // m (bumper-to-bumper)
  "holonomicMode": true,
  "defaultMaxVel": 4.0,         // m/s
  "defaultMaxAccel": 5.0,       // m/s²
  "defaultMaxAngVel": 540.0,    // deg/s
  "defaultMaxAngAccel": 1020.0, // deg/s²
  "robotMass": 63.503,
  "robotMOI": 5.162081,
  "driveWheelRadius": 0.051,    // m
  "driveGearing": 7.031,
  "maxDriveSpeed": 4.39,        // m/s
  "wheelCOF": 1.2,
  "driveMotorType": "krakenX60FOC",
  "driveCurrentLimit": 60.0     // A
}
```

`settings.json` is read by the PathPlanner GUI, not by robot code. Robot code
uses `PP_CONFIG` (constructed in `Drive.java`) which must stay in sync with
`settings.json`. If a team member updates `settings.json` in the GUI but
doesn't update `PP_CONFIG` in code, the trajectory computed by the GUI won't
match what the robot actually executes.

**Values to keep synchronized:**

| settings.json | Drive.java `PP_CONFIG` |
|--------------|----------------------|
| `robotMass` | `ROBOT_MASS_KG` |
| `robotMOI` | `ROBOT_MOI` |
| `wheelCOF` | `WHEEL_COF` |
| `driveWheelRadius` | `TunerConstants.FrontLeft.WheelRadius` |
| `maxDriveSpeed` | `TunerConstants.kSpeedAt12Volts` |
| `driveGearing` | `TunerConstants.FrontLeft.DriveMotorGearRatio` |

---

## Path Following Lifecycle Per Cycle

Each 20ms cycle while a `path` command is active:

```
PathPlannerPath.sample(currentTime)
  → targetState: position, velocity, heading, curvature

PPHolonomicDriveController.calculateRobotRelativeSpeeds(
    currentPose,       ← Drive.getPose()
    targetState
)
  → correctedSpeeds: feedforward + PID correction

Drive.runVelocity(correctedSpeeds)
  → kinematics → 4 module setpoints → TalonFX

Logger.recordOutput("Odometry/Trajectory", allPoses)      ← full path
Logger.recordOutput("Odometry/TrajectorySetpoint", target) ← current target
```

The trajectory setpoint and full path are logged every cycle, enabling
AdvantageScope's path visualization overlay.

---

## Auto List

All 19 routines available via the dashboard chooser:

```
src/main/deploy/pathplanner/autos/
├── 2.5-Left-Comp.auto
├── 2.5-Right-Comp.auto
├── 2.5-Center-Comp.auto
├── Safe-2-Right-Comp.auto
├── Safe-2-Left-Comp.auto
├── Champs-Left.auto
├── Champs-Right.auto
├── Right-Disruptor.auto
├── Left-Disruptor.auto
├── Left-Comp-Path-2.auto
└── ... (9 more)
```

Path files referenced by these autos:
```
src/main/deploy/pathplanner/paths/
├── Center-Path.path
├── Right-Comp-Path-1.path
├── Right-Comp-Path-2.path
├── Right-Comp-Path-3.path
├── Left-Comp-Path-1.path
└── ... (~20+ paths)
```

---

## Tuning PathPlanner Behavior

### Changing path velocity limits

Edit the `globalConstraints` in the `.path` file, or add a `constraintZone`
for a specific segment. Rebuild and redeploy — no code change required.

### Changing PID gains

Edit `Drive.java` lines 130–131. The PID gains apply to **all** paths globally.
Rebuild and deploy. For most swerve robots at FRC speeds, kP=5.0 is appropriate;
increase if the robot consistently undershoots waypoints, decrease if it
oscillates around them.

### Adding a new auto

1. Design the path in PathPlanner GUI (auto-synced with `settings.json` robot model)
2. Save — the `.auto` and `.path` files write to `src/main/deploy/pathplanner/`
3. Reference any named command strings exactly as registered in `RobotContainer.registerNamedCommands()`
4. Deploy — the new auto appears in the dashboard chooser automatically

### Timing a shot correctly

The race timeout (2.7–3.0 s) in each auto should be:
- Long enough for flywheel spinup (≤0.5 s) + note travel time through feeder (~0.5 s) + note clearance
- Short enough to not waste auto time if the shot fails

Current values of 2.7–3.0 s allow about 1.7–2.0 s of actual feed time after
the 0.5 s spinup timeout.
