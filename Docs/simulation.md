# Simulation

WPILib simulation runs robot code against virtual hardware models. AdvantageKit's
IO abstraction makes this transparent: in `Constants.Mode.SIM`, each subsystem
receives an `*IOSim` instance instead of `*IOReal`. The subsystem class itself
is identical in both modes.

---

## Simulation Mode Selection (RobotContainer)

```java
switch (Constants.currentMode) {
    case REAL:
        subsystem = new Subsystem(new SubsystemIOReal(...));
        break;
    case SIM:
        subsystem = new Subsystem(new SubsystemIOSim(...));
        break;
    default:     // REPLAY
        subsystem = new Subsystem(new SubsystemIO() {});  // no-op stub
        break;
}
```

In SIM mode, `NT4Publisher` is the only data receiver (no USB log writer), so
all simulated values stream to NetworkTables for dashboard display.

---

## Drive Simulation

**IO class:** `ModuleIOSim`

**WPILib model:** `DCMotorSim` for each drive and turn motor.

Each swerve module gets two `DCMotorSim` instances:
- Drive motor: Kraken X60 motor model, module gear ratio, wheel moment of inertia
- Turn motor: Kraken X60 motor model, turn gear ratio, module steer inertia

`ModuleIOSim.updateInputs()`:
- Steps both simulations by `Robot.defaultPeriodSecs` (0.02s)
- Populates drive position/velocity from sim angular position/velocity
- Populates turn position/velocity from sim state
- Applied voltage and current read from sim internal state

`ModuleIOSim.setDriveVelocity()` and `setTurnPosition()` write voltage to the
DC motor model (the closed-loop control for sim runs inside the IO class rather
than on the motor controller hardware).

**Gyro simulation:** `GyroIO {}` stub is used — the gyro always returns the
accumulated yaw computed from simulated module states. The `Drive` subsystem
integrates angular velocity from kinematics when no real gyro is available.

**High-frequency odometry in sim:** `PhoenixOdometryThread` is started in Drive's
constructor regardless of mode, but in sim it collects no Phoenix 6 signals
(there are no real devices). The odometry update in `periodic()` drains an empty
queue and falls back to a single 50 Hz update per cycle.

---

## Vision Simulation

**IO class:** `VisionIOPhotonVisionSim`

**WPILib model:** PhotonLib `VisionSystemSim` + `PhotonCameraSim`

```java
// VisionIOPhotonVisionSim (initialized once, static)
private static VisionSystemSim visionSim;

static {
    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(VisionConstants.aprilTagLayout);
}

// Per-camera constructor
this.camera = new PhotonCamera(name);
this.cameraSim = new PhotonCameraSim(camera, cameraProperties);
visionSim.addCamera(cameraSim, robotToCamera);
cameraSim.enableDrawWireframe(true);  // optional visualization
```

`updateInputs()` each cycle:
1. `visionSim.update(poseSupplier.get())` — update sim with current robot pose
2. `camera.getLatestResult()` — read sim camera output (same API as real camera)
3. Populate `VisionIOInputs` identically to the real implementation

The `poseSupplier` is passed at construction time in `RobotContainer`:

```java
new VisionIOPhotonVisionSim(cameraName, robotToCamera,
    drive::getPose)
```

This creates a feedback loop: the drive subsystem's estimated pose drives the
sim cameras, and the sim camera output feeds back into the drive's pose
estimator. In practice this produces stable behavior because the pose estimator
has its own odometry contribution that prevents vision from being the sole
source.

**Camera noise model:** `PhotonCameraSim` applies simulated detection noise based
on the `SimCameraProperties` passed at construction. Parameters include field of
view, resolution, and detection range. These properties are defined per-camera in
`VisionIOPhotonVisionSim`.

**Static visionSim:** The `VisionSystemSim` is a static field, shared across all
four `VisionIOPhotonVisionSim` instances. This means all four sim cameras update
from the same `visionSim.update()` call each cycle. The `update()` is called in
each camera's `updateInputs()` — meaning it's called 4 times per cycle (once per
camera). `VisionSystemSim.update()` is idempotent when called with the same pose
multiple times in the same cycle, but this is worth verifying if PhotonLib updates
change the sim's internal state machine.

---

## Flywheel Simulation

**IO class:** `FlywheelIOSim`

**WPILib model:** `FlywheelSim` (single-degree-of-freedom rotational inertia)

Simulates 5 motors (leader + 4 followers) as a single combined flywheel with
summed inertia and torque. `FlywheelIOSim.updateInputs()` steps the sim and
fills the `ShooterIOInputs` struct — follower velocities are set equal to the
leader sim velocity (since all followers are physically coupled).

`FlywheelIOSim` does **not** simulate temperature (all temperature fields remain
at 0.0). This is acceptable for development but means temperature-based
protection logic cannot be tested in sim.

---

## Hood Simulation

**IO class:** `HoodIOSim`

**WPILib model:** `SingleJointedArmSim` (or `DCMotorSim` for simple rotation)

Simulates the hood pivot with gravity compensation if applicable. Position
closed-loop runs inside the sim IO class. `hoodPosition` and
`hoodClosedLoopReference` are populated each cycle.

---

## Prestage Simulation

**IO class:** `PrestageIOSim`

**WPILib model:** `DCMotorSim` × 2 (left and right motors)

Both motors simulated independently. Velocity closed-loop runs inside the IO
class. All fields in `PrestageIOInputs` populated from sim state; temperatures
remain 0.0.

---

## Feeder Simulations (Upper and Lower)

**IO classes:** `UpperFeederIOSim`, `LowerFeederIOSim`

**WPILib model:** `DCMotorSim`

Single motor simulation per feeder. Velocity closed-loop inside IO class.
Structurally identical to Prestage simulation.

---

## Transport Simulation

**IO class:** `TransportIOSim`

**WPILib model:** `DCMotorSim`

Single motor. Same pattern as feeders.

---

## Intake Pivot Simulation

**IO class:** `IntakePivotIOSim`

**WPILib model:** `SingleJointedArmSim`

Simulates the arm rotation with configurable mass, length, and gravity. Position
closed-loop inside IO class. `IntakePivotVisualizer` receives the simulated
position and renders a `Mechanism2d` widget in the sim GUI.

---

## Intake Roller Simulation

**IO class:** `intakeRollerIOSim`

**WPILib model:** `DCMotorSim` × 2 (leader + follower)

Voltage-mode control. Follower mirrors leader. All `intakeRollerIOInputs` fields
populated from sim state.

---

## Simulation Coverage Matrix

| Subsystem | IOSim exists | Motor model | Position sim | Velocity sim | Temperature sim | Notes |
|-----------|-------------|-------------|-------------|-------------|-----------------|-------|
| Drive modules | Yes | DCMotorSim | Yes | Yes | No | Gyro uses stub |
| Vision | Yes | VisionSystemSim | — | — | — | 4 cameras, AprilTag field |
| Flywheel | Yes | FlywheelSim | No | Yes | No | 5 motors combined |
| Hood | Yes | DCMotorSim/ArmSim | Yes | Yes | No | |
| Prestage | Yes | DCMotorSim×2 | Yes | Yes | No | |
| UpperFeeder | Yes | DCMotorSim | Yes | Yes | No | |
| LowerFeeder | Yes | DCMotorSim | Yes | Yes | No | |
| Transport | Yes | DCMotorSim | Yes | Yes | No | |
| IntakePivot | Yes | SingleJointedArmSim | Yes | Yes | No | Visualizer works |
| intakeRoller | Yes | DCMotorSim×2 | Yes | Yes | No | |

**All 10 subsystems have simulation implementations.** Temperature is not
simulated for any subsystem — all temperature fields return 0.0 in sim.

---

## Running Simulation

```
./gradlew simulateJava
```

This launches the WPILib simulation GUI with:
- Robot code running with `Constants.currentMode = SIM`
- Simulated joystick / driver station
- AdvantageScope-compatible NT stream on `localhost:5810`
- Field2d widget showing robot pose and auto path preview

The sim controller (port 2 in this codebase) is active during simulation.
The Thrustmaster joystick bindings (port 0) can be mapped to a keyboard or
gamepad in the simulation GUI's "Joystick" panel.

---

## Identified Simulation Gaps

### Gap 1: VisionSystemSim updated 4× per cycle

`visionSim.update(poseSupplier.get())` is called once per camera's
`updateInputs()` invocation. With 4 cameras, it's called 4 times per 20ms
cycle. `VisionSystemSim.update()` should be idempotent within a cycle (same
pose input → same output), but if PhotonLib's internal simulation has any
step-based state (motion blur accumulator, frame counter), calling it 4 times
could produce incorrect results.

**Recommended fix:** Call `visionSim.update()` once per cycle in a shared
static initializer or in `Vision.periodic()` before the per-camera loop.

### Gap 2: Temperature always 0.0 in all IOSim implementations

No simulation produces non-zero temperature. If temperature-based protection
logic is ever added (e.g., reduce flywheel RPM if motor temperature exceeds
threshold), it cannot be tested in sim.

**Not a current risk** since no such logic exists, but worth noting for future
additions.

### Gap 3: Gyro simulation is a stub

`Drive` uses `GyroIO {}` (no-op) in sim. The heading is derived by integrating
simulated module angular velocities. This means the gyro `connected` field is
always false in sim, and there is no simulated gyro drift or noise. Vision
corrections still work, but gyro-only heading accuracy is perfect (no drift),
which may mask field-heading drift issues that occur on the real robot.

### Gap 4: High-frequency odometry is single-rate in sim

`PhoenixOdometryThread` does not collect signals from `DCMotorSim` (sim motors
don't publish to Phoenix 6 signal queues). Drive falls back to a single 50 Hz
odometry update per cycle in simulation, identical to a non-Phoenix robot. The
high-frequency odometry advantage (250 Hz, better pose accuracy at high speed)
is not present in sim.

**Not a correctness issue** — it just means the sim's odometry quality doesn't
exactly match real robot odometry at high speeds.

### Gap 5: No simulated game pieces / intake detection

There are no simulated game piece objects, no simulated note sensors, and no
simulated "note acquired" state. Intake and transport commands complete their
velocity/voltage setpoints but there is no sim feedback that a note was actually
ingested. Auto routines that depend on note detection (if any are added) cannot
be fully tested in sim.

### Gap 6: No simulated CAN bus faults or disconnections

`connected` fields in all IO inputs default to `true` in all IOSim
implementations. Fault injection (disconnected encoder, CAN fault, brownout)
cannot be tested in sim. In real matches, CAN disconnections are a common cause
of match failures.

### Gap 7: Intake Pivot visualizer uses logged inputs but goalPosition is not logged

`IntakePivotVisualizer` receives both `inputs.intakePivotPosition` (current)
and `goalPosition` (commanded). The visualizer displays both. However,
`goalPosition` is only an instance field in `IntakePivot` — it's not logged to
AdvantageKit. In AdvantageScope, the goal arm position is invisible.

**Fix:** Add `Logger.recordOutput("IntakePivot/GoalPosition", goalPosition)` in
`IntakePivot.periodic()`.
