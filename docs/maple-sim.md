# MapleSim Guide

How to run and calibrate the physics simulation. Written for someone who has never used MapleSim.

---

## What MapleSim Is

MapleSim (`maple-sim`, by the Shenzhen Robotics Alliance) is a rigid-body physics engine for FRC
simulation. It replaces the part of the stock AdvantageKit sim that models the drivetrain.

**The stock sim** (`ModuleIOSim`) spins four independent `DCMotorSim`s and asks kinematics where the
chassis must have gone. That model has no mass, no traction limit, and no walls — the robot
accelerates at physically impossible rates and drives straight through field elements.

**MapleSim** simulates the chassis as a rigid body pushed by per-module friction forces. Wheel slip,
collisions, and realistic acceleration all emerge from the physics instead of being assumed away. It
also simulates the 2026 Rebuilt field, so Fuel can be picked up, shot, and scored.

**What it does not change:** every mechanism sim (flywheel, hood, feeders, intake roller/pivot) still
runs on CTRE `TalonFXSimState` + `DCMotorSim` exactly as before. MapleSim only supplies the
drivetrain plant and the field.

---

## Setup

Nothing to install. `vendordeps/maple-sim.json` is committed, so Gradle pulls the library on the next
build.

| | |
|---|---|
| Version | `0.4.0-beta` |
| Season | 2026 (`frcYear: 2026`) |
| Requires | WPILib 2026.2.1 — matches this project |

`0.4.0-beta` is the only release built for the 2026 season. It is marked prerelease. This is
sim-only code with no path to the roboRIO, so a bug here cannot reach the robot.

To upgrade later, replace `vendordeps/maple-sim.json` with the newer release's vendordep from
[the releases page](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/releases) and rebuild.

---

## Running It

```bash
./gradlew simulateJava
```

That is the whole procedure. `Constants.currentMode` resolves to `SIM` automatically off the robot,
and `Constants.useMapleSim` is `true` by default.

A joystick or Xbox controller must be plugged in before launching for driving to work. The sim
prints a `Joystick ... not available` warning otherwise, which is harmless if you only want to watch
the field.

### Turning it off

```java
// Constants.java
public static final boolean useMapleSim = false;
```

Falls back to the original `ModuleIOSim` path. Use this if the beta misbehaves mid-session — you get
the sim that worked before without reverting anything. Has no effect on a real robot, where
`currentMode` is always `REAL`.

---

## AdvantageScope Setup

Connect to the sim over NT4 (`localhost`), then add these keys.

| Key | Type | What it shows |
|---|---|---|
| `MapleSim/GroundTruthPose` | `Pose2d` | Where the robot **actually is** in the physics world |
| `MapleSim/Fuel` | `Pose3d[]` | Every Fuel the arena draws, including shots in flight |
| `MapleSim/Fuel/Drawn` | `int` | Fuel visible on the field |
| `MapleSim/Fuel/HeldByRobot` | `int` | Fuel inside the robot (up to 50) |
| `MapleSim/Fuel/InFlight` | `int` | Shots currently airborne |
| `MapleSim/Fuel/InCirculation` | `int` | Field + robot + in flight |
| `MapleSim/Fuel/AwaitingReturn` | `int` | Scored Fuel the human player has not fed back yet |
| `MapleSim/IntakeRunning` | `boolean` | Whether the intake is currently able to grab Fuel |
| `Odometry/Robot` | `Pose2d` | Where the pose **estimator thinks** it is |

Shot diagnostics, published on every shot:

| Key | What it tells you |
|---|---|
| `MapleSim/Shot/ArrivalHeightMeters` | Where the Fuel will be at the Hub. Compare against **1.575** |
| `MapleSim/Shot/ArrivalErrorMeters` | Signed miss. Negative = low |
| `MapleSim/Shot/ArrivalVerticalSpeedMPS` | Negative = dropping in. Positive = still rising into the goal front |
| `MapleSim/Shot/FlywheelRPM` | Compare against `Flywheel/targetRPM` — below target makes every shot short |
| `MapleSim/Shot/SpunUp` / `Feeding` | Why shots are or are not firing |

**Diagnosing a miss:** if `ArrivalHeight` is right but Fuel still misses, the launch model is fine
and the problem is aim or pose. If `ArrivalHeight` is low, check `FlywheelRPM` first.

### The one view worth building

Put `MapleSim/GroundTruthPose` and `Odometry/Robot` on the 2D field **at the same time**.

The gap between them is your odometry error. On the real robot this quantity is invisible — you
never know the true pose — so this is the single most valuable thing simulation gives you. If they
drift apart and vision snaps them back together, that is the pose estimator working. If they drift
apart and stay apart, something is wrong.

For the 3D view, drag `MapleSim/Fuel` in and set the object type to a game piece to watch shots fly
and land.

---

## How the Robot Interacts With Fuel

Both behaviors are gated on real mechanism state, so the sim will not do anything the robot could
not.

**Pickup** requires the intake pivot deployed **and** the rollers spinning. Driving over Fuel with
the intake stowed does nothing.

**Shooting** requires the upper feeder feeding **and** the flywheel above a minimum speed **and** at
least one Fuel held. Shots are rate-limited, because the feeder runs continuously while the real
mechanism releases discrete balls.

### The launch model

Only the drum radius comes from `FlywheelConstants.TrajectoryVisualization`. Exit point, launch
height, angle, and speed are all defined in `SimulationConstants` — see below for why.

#### Where the Fuel spawns

MapleSim computes the spawn point as:

```
initialPosition = robotPosition + offset.rotateBy(shooterFacing)
```

It rotates by the **shooter facing**, not the robot heading. The offset is therefore expressed in the
*shooter's* frame, where positive means "out the muzzle" — **not** in the robot frame. Passing the
robot-frame value (negative X, since the shooter sits at −X) spawns Fuel out the *front* of the
robot, roughly 0.43 m from where it belongs, and every shot then flies too far and arrives low.

`SHOOTER_EXIT_DISTANCE` is positive for this reason. If you ever see Fuel appearing from the wrong
face of the robot, this sign convention is the first thing to check.

**Hood position is not launch angle.** `Hood.getPosition()` reports a *mechanism* angle: the CANcoder
reading after the 122:12 reduction. Hub shots ask for hood values of roughly 1°–12°, and using those
directly as the ball's elevation gives a nearly flat shot that cannot reach the Hub from anywhere on
the field.

`SimulationConstants` therefore converts hood position to real elevation:

```
launchAngleDeg = 65.60 + (-0.420 × hoodDegrees)     → 60°-65°
launchSpeed    = ω × drumRadius × 1.06
```

#### Why the arc is lofted, not flat

For a given speed and distance, **two** launch elevations reach the Hub: a flat one and a lofted one.
An unconstrained fit finds the flat one first — and it is a trap. At these speeds the flat solution
is degenerate: its apex sits level with the Hub and the Fuel arrives nearly horizontal, so it skims
into the *side* of the goal instead of dropping into it. It scores (the goal volume is generous) but
looks wrong, and has no margin.

The lofted solution is used instead. Apex lands 10–49 in above the Hub, arriving 42°–52° below
horizontal.

Accuracy is therefore judged by **horizontal distance from the Hub center as the Fuel descends
through Hub height** — what actually scores — not by height at a fixed distance. Worst case is 8.8 in
against `RebuiltHub.GoalRadius` of 23.5 in.

#### Fuel vanishing into the Hub is scoring

`RebuiltHub extends Goal`. Fuel entering the goal volume is consumed and points are added, so it does
not return to the field — it is *in the Hub*. Fuel "disappearing into the side of the hub" means
shots are working.

---

## Fuel Count and the Human Player

The arena places **192 Fuel** at reset. Two things then remove it from view:

- The robot holds up to `INTAKE_CAPACITY` (50) — a quarter of the field, invisible inside it.
- Fuel scored in the Hub is **consumed permanently**. MapleSim never returns it.

That second one is the important one. The library expects the robot program to decide when a human
player would feed Fuel back in — `Arena2026Rebuilt.outpostDump()` and `outpostThrow()` exist for
exactly this and are never called on your behalf. Left alone, the field drains as you shoot until
there is nothing to intake.

`MapleSimWorld` now models the human player: it keeps a target number of Fuel **in circulation** and
feeds replacements in at the depot when the Hub has swallowed some.

| Constant | Default | What it does |
|---|---|---|
| `FUEL_RETURN_ENABLED` | `true` | Set false to let the field drain, like a practice field with nobody feeding it |
| `TARGET_FUEL_IN_CIRCULATION` | 192 | **Raise this to put more Fuel in play than the arena starts with** |
| `FUEL_RETURN_INTERVAL_SECONDS` | 0.4 | How fast the human player feeds Fuel back |
| `FUEL_RETURN_POSITION` | (0.7, 0.7) | Blue depot; mirrored for red |

"In circulation" counts Fuel on the field **plus** Fuel in the robot **plus** Fuel in flight. Counting
only what is lying on the field would treat a full 50-Fuel hopper as 50 missing Fuel and flood the
field with replacements.

Watch `MapleSim/Fuel/InCirculation` and `MapleSim/Fuel/AwaitingReturn` to see this working.

**Where those numbers came from:** they were fitted from the robot's own characterization. Every
(distance, RPM, hood) triple in `SPEED_MAP` and `ANGLE_MAP` was tuned until shots scored, so each one
is a known-good shot. Projectile motion is solved for the elevation that carries Fuel from the
shooter into the Hub at each of the nine points, taking the lofted root as described above.

#### MapleSim's gravity is not Earth's

`GamePieceProjectile.GRAVITY` is **11.0 m/s²**, not 9.81 — the library inflates gravity as a rough
stand-in for the air drag it does not model. Its trajectory is `z = h₀ + v_z·t − 5.5t²`.

Any hand-derived shot math must use 11.0. Fitting against 9.81 lands Fuel 5 in low at close range and
15 in low at the far shots — enough to hit the goal structure instead of dropping in, and it looks
exactly like a launch-angle problem. `LaunchModelTest` reads the constant from the library rather
than hardcoding it, so a maple-sim update that changes gravity fails the build instead of quietly
invalidating the fit.

The fit also requires every shot to be **descending** at the Hub. A shot still rising when it arrives
hits the front of the goal structure instead of dropping in, which is what "hitting the top of the
section the plastic attaches to" looks like.

The negative slope is physically sensible — farther shots use more speed and a flatter arc.

**Launch geometry uses the CAD numbers** from `RobotModelVisualizer.SHOOTER_AXIS_POSITION`
(0.2762 m back, 0.415 m up). These disagree with `TrajectoryVisualization`'s hand-entered −6 in and
20 in. The CAD pair is self-consistent and came from the exported model, so it wins here — but the
two sources should be reconciled against the real robot.

**These are derived, not measured.** The fit assumes no aerodynamic drag and a shot through Hub
center. If you measure the real launch angle, replace them; `LaunchModelTest` will tell you
immediately if the new values stop putting characterized shots in the Hub.

> **Note on `VELOCITY_FUDGE_FACTOR`:** the visualizer's value of 0.8 makes Fuel physically unable to
> reach the Hub from any mapped distance — it falls short even on an ideal 45° arc. The sim uses 1.01
> instead, the lowest value consistent with the characterization tables. **The AdvantageScope
> trajectory preview still uses 0.8 and the raw hood angle, so that preview is wrong on the real
> robot too** — it is drawn by `FlywheelVisualizer`, which this work did not change.

---

## Calibration

All values live in `frc/robot/simulation/SimulationConstants.java` and are **simulation-only** —
changing anything here cannot affect match behavior.

### Values measured on the robot

| Constant | Value |
|---|---|
| `BUMPER_LENGTH_X` / `BUMPER_WIDTH_Y` | 33.4 × 33.4 in |
| `INTAKE_WIDTH` | 25.5 in |
| `INTAKE_EXTENSION` | 8.5 in past the front bumper |

### Values mirrored from elsewhere — keep in sync

These are duplicated because the originals are `private`. If you change one, change both.

| Constant | Mirrors |
|---|---|
| `ROBOT_MASS`, `WHEEL_COF` | `Drive.ROBOT_MASS_KG`, `Drive.WHEEL_COF` |
| Gear ratios, wheel radius, friction voltage, `DRIVE_CURRENT_LIMIT` | `COMP_TunerConstants` |

### Values that are guesses — fix when you know better

| Constant | Current | Basis |
|---|---|---|
| `INTAKE_CAPACITY` | 50 Fuel | Measured on the real hopper |
| `SHOT_INTERVAL_SECONDS` | 0.05 s | 20 Fuel/second, measured |
| `SHOOTER_EXIT_DISTANCE` | 0.2000 m | Tuned against sim — **positive, in shooter frame** |
| `LAUNCH_HEIGHT` | 0.415 m | CAD shooter axis |
| `LAUNCH_ANGLE_OFFSET_DEGREES` | 65.60 | Fitted from characterization — replace with a measurement |
| `LAUNCH_ANGLE_PER_HOOD_DEGREE` | −0.420 | Fitted from characterization — replace with a measurement |
| `LAUNCH_VELOCITY_FACTOR` | 1.06 | Fitted from characterization — replace with a measurement |

---

## The Field Bumps — Read This Before Trusting an Auto

**MapleSim cannot simulate driving over anything.** Its physics engine (dyn4j) is strictly 2D. The
chassis has no Z coordinate, so every obstacle is a wall of infinite height. A bump the robot drives
over and a guardrail it cannot cross are the same object to the engine.

The stock `Arena2026Rebuilt` includes the two field bumps as solid obstacles — 47 × 217 in, centered
at x ≈ 4.60 m and x ≈ 11.94 m, spanning most of the field width. With those in place, a simulated
auto that crosses one **stops dead against it**, which makes most routines impossible to run.

`RebuiltArena` removes them, controlled by `SimulationConstants.BUMPS_ARE_PASSABLE` (default `true`).

Neither behavior is correct:

| Setting | Behavior | Wrong because |
|---|---|---|
| `true` (default) | Robot crosses freely | No time loss, traction loss, or deflection |
| `false` | Robot is blocked | The real robot crosses it fine |

**So:** trust a sim auto for *path geometry* across a bump. Do **not** trust it for *timing*, and do
not conclude a path is clean just because it ran here. Given the auto time budget is already tight,
assume a real crossing costs more than the sim shows.

The bumps are matched by physical size rather than list position, and `RebuiltArenaTest` fails the
build if the count is ever not exactly 2 — so a maple-sim update that changes the field map surfaces
as a test failure instead of autos mysteriously stopping dead again.

---

## Known Limitations

**Acceleration is likely optimistic.** MapleSim computes a theoretical max around 13.5 m/s² from the
80 A slip current and the configured COF. If the drivetrain is voltage-saturated rather than
traction-limited on the real field, the sim will out-accelerate the real robot substantially. This
matters most for auto timing — check a real log before trusting sim auto durations. Top speed is
close (sim ≈ 4.39 m/s vs. `kSpeedAt12Volts` 4.0 m/s).

**MapleSim warns about the COF at startup:**

```
The provided "tire coefficient of friction" is 2.225, which seems abnormal
```

Non-fatal; the sim continues. The value is kept because it mirrors `Drive.WHEEL_COF`, and matching
the real configuration is more useful than quietly substituting a different number for sim.

**Battery voltage is modeled per-subsystem, not globally.** Several mechanism sims each call
`RoboRioSim.setVInVoltage(...)` independently, so the last one to run each loop wins. Brownout
behavior in sim is therefore not trustworthy. MapleSim ships a `SimulatedBattery` that could
centralize this — not wired up, since it would mean touching every mechanism IO.

---

## Troubleshooting

**`IllegalStateException: MapleSimWorld accessed outside simulation`**
Something called `MapleSimWorld.getInstance()` without guarding on `MapleSimWorld.isActive()`. That
guard is what keeps a physics world from ever being constructed on a real robot — add the check
rather than removing the exception.

**`MapleSim is running on a real robot!`**
Thrown by MapleSim itself, same cause as above, one layer deeper.

**Robot sits still and never moves**
No joystick connected, or the sim is disabled. Enable it in the Driver Station sim window.

**Estimator pose and ground truth start far apart**
Expected right after a reset if `Drive.setPose()` was bypassed. `setPose()` teleports the physics
body as well as resetting the estimator; anything writing to the estimator directly will desync them.

**Loop overruns**
A handful at startup is normal (~11 in 3000 loops when this was added, with `Drive.periodic()` at
~0.15 ms). Sustained overruns mean the physics step is too expensive — reduce sub-ticks via
`SimulatedArena.overrideSimulationTimings(...)`.

---

## Where the Code Lives

| File | Role |
|---|---|
| `simulation/MapleSimWorld.java` | Owns the physics world, arena, and Fuel interaction |
| `simulation/SimulationConstants.java` | Every tunable value described above |
| `simulation/RebuiltArena.java` | 2026 arena with the bumps made passable |
| `subsystems/drive/ModuleIOMapleSim.java` | Module IO backed by the physics engine |
| `subsystems/drive/GyroIOSim.java` | Simulated gyro with realistic drift |
| `Constants.useMapleSim` | Backend toggle |
| `Robot.simulationPeriodic()` | Steps the world once per robot period |

`MapleSimWorld` is not a subsystem and holds no subsystem references. `RobotContainer` wires it with
suppliers, consistent with the architecture rules in `.claude/rules/01-architecture.md`.
