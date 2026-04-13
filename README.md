# Rebuilt2026

Welcome. This repository is the **Java code** that runs on our FRC robot for the **2026** FIRST Robotics Competition game **Rebuilt**. If you are new to programming, think of this project as a set of instructions the robot follows: when you press a button, a **command** runs, and that command tells different **subsystems** (intake, drive, shooter, etc.) what to do.

Official field layout and game rules are in the [2026 Game Manual (PDF)](https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf). This README only summarizes how **our** robot matches the code—use the manual for scoring zones, field dimensions, and element names.

This document explains:

- What the **physical robot** does and how that matches **folders in the code**
- How the program is **organized** so you can find things quickly
- How we **drive** at competition and how **autonomous** works
- How to **build** the project and where **numbers** (constants) are stored

If you only read one file in code to see how everything plugs together, start with `src/main/java/frc/robot/RobotContainer.java`.

---

## Robot mechanisms (fuel path)

On the real robot, **fuel** (the game piece for Rebuilt) moves along a path from the floor into the shooter. The code splits that path into **subsystems** so each part can be programmed and tested on its own.

### Step by step (what happens physically)

1. **Intake pivot** — The intake arm **rotates out** or **stows**. The code treats pivot position as a goal angle or rotation (see `subsystems/intakePivot/`).
2. **Intake rollers** — Rollers spin so **fuel** moves **toward the shooter** (`subsystems/intakeRoller/`).
3. **Transport** — Floor rollers move fuel **along the path toward the shooter** (`subsystems/transport/`).
4. **Lower feeder** and **upper feeder** — Two stages that push **fuel** **up** toward the shooter (`subsystems/lowerFeeder/`, `subsystems/upperFeeder/`).
5. **Prestage** — **Five rollers** that feed **fuel** **into the flywheel**. The Java code still uses a **single prestage subsystem** (for example a leader and follower motor following the same setpoint) rather than five independent loops—see `subsystems/prestage/`.
6. **Hood** — Moves **up and down** to change the **launch angle**. The hood has rollers that work with the main shooter (`subsystems/hood/`).
7. **Flywheel** — The main **drum** shooter wheels; speed controls how hard the shot is (`subsystems/flywheel/`).

### Important limitation: no sensors in the path

We do **not** have beam breaks or other sensors along the **fuel** path. That means the code **cannot** ask “is the **fuel** exactly here?” Instead we use:

- **Timed waits** (wait X seconds then start the next thing)
- **Motor speeds** we trust from tuning
- **Driver judgment** (when to keep holding intake or shoot)

When you read commands in `commands/ShootSequences.java`, remember: timing is part of the design because we are not sensing **fuel** at each stage.

---

## Big-picture software ideas

These terms show up everywhere in WPILib projects.

### Command-based programming

Instead of one giant loop, we write **small commands** (“set flywheel speed”, “deploy intake”) and **schedule** them. The **command scheduler** runs every robot cycle, updates what each command needs, and stops commands when they finish or when another command interrupts them.

### Subsystems

A **subsystem** owns a piece of hardware (motors, encoders, etc.) and exposes methods like “run at this voltage” or “go to this angle.” Commands **require** subsystems so two commands do not fight over the same motors by accident.

### IO pattern (real robot vs simulation)

Many subsystems use an **IO interface** (for example `FlywheelIO`, `HoodIO`). The real robot uses `*IOReal` or Phoenix implementations; the computer simulator uses `*IOSim`. That lets us test logic without always having the physical robot connected.

---

## Software architecture (how everything fits together)

### Entry point: `Robot.java` and `RobotContainer.java`

- **`Robot.java`** — Provided by WPILib / AdvantageKit template. It starts **logging**, creates **`RobotContainer`**, and runs the scheduler every loop.
- **`RobotContainer.java`** — Where **most wiring lives**: subsystems are created here, **buttons** are bound to commands, and the **auto chooser** is set up. If you wonder “what runs when I press this?”, look here.

### `RobotState` — one place for “where am I on the field?”

**File:** `src/main/java/frc/robot/RobotState.java`

The drive subsystem feeds wheel and gyro data into a **pose estimator**. `RobotState` is a **singleton** (one shared object) that answers questions like:

- What is our **position and heading** on the field?
- How **far** are we from the **alliance hub** and other field targets we care about for shooting?

Other subsystems can use this for **distance-based shooting** without directly talking to the drive code. That keeps the design cleaner.

### `ShotCalculator` and `HoodPosCalculator` — turning distance into shooter settings

**Files:**

- `subsystems/flywheel/ShotCalculator.java`
- `subsystems/hood/HoodPosCalculator.java`

When we want to shoot toward a goal from wherever we are:

1. We get **distance** from `RobotState` (often horizontal distance to a target).
2. **`ShotCalculator`** looks up **flywheel speed** (RPM) from a **table** stored in `FlywheelConstants` (interpolation between measured points).
3. **`HoodPosCalculator`** looks up **hood angle** from a similar table in `HoodConstants`.

So “aiming” is not one magic formula in one line—it is **data we collected while tuning** the robot, stored as maps, and read at runtime.

### `ShootSequences` — recipes that combine subsystems

**File:** `commands/ShootSequences.java`

Shooting is not one motor. This class builds **parallel** and **sequential** commands, for example:

- Spin up **flywheel** and **prestage**, move **hood**, wait, then run **feeders** and **transport** together.

That is how we encode “do A and B at the same time, then after a delay do C.” Read this file to see the exact order of operations for each **shoot** command, **pass**, tuning, and **stop everything** behaviors.

### `Triggers` — buttons and “is it safe to shoot?”

**File:** `src/main/java/frc/robot/Triggers.java`

This class centralizes **joystick / gamepad buttons** and also **logical conditions** from the field (for example, whether we are in a zone where shooting toward our alliance goal is appropriate). Other code can ask `Triggers` instead of duplicating math everywhere.

### Drive (swerve)

**Folder:** `subsystems/drive/`

We use **swerve drive** (each wheel steers and drives). Module positions, gear ratios, and CAN bus settings for the drivetrain come from **generated** Java files under `src/main/java/frc/robot/generated/`:

- `TunerConstants.java` picks between **`COMP_TunerConstants`** and **`ALPHA_TunerConstants`** based on `Constants.robotType` in `Constants.java`.

Those files are produced from the **CTRE Swerve Tuner** tool. If someone changes wheel locations or CAN IDs on the drivetrain, you **regenerate** there—not by guessing numbers in random files.

### Shooter stack CAN IDs (not in Tuner)

Intake, transport, feeders, prestage, hood, and flywheel motor **IDs** live in **`HardwareConstants.java`** inside the nested class `CanIds`. That is the quick reference when you are wiring or debugging Phoenix Tuner on the bench.

---

## Where constants live (numbers you might need to change)

Constants are **named values** so we do not scatter magic numbers through the code. They are split by purpose so you know **where to look**.

### `Constants.java` — simulation mode and which drivetrain file to use

**File:** `src/main/java/frc/robot/Constants.java`

- **`currentMode`** — `REAL` on the roboRIO; on a laptop it follows **`simMode`** (often `SIM` or `REPLAY` for AdvantageKit).
- **`robotType`** — Selects **COMP** vs **ALPHA** swerve constants. Change this when you switch between physical robot variants that use different generated tuner files.

### `HardwareConstants.java` — IDs, driver ports, and “competition” presets

**File:** `src/main/java/frc/robot/HardwareConstants.java`

Contains a lot of day-to-day tuning:

- **`CanIds`** — CAN bus device IDs for shooter stack and intake (see table above mentally).
- **`CompConstants`** — Voltages and velocities we use in **match-like** commands (transport voltage, prestage speed, feeder speeds, idle speeds, spin-up **wait** times).
- **Tower / pass / tuning** presets — Named speeds and hood angles for specific shot types.
- **`ControllerConstants`** — Which USB port expects the Xbox, joystick, button panel, etc.

If a mentor says “the transport feels weak,” you might adjust a voltage here—but **only** after understanding what command is running.

### Per-subsystem `*Constants.java` — gains, limits, and lookup tables

**Examples:** `FlywheelConstants.java`, `HoodConstants.java`, `PrestageConstants.java`, etc.

Here you find:

- **PID / feedforward** style gains for closed-loop control
- **Current limits** and safety caps
- **Distance maps** (flywheel RPM vs meters, hood angle vs meters)

### `FieldConstants` — the official field in code form

**File:** `src/main/java/frc/lib/FieldConstants.java`

Locations of **AprilTags**, **alliance hub**, zones, and other **Rebuilt** field elements (see the [2026 Game Manual](https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf)). Used for pose, distance, and alliance flipping (red vs blue). Coordinates follow the **blue alliance origin** convention documented in that file.

### Policy: when do we edit lookup tables?

The **flywheel speed vs distance** and **hood angle vs distance** tables are **not** edited during normal practice. We change them during **dedicated tuning sessions** when we are intentionally re-characterizing the shooter after mechanical or software changes.

---

## Operator controls (competition)

How humans interface with the code at events:

| Device | Role |
|--------|------|
| **Thrustmaster** (joystick) | **Primary** controller used in **real matches**. |
| **Xbox controller** | **Overrides** and backup—useful when something is remapped or for secondary actions. |
| **Button panel** | **Testing and development** only. Do not assume elims drivers will use it. |

**Field-oriented control (FOC):** During competition teleop, driving is usually **field-oriented**: pushing “forward” on the stick moves the robot **toward the opposite alliance wall** regardless of which way the robot chassis faces. That matches how drivers think on the field. (This is separate from motor-level “FOC” in Phoenix documentation, which refers to torque/current control on individual motors.)

Button numbers and triggers are wired in `Triggers.java` and bound in `RobotContainer.java`.

---

## Autonomous (autos)

### What we use today: PathPlanner

**PathPlanner** is the **working** system for competition:

- Path files live under `src/main/deploy/pathplanner/` (typical WPILib layout).
- **`RobotContainer`** registers **named commands** in `registerNamedCommands()` so paths can call human-readable steps like `DeployIntake`, `Shoot`, `stopAll`.
- The dashboard **auto chooser** lists autos built from PathPlanner.

If Choreo is not ready, **this is what we run in a match.**

### Choreo (future)

We are **actively trying** to integrate **Choreo** for trajectory generation. It is **not** fully working end-to-end yet. Until it is tested on the real robot, treat PathPlanner as the **source of truth** for autonomous.

### Pitfall: EventTriggers and command requirements

`RobotContainer` contains **comments** explaining a subtle WPILib rule: if an **event marker** schedules a command that **requires** the same subsystem as the main auto group, the scheduler may **cancel the whole auto**. Some markers use `runOnce` without subsystem requirements on purpose. Read those comments before changing auto markers.

---

## Logging and dashboard

### AdvantageKit

We use **AdvantageKit** for **logging** (recording sensor data, outputs, and poses to a file). On the real robot, logs are written so we can **replay** matches on a laptop and debug without the robot.

Relevant code: `Robot.java` (how logging is configured).

### Elastic

**Elastic** is our **dashboard** for pit and drive team: graphs, key numbers, and status. It connects over NetworkTables like other WPILib dashboards.

---

## Build, test, and deploy

### What you need installed

- **JDK 17**
- **WPILib** (includes Gradle and the roboRIO deployment tools for FRC)

This project uses **GradleRIO 2026**; see `build.gradle` for versions.

### Common commands

From the project root:

```bash
./gradlew build
```

That **compiles** the code, runs **Spotless** (code formatting), and runs **tests**. You should get in the habit of running `build` before opening a pull request.

To deploy to the robot (when the roboRIO is on the network):

- Use the WPILib VS Code command **Deploy Robot Code**, or the Gradle deploy task your mentors prefer.

**Team number** is read from `.wpilib/wpilib_preferences.json` (or can be passed via Gradle)—do not hardcode it in Java.

### Simulation

Desktop **simulation** is enabled in Gradle. Note: the sim **GUI** may be **off by default** so that **log replay** works cleanly. If simulation looks wrong, check `build.gradle` and AdvantageKit docs.

---

## Vendor libraries (`vendordeps/`)

Third-party libraries ship as JSON files in **`vendordeps/`**. Examples include:

- **Phoenix 6** — CTRE TalonFX, CANcoder, Pigeon, etc.
- **PhotonVision** — camera pipelines
- **PathPlanner** — autonomous paths
- **AdvantageKit** — logging
- Others as listed in that folder

Gradle downloads the right JARs when you build. If you add a new vendor library, the JSON file must be committed.

**Reminder:** Swerve **`TunerConstants`** files are **generated** from CTRE’s tool. Shooter tuning lives in subsystem constants and `HardwareConstants`, not in those generated drive files.

---

## Contributing (for students and mentors)

1. **Find the right layer** — Is it a button binding? `RobotContainer`. Is it shot sequencing? `ShootSequences`. Is it “how fast is the flywheel”? `FlywheelConstants` / commands.
2. **Keep subsystems clean** — Put hardware details in subsystem / IO classes, not in random commands.
3. **Run `./gradlew build`** before you finish—Spotless will reformat Java if needed.
4. **Ask** if you are unsure whether a value is **robot-wide** (`HardwareConstants`) or **one subsystem** (`*Constants` in that folder).

---

## Quick file map

| You want to understand… | Start here |
|-------------------------|------------|
| Program startup and logging | `Robot.java` |
| Buttons, autos, subsystem creation | `RobotContainer.java` |
| Field pose and distance | `RobotState.java` |
| Shooting math from distance | `ShotCalculator.java`, `HoodPosCalculator.java` |
| What happens when we “shoot” or “pass” | `commands/ShootSequences.java` |
| Joystick and safety triggers | `Triggers.java` |
| CAN IDs for shooter / intake | `HardwareConstants.java` → `CanIds` |
| Swerve tuning (generated) | `generated/TunerConstants.java` |
| Field layout | `frc/lib/FieldConstants.java` |

Welcome to the codebase—read a little, run `build`, and trace one command from a button in `RobotContainer` through to a subsystem method. That is the fastest way to learn.
