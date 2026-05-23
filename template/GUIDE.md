# FRC Season Template Guide
*Guerin Robotics — extracted from Rebuilt2026*

Use this directory as the starting point for each new season's robot code.
Copy the timeless pieces. Rewrite the game-specific pieces. Do not carry debt forward.

---

## A — Timeless Patterns (Copy Every Season)

These are architectural decisions that have nothing to do with the game.
They should be preserved in every robot regardless of mechanism layout.

### Infrastructure
| File/Pattern | Why Keep It |
|---|---|
| AdvantageKit IO layer (`XxxIO` / `XxxIOInputsAutoLogged`) | Enables full log replay; catching bugs post-match requires it |
| `RobotState` singleton skeleton | Decouples all subsystems from Drive; prevents cross-subsystem references |
| `Triggers` singleton | Keeps button logic out of RobotContainer; easier to audit bindings |
| Static command factories (`XxxCommands.java`) | Composable, no hidden state, easy to test |
| `AllianceFlipUtil` + caching pattern | Prevents per-loop Optional allocations; red/blue logic in one place |
| `BatteryLogger` per-subsystem current tracking | Critical for diagnosing brownouts; minimal overhead |
| `LoggedTrigger` wrapper | Makes all trigger state visible in AdvantageKit logs |
| `ContinuousConditionalCommand` | Correct solution when command mode must re-evaluate while running |
| `PhoenixUtil` Phoenix config retry helpers | CTRE devices silently ignore configs if the bus is busy; retries fix this |
| `LocalADStarAK` pathfinder | Required for PathPlanner pathfinding with AdvantageKit replay |
| Auto start pose validation | Prevents wrong-position autons; save 5 minutes at every competition |
| `LoggedDashboardChooser` for auto selection | Logs which auto was selected; essential for post-match review |

### Drive Subsystem
The swerve drive subsystem structure is **mechanically stable** across seasons:
- `Drive.java` — copy directly, update `TunerConstants` for new robot
- `Module.java` — copy directly
- `GyroIOPigeon2.java` — copy directly
- `ModuleIOTalonFX.java` — copy directly; update gear ratio constants
- `PhoenixOdometryThread.java` — copy directly
- `DriveCommands.java` — copy the framework; update alignment targets for new game

### Vision Subsystem
The PhotonVision filtering structure is stable:
- `Vision.java` — copy directly; update camera count and names
- `VisionIOPhotonVision.java` — copy directly
- `VisionConstants.java` — update camera positions and calibration transforms

---

## B — Game-Specific Code (Rewrite Each Season)

These files encode the current game. Start from scratch or heavily modify.

| File/Class | What Changes |
|---|---|
| All mechanism subsystems | Entirely new mechanisms every season |
| `ShotCalculator` | Interpolation table is measured against this season's field element |
| `HoodPosCalculator` | Same |
| `HubShiftUtil` | Encodes match-schedule timing windows for this game's scoring |
| `RobotState` zone boundaries | Field coordinates change every season |
| `RobotState` pass targets | Game-specific target locations |
| `HardwareConstants.Zones` | Zone names and boundaries are field-specific |
| `HardwareConstants.CanIds` (mechanisms) | New mechanisms = new CAN devices |
| `ShootSequences.java` | Shooting pipeline is game-specific |
| `SpitSequences.java` | Same |
| PathPlanner `.auto` files | Paths are field-specific |
| Button mappings in `Triggers.java` | Keep the pattern; update what each button does |
| `RobotContainer` named commands | Names and actions change with game |
| Swerve encoder offsets in `TunerConstants` | Re-run Tuner X on the new robot |
| Swerve PID gains | Re-tune on the new robot |
| Vision camera transforms | New robot = new camera positions |

---

## C — Technical Debt (Fix, Don't Copy)

These are known problems in Rebuilt2026 that should not be carried forward.

| Issue | Fix |
|---|---|
| `Vision.java:140` — `maxPoseJumpMeters` filter disabled | Re-enable; tune the threshold against real logs before competition |
| Intake pivot on CANivore without explanation (IDs 41, 44) | Decide intentionally; document in `HardwareConstants.CanIds` |
| `intakeRoller` lowercase-i class name | Rename to `IntakeRoller` / `IntakeRollerCommands` |
| Magic numbers in `DriveCommands.java` (deadband 0.1) | Move to `HardwareConstants` |
| Feeder timeout `1.5 s` not in a named constant | Add `HardwareConstants.CompConstants.Waits.totalFeedTimeoutSeconds` |
| Flywheel/prestage spinup thresholds not in constants | Move to `HardwareConstants` or subsystem constants file |
| No jam detection on transport or intake roller | Add stall current detection or a beam break sensor |
| COMP vs ALPHA robot switch requires editing `Constants.java` | Add a compile-time or deploy-time assertion; consider a hardware ID ping |
| `RobotContainer.java` too long | Split button bindings into `configureBindings()` helper methods per subsystem |
| Pass targets hardcoded in `RobotState` | Consider making them named constants in `HardwareConstants` |

---

## D — Areas Needing Redesign

These are structural patterns that worked but have real design problems.
Address them intentionally at the start of the season, not mid-season.

### 1. `RobotContainer` as monolith
**Problem:** All subsystem wiring + all button bindings in one file.
As the robot grows, this file becomes fragile and hard to review in pull requests.
**Recommended redesign:** Split into `configureFlywheelBindings()`, `configureIntakeBindings()`, etc., each in a separate private method. Or extract a `BindingConfigurator` class that takes all subsystems.

### 2. Implicit shoot-readiness (no state machine)
**Problem:** Shot readiness is determined by `waitUntil(isSpunUp)` + `waitUntil(isAligned)` timeout chaining. There is no explicit state — you cannot ask "what state is the shooter in right now?" without reconstructing it from command tree.
**Recommended redesign:** Consider a `ShooterState` enum (`IDLE`, `SPINNING_UP`, `READY`, `FIRING`) managed by a state machine in a coordinator class. Makes debugging and logging much cleaner.

### 3. Open-loop transport and roller with no feedback
**Problem:** Transport belt and intake roller have no velocity or stall detection. Jams are silent.
**Recommended redesign:** Add supply current monitoring in `periodic()`. Log a warning and optionally cancel intake commands when current spikes above jam threshold for >X ms.

### 4. HubShiftUtil hard-coupling to match schedule
**Problem:** `HubShiftUtil` encodes the specific timing of when hubs are active in this game's schedule. It's opaque and fragile — if the game rules change mid-season, it's easy to miss.
**Recommended redesign:** Replace with a simple `isTargetActive()` boolean driven by FMS data or a dashboard override. `HubShiftUtil`'s internal schedule should be a config file, not hardcoded constants.

### 5. Dual TunerConstants files with manual switch
**Problem:** `COMP_TunerConstants.java` and `ALPHA_TunerConstants.java` are selected by editing `Constants.java`. Easy to deploy wrong constants under competition pressure.
**Recommended redesign:** Auto-detect robot by CAN device ID or a jumper pin. Or add an assertion in `Robot.java` that logs a prominent warning if the robot hardware doesn't match the selected constants.

---

## New Season Checklist

```
[ ] Copy Drive subsystem (update gear ratios + tuner constants for new robot)
[ ] Copy Vision subsystem (update camera positions + transforms)
[ ] Copy util/ (all of it — nothing game-specific here)
[ ] Rewrite all mechanism subsystems (blank slate)
[ ] Rewrite RobotState (keep singleton skeleton, rewrite zone + geometry methods)
[ ] Update HardwareConstants (new CAN IDs, new zones, new tuning values)
[ ] Run Tuner X swerve project generator → regenerate COMP_TunerConstants
[ ] Physically measure camera positions → update VisionConstants transforms
[ ] Re-tune drive PID gains (translation + rotation)
[ ] Re-tune vision std dev thresholds (test with real AprilTags)
[ ] Register new named commands for auto paths
[ ] Create new PathPlanner auto paths
[ ] Re-validate auto start pose check tolerances
[ ] Update button bindings for new mechanism layout
[ ] Fix the debt items from Section C above
[ ] Address at least one redesign item from Section D
```

---

## Folder Structure for a New Season

```
src/main/java/frc/robot/
├── Robot.java               ← copy from template/; update metadata strings
├── RobotContainer.java      ← rewrite; use template pattern
├── RobotState.java          ← copy skeleton; rewrite game-specific methods
├── Triggers.java            ← copy skeleton; update bindings
├── HardwareConstants.java   ← rewrite; carry structure, replace values
├── Constants.java           ← copy; update robot type
├── subsystems/
│   ├── drive/               ← copy entire directory; retune constants
│   ├── vision/              ← copy entire directory; update camera config
│   └── [new mechanisms]/    ← write from scratch using template pattern
├── commands/
│   ├── DriveCommands.java   ← copy; update alignment targets
│   └── [new commands]/      ← write from scratch using template pattern
└── util/                    ← copy entire directory
```
