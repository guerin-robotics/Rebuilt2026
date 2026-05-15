# Dependency Graph

All dependencies confirmed by reading source. File:line references included.

---

## 1. Subsystem Dependency Graph

```
┌─────────────────────────────────────────────────────────────────┐
│                        RobotContainer                           │
│  Constructs, wires, and owns all subsystems and commands        │
└───┬──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┬───┘
    │      │      │      │      │      │      │      │      │
    ▼      ▼      ▼      ▼      ▼      ▼      ▼      ▼      ▼
 Drive  Vision Flywheel Hood Prestage Upper Lower Transport Intake
                              Feeder Feeder         Pivot+Roller
```

### Direct Object References (constructor-injected or method-wired)

```
Drive ──────────────────────► RobotState (write only)
                                  Drive.java:158  setPoseSupplier(this::getPose)
                                  Drive.java:216  setModuleStates(...)

RobotState ◄────────────────── Drive (read via supplier)
                                  RobotState reads pose via poseSupplier.get()
                                  RobotState NEVER calls Drive methods directly

Vision ─────────────────────► RobotState (read only)
                                  Vision.java:91  getFieldRelativeVelocity()

Vision ─────────────────────► Drive (write via callback)
                                  RobotContainer.java:159  drive::addVisionMeasurement
                                  Vision holds the callback, not a Drive reference

Flywheel ◄──────────────────── Hood (angle supplier)
                                  RobotContainer.java:229  flywheel.setHoodAngleSupplier(hood::getPosition)
                                  Flywheel holds a Supplier<Angle>, not a Hood reference

Triggers ───────────────────► RobotState (read only)
                                  Triggers.java:146  getZone(), getEstimatedPose()
                                  Triggers.java:171  isAlignedToHub(), isAlignedToPass()

Commands ───────────────────► RobotState (read only)
                                  FlywheelCommands, HoodCommands read distance/angle
                                  DriveCommands reads heading from RobotState

RobotContainer ─────────────► Triggers (singleton read)
                                  Triggers.getInstance() for all button bindings
```

### Subsystem → Subsystem References

**None.** No subsystem directly references another subsystem. All
cross-subsystem coordination passes through commands, suppliers passed at
construction time, or the RobotState singleton.

This is correct WPILib architecture. The risk is that RobotState becomes a
de-facto god object — see risks section.

---

## 2. Command Interaction Map

### Subsystem Requirements by Command

A command that "requires" a subsystem will interrupt any other command currently
running that requires the same subsystem. Requirements below are what each
command actually declares (not what it calls).

| Command | Requires |
|---------|---------|
| `joystickDrive` | Drive |
| `joystickDriveAtAngle` | Drive |
| `alignOrXForShoot` | Drive |
| `stopWithX` | Drive |
| `feedforwardCharacterization` | Drive |
| `wheelRadiusCharacterization` | Drive |
| `FlywheelCommands.*` | Flywheel |
| `HoodCommands.hoodIdle` | Hood |
| `HoodCommands.setHoodPos*` | Hood |
| `PrestageCommands.*` | Prestage |
| `FeederCommands.setUpperFeeder*` | UpperFeeder |
| `FeederCommands.setLowerFeeder*` | LowerFeeder |
| `TransportCommands.*` | Transport |
| `IntakePivotCommands.*` | IntakePivot |
| `intakeRollerCommands.*` | intakeRoller |
| `ShootSequences.autoShootToHub` | Flywheel + Hood + Prestage + UpperFeeder + LowerFeeder + Transport + intakeRoller + IntakePivot |
| `ShootSequences.stopAll` | Flywheel + Hood + Prestage + UpperFeeder + LowerFeeder + Transport + intakeRoller |
| `EventTrigger commands` | **None** (intentional — see risks) |

### Default Commands

| Subsystem | Default Command | Set at |
|-----------|----------------|--------|
| Drive | `DriveCommands.joystickDrive(...)` | RobotContainer |
| Hood | `HoodCommands.hoodIdle(hood)` | RobotContainer |

All other subsystems have no default command. When no command requires them,
their `periodic()` still runs but no motor setpoints are issued.

### Command Conflict Map

The following commands require overlapping subsystems and will interrupt each other:

```
autoShootToHub ─────────────────────────────────────────────────────────────────────────────────────────┐
  requires: Flywheel, Hood, Prestage, UpperFeeder, LowerFeeder, Transport, intakeRoller, IntakePivot     │
                                                                                                         │
Teleop shoot (FlywheelCommands + PrestageCommands) ──────────────────────────────────────────────────────┤ CONFLICT
  requires: Flywheel, Prestage                                                                           │ if teleop
                                                                                                         │ bindings
IntakePivotCommands (whileTrue shoot button) ────────────────────────────────────────────────────────────┤ fire during
  requires: IntakePivot                                                                                  │ auto
                                                                                                         │
intakeRollerCommands (RunIntake named command) ───────────────────────────────────────────────────────────┘
  requires: intakeRoller
```

**Key risk:** If any teleop trigger is accidentally active during autonomous
(e.g., a whileTrue binding whose Trigger condition evaluates to true because
of a stale state), it will interrupt the auto command group. This is mitigated
by WPILib's `autonomousInit()` cancelling all commands, but teleop bindings
remain registered and their Triggers keep polling.

### Event Trigger Conflict Bypass

RobotContainer.java:349–384 intentionally registers event trigger commands
**without subsystem requirements**:

```java
new EventTrigger("DeployIntake").onTrue(
    Commands.runOnce(
        () -> intakePivot.setPivotPosition(...)
        // NO intakePivot passed as requirement
    )
);
```

This allows the event trigger to call hardware methods without the scheduler
declaring a conflict with `autoShootToHub`, which requires `IntakePivot`.

**Risk:** The subsystem's hardware is mutated from two command contexts
simultaneously (the auto group's `IntakePivotCommands.autoPivotCompress` and
the event trigger's `runOnce`). The last setter wins each cycle. No crash, but
unexpected motion is possible if events fire at the wrong time in the sequence.

---

## 3. Singleton / Shared State Map

```
RobotState (singleton)
├── poseSupplier          ← set by Drive (Drive.java:158)
├── getEstimatedPose()    → read by Triggers, Commands, Vision filter
├── getDistanceToHub()    → read by ShotCalculator, HoodPosCalculator lookups
├── getSpeakerAngleRad()  → read by DriveCommands for heading target
├── getFieldRelativeVelocity() → read by Vision.periodic() for spin filter
├── getBroadZone()        → read by Triggers for zone-based shoot logic
└── getModuleStates()     ← set by Drive.periodic()

Triggers (singleton)
├── shootButton
├── intakeInButton, intakeOutButton, compressButton
├── trenchAlignButton, bumpAlignButton
├── towerShotButton, passButton
├── doubleCompressOverride, autoXOverride
├── isShootSafeZone       → reads RobotState.getBroadZone()
├── isShootClear          → isShootSafeZone AND isShootSafeTime
├── isAlignedForCurrentShot → zone-dependent, 1.5° or 7.0°, debounced 0.3s
└── isAlignedLooser       → same but no debounce
```

**Risk:** `Triggers` is also a singleton (`Triggers.getInstance()`).
Its constructor is called in `RobotContainer`, but any other code that calls
`getInstance()` before `RobotContainer` sets up the controllers would get a
partially-initialized singleton. This is safe in practice (everything runs in
`RobotContainer`), but worth noting.

---

## 4. Tight Coupling Points

### High coupling (explicit object references):

| From | To | Mechanism | Risk |
|------|----|-----------|------|
| Drive | RobotState | Direct `getInstance()` call in constructor | RobotState must exist before Drive |
| Vision | RobotState | Direct `getInstance()` in `periodic()` | Same |
| Triggers | RobotState | Direct `getInstance()` in constructor | Same |
| All commands | RobotState | Direct `getInstance()` in execute/initialize | Same |
| RobotContainer | All subsystems | Constructor injection | Fine — RC owns them |

### Loose coupling (suppliers/callbacks):

| From | To | Mechanism |
|------|----|-----------|
| Vision | Drive | `VisionConsumer` callback (`drive::addVisionMeasurement`) |
| Flywheel | Hood | `Supplier<Angle>` (`hood::getPosition`) |
| Drive | RobotState | `Supplier<Pose2d>` (`this::getPose`) |
| PathPlanner | Drive | Four lambda callbacks in `AutoBuilder.configure()` |

### Duplicate Logic

| Logic | Location 1 | Location 2 | Risk |
|-------|-----------|-----------|------|
| Alliance flip check | `Drive.java` (AutoBuilder callback) | `AllianceFlipUtil.shouldFlip()` | Divergence if either is updated |
| Hub alignment check | `Triggers.isAlignedForCurrentShot` | `DriveCommands.alignOrXForShoot` condition | Two implementations of same concept |
| Distance-to-hub | `RobotState.getDistanceToHub()` | `FieldConstants.hubPosition` geometry | Single source is fine; risk is FieldConstants drift |
| Shoot safety | `isShootSafeZone` (field position) | `isShootSafeTime` (hub state) | Two-part gate — both must be correct |

---

## 5. Initialization Order

Order matters because of singletons and AutoBuilder requirements:

```
Robot.java constructor:
  1. Logger setup (AdvantageKit — must be first)
  2. new RobotContainer()
       a. new Drive(...)           → starts PhoenixOdometryThread
                                  → calls RobotState.getInstance().setPoseSupplier()
       b. new Vision(...)          → wires drive::addVisionMeasurement callback
       c. new Flywheel(...)
       d. new Hood(...)
       e. [other shooter subsystems]
       f. new IntakePivot(...)
       g. new intakeRoller(...)
       h. flywheel.setHoodAngleSupplier(hood::getPosition)
       i. AutoBuilder.configure(drive, ...)   ← must happen BEFORE buildAutoChooser
       j. registerNamedCommands()             ← must happen BEFORE buildAutoChooser
       k. registerEventTriggers()
       l. autoChooser = AutoBuilder.buildAutoChooser()
       m. configureButtonBindings()
  3. AutoLogOutputManager.addObject(RobotState.getInstance())
```

**Risk:** Step (i) and (j) must precede (l). If `buildAutoChooser()` is called
before named commands are registered, PathPlanner will load `.auto` files and
fail silently to resolve any named command references. The comments in
`RobotContainer.java:231–235` acknowledge this explicitly.
