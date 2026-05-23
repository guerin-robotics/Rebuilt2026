# Architecture Rules

These rules encode decisions made during the 2025/2026 season that must not be 
reversed without deliberate team discussion. They exist because the alternatives 
were tried and caused problems.

---

## AdvantageKit IO Layer

**Rule:** Every subsystem must wrap hardware behind an `XxxIO` interface.

```
CORRECT                          WRONG
──────────────────────           ──────────────────────
class Flywheel {                 class Flywheel {
  FlywheelIO io;                   TalonFX motor;
  void periodic() {                void periodic() {
    io.updateInputs(inputs);         motor.get...   ← HARDWARE IN SUBSYSTEM
    Logger.processInputs(...)      }
  }                              }
}
```

**Why:** This enables AdvantageKit log replay. If hardware calls are in the subsystem, 
you cannot replay a match log to reproduce a bug. We caught real match bugs this way.

**Corollary:** `XxxIOSim` must exist for every subsystem. It can be stubs. 
Without it, the robot cannot run in simulation.

---

## RobotState Singleton

**Rule:** No subsystem may hold a reference to another subsystem.

All shared state (pose, distances, zone classification, alignment booleans) must 
go through `RobotState.getInstance()`.

```
WRONG:  flywheel.setSpeedForPose(drive.getPose())   ← subsystem ↔ subsystem
CORRECT: flywheel.setSpeedForPose()                 ← reads from RobotState internally via ShotCalculator
CORRECT: supplier callback in constructor            ← RobotContainer passes drive::getPose to a subsystem
```

**Why:** Subsystem cross-references create initialization order dependencies, 
circular logic, and make unit testing impossible.

**Exception:** `RobotContainer` is the wiring layer — it may hold references to all subsystems 
for the purpose of passing them to commands and composing bindings.

---

## Static Command Factories

**Rule:** Commands are static factory methods, not classes.

```java
// CORRECT
public class FlywheelCommands {
    public static Command runAtVelocity(Flywheel flywheel, AngularVelocity velocity) {
        return Commands.run(() -> flywheel.setVelocity(velocity), flywheel)
            .withName("Flywheel_Velocity");
    }
}

// WRONG
public class RunFlywheelAtVelocityCommand extends Command {
    private final Flywheel flywheel;
    ...
}
```

**Why:** Static factories are composable, testable, and traceable in AdvantageKit logs. 
Named command classes create unnecessary files and hide composition structure.

---

## Triggers Singleton

**Rule:** All `Trigger` and `LoggedTrigger` objects live in `Triggers.java`. 
`RobotContainer` reads from `Triggers.getInstance()` — it never creates triggers.

**Why:** Button logic and state triggers are easier to audit in one place. 
We caught timing bugs by being able to see all trigger conditions in one file.

---

## AllianceFlipUtil

**Rule:** Never call `DriverStation.getAlliance()` in a hot path (any method called 
more than once per match).

Use `AllianceFlipUtil.shouldFlip()` which caches the result once per loop.

**Why:** `DriverStation.getAlliance()` returns `Optional<Alliance>` and allocates on 
every call. At 50 Hz this creates GC pressure and unpredictable periodic() timing.

---

## PathPlanner Wiring

**Rule:** `AutoBuilder.configure()` lives in `Drive.java`, not `RobotContainer`.

**Rule:** Named commands and event triggers must be registered before 
`AutoBuilder.buildAutoChooser()` is called.

**Rule:** Event triggers use `Commands.runOnce()` without subsystem requirements to 
avoid interrupting the path-following command.

**Why:** PathPlanner resolves named commands at chooser-build time. Commands registered 
after that call are silently ignored.

---

## Pose Estimator

**Rule:** There is exactly one `SwerveDrivePoseEstimator` in the codebase — inside `Drive.java`. 
`RobotState` delegates to it via `poseSupplier`.

**Why:** Two independent estimators diverge when vision is lost and cause pose jumps 
when they resync. This was a real bug in an earlier version of this codebase.

---

## No Opportunistic Cleanup

**Rule:** Do not change code outside the scope of the current task.

If you see a magic number, a style issue, or commented-out code while working on 
something else — note it, do not fix it. Unrequested changes hide in reviews and 
introduce silent regressions.
