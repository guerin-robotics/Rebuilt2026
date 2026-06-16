# Command Rules

---

## Factory Method Pattern (Mandatory)

```java
// Every command method:
public static Command doThing(MySubsystem sub, SomeParam param) {
    return Commands.<something>(...)
        .withName("MySubsystem_DoThing");   // ALWAYS name commands
}
```

No exceptions. Never `new SomeCommand(...)` for game logic.

---

## Choosing the Right Factory

| Scenario | Factory to use |
|---|---|
| "Run while button held, stop on release" | `Commands.startEnd(onStart, onEnd, subsystem)` |
| "Do this once and finish" | `Commands.runOnce(action, subsystem)` |
| "Run every loop until interrupted" | `Commands.run(action, subsystem)` |
| "Run A, then B, then C" | `Commands.sequence(A, B, C)` |
| "Run A and B simultaneously" | `Commands.parallel(A, B)` |
| "Run A until condition, then B" | `Commands.sequence(A.until(cond), B)` |
| "Run both, stop when A finishes" | `Commands.deadline(A, B)` |

---

## Timeouts Are Mandatory on `waitUntil`

```java
// CORRECT — always has a timeout escape
Commands.waitUntil(flywheel::isSpunUp).withTimeout(1.5)

// WRONG — hangs forever if condition never becomes true
Commands.waitUntil(flywheel::isSpunUp)
```

This is a competition safety rule. A robot that hangs mid-sequence scores zero points.

---

## The Spinup → Align → Act Pattern

Use this pattern for any action that requires mechanism readiness + robot alignment:

```java
Commands.sequence(
    // Phase 1: wait for mechanism readiness (hard timeout)
    Commands.waitUntil(mechanism::isReady)
        .withTimeout(HardwareConstants.CompConstants.Waits.mechanismReadySeconds),

    // Phase 2: wait for alignment OR give up after budget
    Commands.waitUntil(isAligned)
        .withTimeout(
            HardwareConstants.CompConstants.Waits.totalTimeoutSeconds
            - HardwareConstants.CompConstants.Waits.mechanismReadySeconds),

    // Phase 3: act
    runTheAction()
)
```

Phase 2 timeout = total budget minus phase 1. This guarantees the robot always
fires within the total budget even if alignment never arrives.

All timeout constants must live in `HardwareConstants.CompConstants.Waits` — not inline.

---

## Default Commands

Default commands run when nothing else requires the subsystem. Rules:
- Default commands should be safe idle states (stop, stow, low-speed idle)
- They must not end (use `Commands.run()`, not `Commands.runOnce()`)
- Set in `RobotContainer` via `subsystem.setDefaultCommand()`
- Hood default: stow to 0°
- Intake pivot default: deploy (pivot down)

---

## Command Naming Convention

`.withName()` format: `"SubsystemName_ActionVerb_OptionalParam"`

Examples:
```
"Flywheel_Velocity_2000RPM"
"Hood_PosForHub"
"Transport_VoltageAfterWait"
"Drive_AlignForHub"
"Intake_CompressSingle"
```

Names appear in AdvantageKit's command log. Bad names make debugging impossible.

---

## Named Commands (PathPlanner)

Register in `RobotContainer` before `AutoBuilder.buildAutoChooser()`:

```java
NamedCommands.registerCommand("Shoot", ShootSequences.autoShootToHub(...));
```

Event triggers must NOT declare subsystem requirements:
```java
new EventTrigger("DeployIntake")
    .whileTrue(Commands.runOnce(() -> intakePivot.setPivotPosition(down)));
    //                          ^^ no subsystem arg — intentional
```

**Why:** An event trigger with a subsystem requirement will interrupt the path-following
command that requires the drive subsystem. Removing the requirement lets both run.

---

## ContinuousConditionalCommand

Use this (from `util/`) when a command's mode must be re-evaluated while running:

```java
new ContinuousConditionalCommand(commandIfTrue, commandIfFalse, conditionSupplier)
```

WPILib's `ConditionalCommand` evaluates the condition only at schedule time.
Use `ContinuousConditionalCommand` when the driver can toggle a mode mid-execution
(e.g., single/double compress toggle).

---

## Cancellation Flags

Three boolean flags in `RobotContainer` gate robot behavior:
- `compressCancelled` — prevents auto compress after manual intake override
- `xCancelled` — prevents auto X-wheels after manual override
- `doubleCompress` — toggles compress mode

When modifying command bindings, check whether your change interacts with these flags.
Resetting a flag at the wrong time causes auto-behaviors to re-engage unexpectedly.
