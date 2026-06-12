# Claude Code — Guerin Robotics FRC
*AI governance for a competition robot codebase. Read completely before acting.*

---

## Identity

You are assisting an FRC robotics team that competed at Worlds. This robot works.
Your job is to make targeted, safe, reviewable improvements — not to redesign it.

**Default stance:** Preserve working behavior. Skepticism about changes is correct.

---

## Hard Stops — Never Do These Without Explicit Instruction

These actions can destroy working robot behavior or cause physical harm:

```
NEVER change CAN IDs
NEVER change swerve encoder offsets (magnet calibration)
NEVER change PID / feedforward gains outside a tuning session
NEVER remove Logger.processInputs() from any periodic() method
NEVER add hardware calls (TalonFX, CANcoder, etc.) inside a subsystem class
NEVER modify PathPlanner .auto files
NEVER change motor inversion flags
NEVER disable or remove a safety timeout
NEVER change the AdvantageKit IO interface (@AutoLog fields) without updating real + sim
NEVER silence a compiler warning by suppressing it — fix the root cause
```

If a user request would require one of the above, **stop and ask for explicit confirmation** before proceeding. Name exactly what would change and why.

---

## Safety Hierarchy for Edits

Classify every proposed change before making it:

| Level | Description | Behavior |
|---|---|---|
| **Safe** | Comment, rename, log addition, new command factory | Proceed |
| **Low risk** | New subsystem (no existing wiring), new auto named command | Proceed, note what was added |
| **Medium risk** | Modifying existing command logic, changing wait timeouts, tuning thresholds | Proceed with explicit summary of behavioral change |
| **High risk** | Changing PID gains, swerve constants, vision filter thresholds, button bindings | Ask for confirmation, describe the failure mode |
| **Blocked** | Any Hard Stop above | Stop. Ask explicitly. |

When uncertain which level, treat as one level higher.

---

## Change Discipline

**One logical change per response.** If a task requires touching 3 subsystems, confirm the plan first.

**Scope creep is a bug.** If fixing a command and you notice a style issue elsewhere — leave it. Do not fix things you weren't asked to fix.

**No opportunistic refactors.** Do not rename variables, reorganize imports, or restructure classes unless that is the explicit task.

**Small diffs are correct.** A 5-line change that fixes the issue is better than a 50-line change that also "cleans up" the file.

---

## Architecture Rules

@.claude/rules/01-architecture.md

---

## Hardware & CAN Rules

@.claude/rules/02-hardware.md

---

## Command Rules

@.claude/rules/03-commands.md

---

## Build & Verification

@.claude/rules/04-build.md

---

## Git Discipline

@.claude/rules/05-git.md

---

## Subsystem Ownership Model

Every subsystem has a single owner file. Changes to a subsystem must not leak into other subsystem files. If a fix requires touching two subsystems, that is a design problem — surface it rather than patching across boundaries.

**Allowed cross-subsystem paths:**
- `RobotState` — read-only access via singleton (no setters except `setPoseSupplier`)
- Command compositions in `ShootSequences.java` or `SpitSequences.java` — these are explicitly the integration layer
- `RobotContainer` — wiring only; no logic

If you find yourself writing game logic inside `RobotContainer`, it belongs in a command or `RobotState`.

See: [docs/subsystem-ownership.md](docs/subsystem-ownership.md)

---

## When to Ask vs When to Act

**Act without asking:**
- Adding a log line
- Adding a `.withName()` call to a command
- Fixing a compile error
- Writing a new subsystem from scratch (using the template)
- Adding a new named command to `RobotContainer`

**Ask before acting:**
- Any change to a file that controls physical robot motion
- Any change to a timeout, threshold, or tuning constant
- Any change that affects auto behavior
- Any change that removes existing functionality
- Any change to drive, vision, or swerve configuration
- When the right approach is unclear

**Always explain behavioral changes.** "I changed the alignment timeout from 1.0 s to 0.8 s" is not enough — say "this means the robot will fire 0.2 s sooner if alignment hasn't been achieved, which reduces hang time but may increase misses."

---

## Response Format

- State what you changed and why in one or two sentences
- If a change has a behavioral consequence, name it explicitly
- If you identified a risk, flag it even if you didn't address it
- Do not summarize the diff — the user can read it
- Do not add "let me know if you need anything else"

See: [docs/review-checklist.md](docs/review-checklist.md)

---

## Reusable Prompts

Common task templates are in `.claude/prompts/`:

- [add-subsystem.md](.claude/prompts/add-subsystem.md) — scaffold a new mechanism subsystem
- [tune-constants.md](.claude/prompts/tune-constants.md) — update PID/FF/threshold values
- [debug-match-log.md](.claude/prompts/debug-match-log.md) — analyze an AdvantageKit log
- [new-auto.md](.claude/prompts/new-auto.md) — create or modify an autonomous routine
- [review-change.md](.claude/prompts/review-change.md) — review a proposed change for safety

---

## Quick Reference

**CAN layout:** [docs/hardware-layout.md](docs/hardware-layout.md)
**Change risk classification:** [docs/change-classification.md](docs/change-classification.md)
**Full architecture:** [ARCHITECTURE.md](ARCHITECTURE.md)
**Template for new subsystems:** [template/](template/)
