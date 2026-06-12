# Build & Verification Rules

---

## Before Reporting a Task Complete

Run these checks in order. Do not skip.

### 1. Compile Check
```bash
./gradlew compileJava
```
Every change must compile. Do not deliver code with compile errors.

### 2. AdvantageKit Annotation Processor
If you added or modified an `@AutoLog` inputs class, check that the annotation
processor generated the updated `*AutoLogged` class:
```bash
./gradlew generateSources   # or: ./gradlew build -x test
```

### 3. Code Formatting
```bash
./gradlew spotlessCheck
# If formatting issues found:
./gradlew spotlessApply
```
The repo uses Google Java Format via Spotless. Non-formatted code will fail CI.

### 4. Full Build (before any commit that touches hardware or command logic)
```bash
./gradlew build
```

---

## What Each Check Catches

| Check | What it catches |
|---|---|
| `compileJava` | Type errors, missing methods, wrong imports |
| `generateSources` | `@AutoLog` changes not reflected in AutoLogged class |
| `spotlessCheck` | Formatting inconsistencies |
| Full `build` | Everything above + any test failures |

---

## Simulation Verification

For changes to command logic, trigger logic, or RobotState calculations —
verify in simulation before declaring complete:

```bash
./gradlew simulateJava
```

Check in Advantage Scope (connected to sim via NT4):
- Subsystem inputs are logging correctly
- Commands activate and deactivate as expected
- RobotState values update correctly

---

## Things That Don't Replace Build Verification

- "It looks right" — does not catch type errors
- "I only changed a constant" — constants have types; wrong type breaks build
- "The logic is correct" — logic is irrelevant if it doesn't compile
- "I've done this before" — always verify

---

## Annotation Processor Note

AdvantageKit uses annotation processing to generate `*AutoLogged` classes.
If you add a field to an `@AutoLog` class (e.g., `FlywheelIOInputs`),
the `FlywheelIOInputsAutoLogged` class is regenerated at build time.

Do not manually edit `*AutoLogged` files — they are overwritten on every build.

If a compile error says `*AutoLogged` is missing a field you just added:
```bash
./gradlew clean generateSources
```

---

## Deploy Verification (physical robot only)

Before declaring a change ready to deploy:
1. Compile locally: `./gradlew compileJava`
2. Check `Constants.getMode()` == `REAL`
3. Check `Constants.getRobotType()` == `COMP` (or the intended robot)
4. Check `HardwareConstants.TuningConstants.DEMO_MODE` == `false`
5. Check `HardwareConstants.TuningConstants.TUNING_MODE` == `false` (unless intentional)
6. Verify the USB drive is present for log writing

---

## Build Flags and JVM Options

The robot's `build.gradle` sets:
- Heap: 100 MB max
- GC: SerialGC with 50 ms max pause target
- Log output: `/U/logs` on USB drive

Do not change JVM flags without understanding the tradeoff. The current settings
were tuned to prevent GC pauses during match play.
