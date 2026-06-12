---
name: Replay Testing
description: How to use AdvantageKit replay mode to re-run robot code against a real match log, enabling deterministic bug reproduction and validation of code changes.
---

# Replay Testing

AdvantageKit replay re-runs recorded hardware inputs through the current robot code. The robot runs at full speed without hardware. This is the primary tool for validating that a code change fixes a real match bug without changing behavior elsewhere.

## How Replay Works

1. The real robot writes all hardware inputs to a `.wpilog` (via `WPILOGWriter`)
2. In replay mode, `WPILOGReader` feeds those exact inputs back into the subsystems
3. The robot code runs deterministically — same inputs, same outputs every time
4. A new `_sim.wpilog` is written with the replay outputs alongside the originals
5. Open both in AdvantageScope and compare: real outputs vs. replayed outputs

## Setting Up Replay Mode

In [Constants.java](../../src/main/java/frc/robot/Constants.java), set:
```java
public static final Mode currentMode = Mode.REPLAY;
```

Then run:
```bash
./gradlew simulateJava
```

When prompted, select the `.wpilog` file to replay. The replay log is written to the same directory with `_sim` appended to the filename.

**Reset to SIM or REAL before deploying.** Never leave `REPLAY` mode as the active constant.

## Available Match Logs

| File | Event | Match |
|---|---|---|
| `akit_26-03-21_15-19-00_incol_q4.wpilog` | Indiana Colonials | Q4 |
| `akit_26-03-21_16-32-11_incol_q12.wpilog` | Indiana Colonials | Q12 |

## Workflow: Reproduce a Bug

1. Identify the timestamp range from AdvantageScope where the issue occurred
2. Set `Constants.currentMode = Mode.REPLAY`
3. Run `./gradlew simulateJava`, select the log
4. Add temporary `Logger.recordOutput()` calls around the suspected code
5. Open the `_sim.wpilog` and look for unexpected values
6. Fix the code, replay again, confirm the output changes as expected
7. Reset `Constants.currentMode` back to `SIM` or `REAL`
8. Run `./gradlew compileJava` and `./gradlew spotlessCheck`

## Workflow: Validate a Code Change

After making a change to command logic or RobotState:
1. Replay Q4 log — verify shot sequences still fire at the right times
2. Replay Q12 log — cross-check on a different match
3. Compare `isAlignedForCurrentShot`, `isFlywheelSpunUp`, `RobotState/shooting` between real and replayed outputs
4. Any divergence between real and replayed outputs indicates the code change altered behavior

## Key Invariant

`Logger.processInputs()` must be called in every subsystem's `periodic()`. Without it, AdvantageKit cannot inject the replayed inputs and the replay produces garbage data. This is a hard constraint — never remove these calls.

## Comparing Logs in AdvantageScope

1. Open the original log
2. File → Open Log (additional) → select `_sim.wpilog`
3. Pin both on the same timeline
4. Look for divergence in: `RobotState/IsAlignedToHub`, `isFlywheelSpunUp`, `RobotState/shooting`, command active states
