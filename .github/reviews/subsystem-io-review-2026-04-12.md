# Subsystem & IO Layer Code Review — April 12, 2026

## Summary

There are several **real bugs** that will cause incorrect behavior on the robot and in simulation. The most critical are: a copy-paste error that overwrites leader velocity with follower velocity in the intake roller, follower alignment mismatches between real and sim in the flywheel, a missing input field in the sim, a swapped ClosedLoopReference/Error field in the prestage sim, and duplicate AdvantageKit log keys that will silently overwrite each other. The IO pattern overall is solid and consistently applied, but the sim layers have drifted from their real counterparts in several places.

---

## Critical Issues

### 1. ✅ FIXED BY USER — intakeRollerIOReal — Leader velocity overwritten by follower velocity
**File:** `intakeRollerIOReal.java`, lines 150 and 159

`inputs.intakeRollerVelocity` is written twice. Line 150 correctly sets it to the leader velocity, but line 159 immediately overwrites it with the follower velocity. The second assignment should be `inputs.intakeRollerFollowerVelocity`.

### 2. FlywheelIOSim — Follower alignment does not match real hardware
**File:** `FlywheelIOPhoenix6.java` vs `FlywheelIOSim.java`

| Motor | Real (Phoenix6) | Sim |
|-------|----------------|-----|
| follower1 | **Aligned** | **Opposed** ❌ |
| follower2 | **Aligned** | **Aligned** ✓ |
| follower3 | Opposed | Opposed ✓ |
| follower4 | Opposed | Opposed ✓ |

`follower1` is `Aligned` in real hardware but `Opposed` in sim.

### 3. intakeRollerIOSim — Follower alignment does not match real hardware
**File:** `intakeRollerIOReal.java` vs `intakeRollerIOSim.java`

Real hardware sets the follower to `Opposed`, but sim sets it to `Aligned`.

### 4. PrestageIOSim — `prestageLeftClosedLoopReference` reads from ClosedLoopError instead of ClosedLoopReference
**File:** `PrestageIOSim.java`, line 146

```java
inputs.prestageLeftClosedLoopReference =
    RotationsPerSecond.of(prestageLeft.getClosedLoopError().getValueAsDouble());
```
Should be `prestageLeft.getClosedLoopReference()`.

### 5. PrestageIOSim — `prestageLeftClosedLoopError` is never set
Because the left reference reads from the error signal, the left error field is never written. It will be `null`.

### 6. LowerFeeder & UpperFeeder — Duplicate `Logger.processInputs` key `"Feeder"`
**Files:** `LowerFeeder.java:25` and `UpperFeeder.java:25`

Both subsystems call `Logger.processInputs("Feeder", inputs)` with the same key. AdvantageKit will overwrite one subsystem's inputs with the other's each loop cycle.

---

## Warnings

### 7. FlywheelIOSim — `leaderAngle` (position) not populated
The real `FlywheelIOPhoenix6.updateInputs()` sets `inputs.leaderAngle` but `FlywheelIOSim.updateInputs()` never sets it. It will remain `null`.

### 8. FlywheelIO — All `@AutoLog` `Inputs` fields default to `null`
Unlike `HoodIOInputs` (which initializes every field), `ShooterIOInputs` leaves all `Measure` fields at Java default `null`. Same issue for `IntakePivotIOInputs`, `intakeRollerIOInputs`, `LowerFeederIOInputs`, `PrestageIOInputs`, `UpperFeederIOInputs`.

### 9. IntakePivotIOSim — NeutralMode is Coast, Real is Brake
The real motor is `Brake`, sim is `Coast`. For an arm mechanism with gravity this means different behavior when commands end.

### 10. PrestageIOSim — Right motor not driven by `setPrestageVoltage` / `setPrestageVelocity`
In real robot, right motor is a follower. In sim, right motor is independent but never commanded. It will always read 0.

### 11. ✅ REMOVED BY USER — PrestageIO `setOneVelo` had no implementation

### 12. HoodIOSim — No physics simulation (USER: skip for now)

### 13. ✅ REMOVED BY USER — HoodIO `stopHood()` was unimplemented

### 14. IntakePivotIOSim — Uses Voltage-based control (USER: leave as-is, intentional)

### 15. IntakePivotIOReal — Feedback config applied separately (USER: leave as-is, works)

### 16. FlywheelConstants.Sim.NUM_MOTORS = 5, but only 4+1 motors exist
Misleading constant, not a functional bug.

---

## Suggestions

- **S1.** Naming conventions: `intakeRoller*` classes use lowercase, violating Java PascalCase.
- **S2.** `TransportIOInputs` field naming uses PascalCase instead of camelCase.
- **S3.** `IntakePivot.intakeJostleByCurrent`: `new WaitCommand(seconds)` is constructed but never scheduled.
- **S4.** `IntakePivot.intakeJostleByPos`: Same issue with unused `WaitCommand`.
- **S5.** `IntakePivot.intakeHome`: Current check `> 0.5` seems inverted for homing logic.
- **S6.** `intakeRollerIOSim` — Missing follower input population (all follower fields null in sim).
- **S7.** `LowerFeederIOSim`, `UpperFeederIOSim`, `TransportIOSim` — Missing `pos` field population.
