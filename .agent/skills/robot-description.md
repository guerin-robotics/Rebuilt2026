---
name: Robot Description
description: High-level overview of the Guerin Robotics 2026 competition robot — subsystems, hardware, command patterns, and key architectural constraints.
---

# Robot Description

Guerin Robotics FRC 2026 competition robot. Competed at Worlds. Swerve drive, hub shooter with distance-based RPM lookup, AdvantageKit IO layer throughout.

## Subsystems

| Subsystem | Purpose | Key Hardware |
|---|---|---|
| Drive | 4-module swerve (SDS MK5n) | 8x TalonFX, Pigeon2, 4x CANcoder |
| Flywheel | Hub/pass shooter | 1 leader + 4 followers TalonFX |
| Hood | Shooter angle pivot | TalonFX + CANcoder (FusedCANcoder) |
| IntakePivot | Deploy/retract intake arm | TalonFX + CANcoder |
| IntakeRoller | Ball intake | 2x TalonFX |
| Prestage | Staging wheels before feeder | 2x TalonFX |
| Transport | Belt from intake to prestage | 1x TalonFX (open loop) |
| UpperFeeder / LowerFeeder | Feed balls into flywheel | 2x TalonFX |
| Vision | AprilTag pose estimation | 4x PhotonVision cameras |

## Key Architecture Points

- **AdvantageKit IO layer** — every subsystem wraps hardware behind `XxxIO`. Real/sim/replay IO swap at startup. Never put TalonFX calls inside the subsystem class itself.
- **RobotState singleton** — single source of truth for pose, zone, alignment booleans. No subsystem references another subsystem directly.
- **Static command factories** — all commands are `static` methods returning `Command`. No named command classes.
- **Triggers singleton** — all `Trigger` / `LoggedTrigger` objects live in `Triggers.java`.

## Shot Routing

- Hub shot: flywheel RPM from `ShotCalculator` (distance → RPM interpolation), hood angle from `HoodPosCalculator`
- Pass shot: separate RPM/angle tables, target coordinates hardcoded in `RobotState.getPassTarget()`
- Tower shot: fixed RPM ~1625 RPM at ~110 in from hub center
- Alignment: `RobotState.isAlignedToHub()` / `isAlignedToPass()` with degree tolerances from `HardwareConstants.CompConstants.Thresholds`

## Shoot Sequence Pattern

```
Phase 1 (parallel forever): spin flywheel + prestage + set hood
Phase 2 (gate):             waitUntil(flywheel::isSpunUp).withTimeout(spinUpTimeOut)
Phase 3 (after gate):       start feeders + transport + agitate intake
```

Total shot budget is capped. If alignment never comes, robot fires anyway.

## Key Files

- [ARCHITECTURE.md](../../ARCHITECTURE.md) — full architecture reference
- [src/main/java/frc/robot/HardwareConstants.java](../../src/main/java/frc/robot/HardwareConstants.java) — all CAN IDs, tuning constants, zone boundaries
- [src/main/java/frc/robot/RobotState.java](../../src/main/java/frc/robot/RobotState.java) — pose, zones, alignment
- [src/main/java/frc/robot/commands/ShootSequences.java](../../src/main/java/frc/robot/commands/ShootSequences.java) — composite shot commands
- [docs/hardware-layout.md](../../docs/hardware-layout.md) — CAN ID table

## Constraints for Code Changes

- Never change CAN IDs, encoder offsets, or PID gains without explicit confirmation
- All new subsystems need an IO interface + XxxIOSim stub
- Commands must use static factory pattern with `.withName()`
- See [CLAUDE.md](../../CLAUDE.md) for the full safety hierarchy before making any change
