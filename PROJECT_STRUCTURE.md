# Project Structure

This document shows the complete file structure that will be created when all 24 issues are implemented.

## Overview

```
frc.robot/
├── subsystems/
│   ├── climber/
│   │   ├── Climber.java                  (Issue 3)
│   │   ├── ClimberIO.java                (Issue 1)
│   │   └── ClimberIOTalonFX.java         (Issue 2)
│   │
│   ├── shooter/
│   │   ├── Shooter.java                  (Issue 7)
│   │   ├── ShooterIO.java                (Issue 5)
│   │   └── ShooterIOTalonFX.java         (Issue 6)
│   │
│   ├── hopper/
│   │   ├── Hopper.java                   (Issue 11)
│   │   ├── HopperIO.java                 (Issue 9)
│   │   └── HopperIOTalonFX.java          (Issue 10)
│   │
│   ├── feeder/
│   │   ├── Feeder.java                   (Issue 15)
│   │   ├── FeederIO.java                 (Issue 13)
│   │   └── FeederIOTalonFX.java          (Issue 14)
│   │
│   ├── intake/
│   │   ├── Intake.java                   (Issue 19)
│   │   ├── IntakeIO.java                 (Issue 17)
│   │   └── IntakeIOTalonFX.java          (Issue 18)
│   │
│   └── leds/
│       ├── LEDs.java                     (Issue 23)
│       ├── LEDsIO.java                   (Issue 21)
│       └── LEDsIOAddressableLED.java     (Issue 22)
│
└── commands/
    ├── climber/
    │   ├── RunClimberCommand.java        (Issue 4)
    │   └── StopClimberCommand.java       (Issue 4)
    │
    ├── shooter/
    │   ├── RunShooterCommand.java        (Issue 8)
    │   └── StopShooterCommand.java       (Issue 8)
    │
    ├── hopper/
    │   ├── RunHopperCommand.java         (Issue 12)
    │   └── StopHopperCommand.java        (Issue 12)
    │
    ├── feeder/
    │   ├── RunFeederCommand.java         (Issue 16)
    │   └── StopFeederCommand.java        (Issue 16)
    │
    ├── intake/
    │   ├── RunIntakeCommand.java         (Issue 20)
    │   └── StopIntakeCommand.java        (Issue 20)
    │
    └── leds/
        ├── RunLEDsCommand.java           (Issue 24)
        └── StopLEDsCommand.java          (Issue 24)
```

## File Count

**Total Files**: 36
- **Subsystem Files**: 18 (6 subsystems × 3 files each)
  - 6 IO interfaces
  - 6 RealIO implementations
  - 6 Subsystem classes
- **Command Files**: 18 (6 subsystems × 3 commands each, minimum)
  - 6 Run commands
  - 6 Stop commands
  - Additional commands may be added per subsystem needs

## Implementation Order

### Priority: Medium (Start Here)
1. Define all IO interfaces (Issues 1, 5, 9, 13, 17, 21)
   - This creates the contracts that everything else depends on
   - Can be done in parallel across subsystems

### Priority: Low (After IO Defined)
2. Implement RealIO layers (Issues 2, 6, 10, 14, 18, 22)
   - Requires hardware CAN IDs
   - Can be done in parallel across subsystems

3. Implement Subsystems (Issues 3, 7, 11, 15, 19, 23)
   - Depends on IO interfaces
   - Can be done in parallel across subsystems

4. Implement Commands (Issues 4, 8, 12, 16, 20, 24)
   - Depends on subsystems
   - Can be done in parallel across subsystems

## Package Structure

```
src/main/java/frc/robot/
├── subsystems/
│   ├── climber/
│   ├── shooter/
│   ├── hopper/
│   ├── feeder/
│   ├── intake/
│   └── leds/
└── commands/
    ├── climber/
    ├── shooter/
    ├── hopper/
    ├── feeder/
    ├── intake/
    └── leds/
```

## Dependencies Between Components

```
┌─────────────────┐
│   IO Interface  │ ◄─── Priority: Medium (Define First)
│   (Define)      │
└────────┬────────┘
         │
         │ implements
         ▼
┌─────────────────┐
│   RealIO        │ ◄─── Priority: Low
│   (TalonFX)     │
└────────┬────────┘
         │
         │ uses
         ▼
┌─────────────────┐
│   Subsystem     │ ◄─── Priority: Low
│   (Logic)       │
└────────┬────────┘
         │
         │ controls
         ▼
┌─────────────────┐
│   Commands      │ ◄─── Priority: Low
│   (Operators)   │
└─────────────────┘
```

## Integration Points

Each subsystem integrates with:
1. **Robot.java** - Subsystem instantiation
2. **RobotContainer.java** - Command binding
3. **Constants.java** - CAN IDs and configuration
4. **AdvantageKit** - Automatic logging

## Testing Strategy

For each subsystem:
1. ✅ IO interface compiles
2. ✅ RealIO implementation compiles
3. ✅ Subsystem instantiates without errors
4. ✅ Commands bind to controllers
5. ✅ Telemetry appears in AdvantageScope
6. ✅ Hardware responds to commands (if available)

## Code Review Checklist

For each file:
- [ ] Follows team naming conventions
- [ ] Includes JavaDoc comments
- [ ] Uses proper package structure
- [ ] Implements all required methods
- [ ] Handles edge cases
- [ ] Includes proper error handling
- [ ] Logs important state changes

## Documentation Files Created

This repository includes:
- `README.md` - Main repository documentation
- `QUICK_START.md` - How to run the script
- `ISSUES_README.md` - Detailed issue documentation
- `ISSUE_PREVIEW.md` - Preview of all 24 issues
- `TESTING_CHECKLIST.md` - Verification checklist
- `PROJECT_STRUCTURE.md` - This file
- `create_issues.sh` - Issue creation script
- `issues_manifest.json` - JSON reference

## Next Steps After Issues Created

1. Review all 24 issues in GitHub
2. Assign issues to team members
3. Set up project board (optional)
4. Create milestone for "Basic Subsystems" (optional)
5. Begin implementation with IO Layer issues
6. Regular code reviews
7. Integration testing as subsystems complete
