# GitHub Issues Creation Guide

This document describes the 24 GitHub issues that will be created for the Rebuilt2026 robotics project.

## Overview

The project uses AdvantageKit's IO layer pattern for hardware abstraction, following best practices for FRC robot code. We're creating issues for **6 subsystems**, each with **4 implementation phases**.

### Subsystems
1. **Climber** - Robot climbing mechanism
2. **Shooter** - Game piece shooting mechanism
3. **Hopper** - Game piece storage
4. **Feeder** - Feeds game pieces to shooter
5. **Intake** - Picks up game pieces from field
6. **LEDs** - Visual feedback system

### Implementation Phases (per subsystem)

#### Phase 1: Define IO Layer (Size: S, Priority: Medium)
- Create the IO interface following AdvantageKit pattern
- Define sensor inputs using `@AutoLog` annotation
- Define control methods (voltage control, no PID)
- **Files**: `[Subsystem]IO.java`

#### Phase 2: Implement RealIO Layer (Size: M, Priority: Low)
- Implement real hardware interface using CTRE Phoenix 6
- Configure TalonFX motors
- Read sensor data efficiently using status signals
- **Files**: `[Subsystem]IOTalonFX.java`

#### Phase 3: Implement Basic Subsystem (Size: M, Priority: Low)
- Create subsystem class extending SubsystemBase
- Use IO abstraction for hardware independence
- Implement periodic logging
- Add basic control methods
- **Files**: `[Subsystem].java`

#### Phase 4: Implement Basic Subsystem Commands (Size: S, Priority: Low)
- Create command classes for subsystem operations
- Enable operator control
- Follow WPILib command-based patterns
- **Files**: `Run[Subsystem]Command.java`, `Stop[Subsystem]Command.java`, etc.

## Issue Labels

Each issue will have appropriate labels:

- **Size Labels**: 
  - `size: S` - Small (IO definition, Commands)
  - `size: M` - Medium (RealIO, Subsystem implementation)

- **Priority Labels**:
  - `priority: Medium` - IO layer definitions (foundation for other work)
  - `priority: Low` - Implementation tasks (can be done after IO is defined)

## Creating the Issues

### Prerequisites
1. GitHub CLI (`gh`) must be installed
2. You must be authenticated with GitHub (`gh auth login`)
3. You must have permission to create issues in the repository

### Using the Script

```bash
# Make sure you're authenticated
gh auth login

# Run the script
./create_issues.sh
```

The script will:
1. Check that you're authenticated with GitHub CLI
2. Create all 24 issues with proper titles, descriptions, and labels
3. Provide a summary of created issues

### Manual Creation

If you prefer to create issues manually, reference the script content for the exact titles, descriptions, and labels for each issue.

## Issue Structure

Each issue follows the team's standard template:

### What
Description of the task and its purpose

### Implementation
- Detailed implementation guidance
- Skeleton code examples
- Key points to remember

### Definition of Done
Checklist of requirements for issue completion

### Other Resources
Links to relevant documentation and examples

## Technical Details

### AdvantageKit IO Pattern
- **IO Interface**: Defines contract for hardware interaction
- **IO Inputs**: AutoLogged data class for sensor readings
- **RealIO Implementation**: Actual hardware interface
- **SimIO Implementation**: Simulation (not in initial 24 issues)

### CTRE Phoenix 6
- Uses TalonFX motors throughout
- Configuration via TalonFXConfiguration
- Efficient sensor reading with BaseStatusSignal
- Modern controls API (VoltageOut, etc.)

### Code Conventions
- Simplified skeleton examples (students fill in details)
- Comments instead of complete implementations
- No PID or closed-loop control initially
- Focus on basic voltage control
- Follows team naming and structure conventions

## Issue Dependencies

While issues can be worked on in parallel, there's a logical dependency flow:

1. **IO Layer** must be defined before RealIO can be implemented
2. **RealIO** should exist before Subsystem is created
3. **Subsystem** must exist before Commands can be implemented

However, multiple subsystems can be developed in parallel.

## Expected Outcomes

After all 24 issues are completed:
- ✅ 6 subsystems with complete IO abstraction
- ✅ Hardware-independent code (can run in simulation)
- ✅ Automatic logging via AdvantageKit
- ✅ Basic operator control commands
- ✅ Foundation for advanced features (PID, auto, etc.)

## Support

For questions about:
- **AdvantageKit**: https://github.com/Mechanical-Advantage/AdvantageKit
- **Phoenix 6**: https://v6.docs.ctr-electronics.com/
- **WPILib**: https://docs.wpilib.org/

## Complete Issue List

### Climber Subsystem
1. Climber - Define IO Layer (S, Medium)
2. Climber - Implement RealIO Layer (M, Low)
3. Climber - Implement Basic Subsystem (M, Low)
4. Climber - Implement Basic Subsystem Commands (S, Low)

### Shooter Subsystem
5. Shooter - Define IO Layer (S, Medium)
6. Shooter - Implement RealIO Layer (M, Low)
7. Shooter - Implement Basic Subsystem (M, Low)
8. Shooter - Implement Basic Subsystem Commands (S, Low)

### Hopper Subsystem
9. Hopper - Define IO Layer (S, Medium)
10. Hopper - Implement RealIO Layer (M, Low)
11. Hopper - Implement Basic Subsystem (M, Low)
12. Hopper - Implement Basic Subsystem Commands (S, Low)

### Feeder Subsystem
13. Feeder - Define IO Layer (S, Medium)
14. Feeder - Implement RealIO Layer (M, Low)
15. Feeder - Implement Basic Subsystem (M, Low)
16. Feeder - Implement Basic Subsystem Commands (S, Low)

### Intake Subsystem
17. Intake - Define IO Layer (S, Medium)
18. Intake - Implement RealIO Layer (M, Low)
19. Intake - Implement Basic Subsystem (M, Low)
20. Intake - Implement Basic Subsystem Commands (S, Low)

### LEDs Subsystem
21. LEDs - Define IO Layer (S, Medium)
22. LEDs - Implement RealIO Layer (M, Low)
23. LEDs - Implement Basic Subsystem (M, Low)
24. LEDs - Implement Basic Subsystem Commands (S, Low)
