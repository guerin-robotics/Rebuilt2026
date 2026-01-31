# Rebuilt2026

## GitHub Issues Setup

This repository includes automation for creating 24 GitHub issues for implementing robot subsystems.

### Quick Start

1. Authenticate with GitHub CLI: `gh auth login`
2. Run the script: `./create_issues.sh`
3. Verify issues at: https://github.com/guerin-robotics/Rebuilt2026/issues

### Documentation

- **[QUICK_START.md](QUICK_START.md)** - Quick guide to creating issues
- **[ISSUES_README.md](ISSUES_README.md)** - Detailed documentation of all 24 issues
- **[create_issues.sh](create_issues.sh)** - Automated issue creation script
- **[issues_manifest.json](issues_manifest.json)** - JSON reference of all issues

### Overview

The project implements 6 robot subsystems using AdvantageKit's IO layer pattern:

1. **Climber** - Robot climbing mechanism
2. **Shooter** - Game piece shooting mechanism
3. **Hopper** - Game piece storage
4. **Feeder** - Feeds game pieces to shooter
5. **Intake** - Picks up game pieces from field
6. **LEDs** - Visual feedback system

Each subsystem has 4 implementation phases:
- Define IO Layer (Size: S, Priority: Medium)
- Implement RealIO Layer (Size: M, Priority: Low)
- Implement Basic Subsystem (Size: M, Priority: Low)
- Implement Basic Subsystem Commands (Size: S, Priority: Low)

### Technical Details

- **Framework**: AdvantageKit IO pattern for hardware abstraction
- **Motors**: CTRE TalonFX with Phoenix 6 API
- **Architecture**: Command-based programming (WPILib)
- **Focus**: Simplified implementations with skeleton examples
- **Control**: Basic voltage control (no PID initially)