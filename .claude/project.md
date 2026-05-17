# FRC Robot Project Instructions

You are working on a competitive FIRST Robotics Competition (FRC) Java robot codebase.

The project uses:
- WPILib Java
- Command-based framework
- Phoenix 6
- AdvantageKit
- PathPlanner
- PhotonVision

Your responsibilities:
- Learn the existing architecture before making changes
- Preserve current subsystem ownership patterns
- Preserve logging and telemetry systems
- Avoid architectural rewrites unless explicitly requested
- Prefer extending existing systems over creating new abstractions

---

# Architecture Requirements

Before modifying code:
1. Inspect subsystem structure
2. Inspect IO layer structure
3. Inspect constants organization
4. Inspect RobotContainer bindings
5. Inspect autonomous framework
6. Inspect logging conventions
7. Inspect current command lifecycle patterns

Always identify:
- State machines
- Scheduler interactions
- Hardware abstraction layers
- Vision integration flow
- Pose estimation flow
- Driver/operator control flow

---

# Coding Standards

## General
- Use modern WPILib APIs only
- Never invent APIs
- Avoid deprecated APIs
- Prefer immutable constants
- Use SI units internally
- Avoid unnecessary object allocation in loops

## Subsystems
- Subsystems should remain hardware-oriented
- Commands should contain robot behavior
- Preserve existing subsystem responsibilities
- Avoid singleton abuse

## Commands
- Prefer deterministic command flow
- Avoid blocking logic
- Avoid Thread.sleep
- Use command composition appropriately

## Performance
- Optimize for RoboRIO 2.0
- Minimize CAN bus spam
- Avoid repeated calculations in periodic loops
- Preserve AdvantageKit logging compatibility

---

# Repository Learning Instructions

When first analyzing the repository:
1. Create a mental model of subsystem relationships
2. Identify all hardware devices and CAN IDs
3. Identify drivetrain architecture
4. Identify autonomous framework structure
5. Identify vision pipeline structure
6. Identify tuning/configuration locations
7. Identify operator controls and bindings
8. Identify reusable utilities

Then generate:
- docs/architecture.md
- docs/subsystems.md
- docs/commands.md
- docs/hardware.md
- docs/autonomous.md
- docs/vision.md
- docs/tuning.md

Only document things that actually exist in the repository.

---

# Safety Rules

Never:
- Remove logging without instruction
- Change public interfaces unnecessarily
- Rewrite unrelated files
- Invent fake hardware APIs
- Change scheduler behavior unexpectedly
- Create duplicate subsystems
- Break AdvantageKit logging structure

Always:
- Explain major architectural changes
- Keep changes localized
- Preserve existing naming conventions
- Verify imports and APIs


# Code Review Requirements

All generated code must pass CodeRabbit review standards.

Prioritize:
- deterministic robot behavior
- scheduler correctness
- low allocation periodic loops
- AdvantageKit compatibility
- replay compatibility
- CAN efficiency
- safe command ownership

Avoid:
- blocking calls
- fake APIs
- unnecessary abstraction
- duplicated subsystem ownership
- excessive logging
- unsafe static state

All changes should:
- preserve architecture
- preserve logging
- preserve autonomous behavior
- preserve simulation support
