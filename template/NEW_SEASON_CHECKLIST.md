# New Season Checklist

Work through this top to bottom. Do not skip steps. Many items have dependencies.

---

## Week 1 — Infrastructure

- [ ] Create new repo from this template (do not fork Rebuilt2026 directly)
- [ ] Copy `util/` verbatim — nothing game-specific here
- [ ] Copy `subsystems/drive/` verbatim — update constants after robot is built
- [ ] Copy `subsystems/vision/` verbatim — update camera config after robot is built
- [ ] Copy `Robot.java`, `Constants.java`, `BuildConstants.java` — update project name
- [ ] Copy `RobotState.java` skeleton (pose/velocity/singleton structure only — delete game-specific geometry)
- [ ] Copy `Triggers.java` skeleton — delete all game-specific bindings
- [ ] Set `Constants.getMode()` → `REAL` for competition, `SIM` for development
- [ ] Confirm AdvantageKit version matches current WPILib year
- [ ] Confirm PathPlannerLib version is current
- [ ] Confirm CTRE Phoenix 6 version is current
- [ ] Confirm PhotonVision version is current

## Week 2 — Hardware Setup

- [ ] Run CTRE Tuner X Swerve Project Generator → regenerate `COMP_TunerConstants.java`
- [ ] Verify all swerve CAN IDs in `HardwareConstants.CanIds` with the physical robot
- [ ] Physically measure all camera positions → update `VisionConstants` transforms
- [ ] Assign CAN IDs for all new mechanism motors — document bus (`Canivore` vs `rio`) in comments
- [ ] Zero all CANcoder magnet offsets for pivoting mechanisms

## Week 3 — Mechanism Subsystems

For each mechanism:
- [ ] Create `subsystems/myMechanism/io/MyMechanismIO.java` (use template)
- [ ] Create `subsystems/myMechanism/io/MyMechanismIOReal.java` (use template)
- [ ] Create `subsystems/myMechanism/io/MyMechanismIOSim.java` (use template)
- [ ] Create `subsystems/myMechanism/MyMechanism.java` (use template)
- [ ] Create `commands/MyMechanismCommands.java` (use template)
- [ ] Wire real/sim in `RobotContainer` based on `Constants.getMode()`
- [ ] Verify AdvantageKit logs appear in Advantage Scope

## Week 4 — Tuning

- [ ] Tune swerve drive PID (translation Kp, rotation Kp)
- [ ] Tune swerve steer PID gains in `COMP_TunerConstants`
- [ ] Build shot interpolation table (`ShotCalculator`) against real field element
- [ ] Build hood angle table (`HoodPosCalculator`) against same distances
- [ ] Tune alignment tolerances in `RobotState` (`hubAlignmentToleranceDegrees`, etc.)
- [ ] Tune vision std dev scaling against real AprilTag observations
- [ ] Re-enable and tune `maxPoseJumpMeters` vision filter (disabled in Rebuilt2026 — start here)
- [ ] Tune feeder wait timeouts (`HardwareConstants.CompConstants.Waits`)

## Week 5 — Auto

- [ ] Register all `NamedCommands` before `AutoBuilder.buildAutoChooser()`
- [ ] Register all `EventTrigger` objects (without subsystem requirements)
- [ ] Create PathPlanner auto paths
- [ ] Test auto start pose check tolerances
- [ ] Validate auto on field with pose visualization

## Pre-Competition

- [ ] Deploy COMP config, verify correct robot selected in `Constants.java`
- [ ] Smoke test all subsystems disabled on carpet
- [ ] Run full auto preview in Advantage Scope
- [ ] Verify battery logger is logging all subsystems
- [ ] Check that `WPILOGWriter` is writing to `/U/logs` (USB drive present)
- [ ] Confirm demo mode is OFF (`HardwareConstants.TuningConstants.DEMO_MODE = false`)
- [ ] Confirm tuning mode is OFF

---

## Debt Items to Fix This Season

From Rebuilt2026 — carry these forward until fixed:

- [ ] Re-enable `maxPoseJumpMeters` vision filter (Vision.java:140)
- [ ] Rename `intakeRoller` → `IntakeRoller` (and commands file)
- [ ] Move drive deadband to `HardwareConstants`
- [ ] Move feeder timeout `1.5 s` to named constant
- [ ] Add stall/jam detection to open-loop transport and roller
- [ ] Add deploy-time assertion for COMP vs ALPHA robot selection
