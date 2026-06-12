# Example Profile: 2026-Robot-Code

This author-specific profile encodes known-good defaults for `~/FRC/2026-Robot-Code` on `comp-dev`. Treat it as an example of profile-driven validation, not as a generic setup every team should use unchanged.

## Defaults

- `auto_path=straight`
- `duration=30`
- `record_delay=3`
- `check_ds=true`
- `state_key=/AdvantageKit/RealOutputs/SwerveDrive/currentSystemState`
- `expected_states=FOLLOW_PATH,IDLE`

## Validation Patch Set (Sandbox Only)

The profile applies temporary sandbox edits before sim:

1. `RobotContainer.getAutonomousCommand()` -> `Autos.followPath("straight", true)`
2. `Robot.robotInit()` -> add `DriverStationSim` autonomous/enable/notify calls in SIM mode
3. `settings.gradle` -> comment local `BLine-Lib` include block
4. `Constants.currentMode` -> force `Mode.SIM`

These edits are intended for sandbox validation only.
