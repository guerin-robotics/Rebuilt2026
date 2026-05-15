# Tuning and Configuration

---

## Tuning Mode

`HardwareConstants.TuningConstants.TUNING_MODE` is the master toggle. When
`true`, subsystems that have distance-based lookup tables (Flywheel, Hood)
instead use fixed tuning values so a shooter can be dialed in manually.

Auto-disable logic in `Robot.java`:

```java
if (HardwareConstants.TuningConstants.atComp) {
    if (DriverStation.isFMSAttached()) {
        TUNING_MODE = false;   // Always off during actual matches
    } else {
        TUNING_MODE = isTuning; // Allowed at practice / pit
    }
}
```

| Constant | Default | Meaning |
|----------|---------|---------|
| `TUNING_MODE` | `false` | Master tuning override toggle |
| `atComp` | `true` | FMS-attached auto-disable |
| `isTuning` | `false` | Practice field tuning enable |
| `FlywheelTuningVelocity` | 2000 RPM | Flywheel RPM when tuning |
| `HoodTuningPos` | 12.25° | Hood angle when tuning |

---

## NetworkTable Tunables

### Flywheel tuning RPM

```java
// Flywheel.java
tuningRPM = new LoggedNetworkNumber("Tune/flywheel/tuningRPM", 20);
```

- NT entry: `Tune/flywheel/tuningRPM`
- Visible and editable from SmartDashboard or Shuffleboard during a match or test
- `LoggedNetworkNumber` wraps `NetworkTableEntry` and logs changes via AdvantageKit
  so tuning sessions can be replayed later
- Only active when `TUNING_MODE == true`

---

## Distance-Based Shot Tables

### ShotCalculator (Flywheel)

`src/main/java/frc/robot/subsystems/flywheel/ShotCalculator.java`

Holds an interpolation table mapping `distanceMeters → RPM`. Called by
`FlywheelCommands.setVelocityForHub()`:

```java
double distance = RobotState.getInstance().getDistanceToHub();
double rpm = ShotCalculator.getRPM(distance);
flywheel.setFlywheelVelocity(RPM.of(rpm));
```

### HoodPosCalculator (Hood)

`src/main/java/frc/robot/subsystems/hood/HoodPosCalculator.java`

Same pattern — interpolation table mapping `distanceMeters → hoodAngleDegrees`.
Called by `HoodCommands.setHoodPosForHub()`.

Both tables are hand-tuned empirically by shooting at various distances and
logging results. The interpolation linearly blends between data points.

---

## Fixed Shot Constants

For scenarios where distance-based lookup is not needed:

### Tower Shot (`HardwareConstants.TowerConstants`)

| Parameter | Value |
|-----------|-------|
| Flywheel | 1625 RPM |
| Hood | 2.5° |

### Pass Shot (`HardwareConstants.PassConstants`)

| Parameter | Value |
|-----------|-------|
| Flywheel | 2700 RPM |
| Hood | 35° |

---

## SysId Routines

SysId characterization routines exist in `DriveCommands.java` for the drivetrain:

- `sysIdQuasistatic(drive, direction)` — slowly ramps voltage
- `sysIdDynamic(drive, direction)` — step voltage

These are defined but **commented out** of the auto chooser in `RobotContainer`:

```java
// autoChooser.addOption("Drive SysId (Quasistatic Forward)",
//     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
```

To use: uncomment the relevant options, deploy, run the routine from the
dashboard, and analyze the resulting `.wpilog` with WPILib SysId tooling.

---

## Auto Delay

A configurable autonomous delay is provided via SmartDashboard:

```java
double delay = SmartDashboard.getNumber(autoDelayKey, 0.0);
return Commands.sequence(
    Commands.waitSeconds(delay),
    autoChooser.get().asProxy()
);
```

Key name: `autoDelayKey` (constant in `RobotContainer`). Default: 0 seconds.
Can be set from the dashboard before a match to coordinate with alliance partners.

---

## Phoenix Motor Configuration

Motor controller gains (kP, kI, kD, kV, kS, kA) are set inside the `*IOReal`
classes via Phoenix 6 `TalonFXConfiguration`. To change a gain:

1. Update the configuration in the relevant `*IOReal` constructor or
   configuration method.
2. Deploy and test.
3. Optionally, export the final configuration via Phoenix Tuner X and update
   `TunerConstants.java` if it affects swerve drive modules.

---

## AdvantageKit Replay for Tuning

Replay mode (`Constants.Mode.REPLAY`) lets you re-run robot logic against a
recorded `.wpilog` without hardware:

1. Collect a match or test log (logged to USB drive automatically during real
   robot operation).
2. Copy the `.wpilog` file to the project root (or set the path in `Robot.java`
   replay block).
3. Change `Constants.currentMode` to `REPLAY`.
4. Deploy to a roboRIO or run in simulation.
5. Robot code re-runs all logic using recorded inputs; new outputs are written
   to `*_sim.wpilog`.
6. Open both logs in AdvantageScope to compare behavior before/after a code
   change.

This is especially useful for:
- Testing shot table changes without driving to the field
- Debugging vision filter behavior
- Verifying auto timing changes against real match data

---

## Logging Dashboard Values

AdvantageKit-logged dashboard keys (readable in AdvantageScope or on NT
dashboards):

| Key | Content |
|-----|---------|
| `Auto/StartCheck/DistanceInches` | Distance from expected start pose |
| `Auto/StartCheck/PositionOK` | Boolean: within 6 inch tolerance |
| `RobotState/EstimatedPose` | Current 2D robot pose |
| `Drive/ActivePath` | Currently followed PathPlanner path |
| `Drive/TargetPose` | Instantaneous target pose from path controller |
| `Tune/flywheel/tuningRPM` | Current tuning RPM (NetworkTable) |
| `Flywheel/IsSpunUp` | Boolean: flywheel at speed |
