package frc.robot;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.LoggedTrigger;

public class Triggers {

  /* These are all booleans that we'll use later to determine when commands should be called - mostly for safe
  driving. They use pose, velocity, and zone estimation from RobotState. */

  // Single-instance
  private static Triggers instance;

  public static Triggers getInstance() {
    if (instance == null) {
      instance = new Triggers();
    }
    return instance;
  }

  // Which physical controller drives the real robot. Set the DRIVE_CONTROL_MODE constant below,
  // then redeploy — this is a compile-time switch, not a live dashboard selection.
  //   THRUSTMASTER              - Thrustmaster drives + its game buttons; port-1 Xbox does
  // overrides.
  //   XBOX_CONTROLLER           - Xbox drive pad drives (LT/RT/RB/D-pad/sticks); port-1 Xbox
  // overrides.
  //   XBOX_CONTROLLER_OVERRIDE  - single Xbox drive pad drives AND owns the A/B/X/Y overrides.
  public enum DriveControlMode {
    THRUSTMASTER,
    XBOX_CONTROLLER,
    XBOX_CONTROLLER_OVERRIDE
  }

  // >>> Drive team: change this to pick the active control scheme, then redeploy. <<<
  public static final DriveControlMode DRIVE_CONTROL_MODE = DriveControlMode.THRUSTMASTER;

  // Returns the compile-time selected mode.
  public DriveControlMode driveControlMode() {
    return DRIVE_CONTROL_MODE;
  }

  // True when the Xbox drive pad owns driving + the shoot/intake/pass buttons (either Xbox mode).
  public boolean useXboxDrive() {
    return driveControlMode() != DriveControlMode.THRUSTMASTER;
  }

  // True only in the single-pad mode where the Xbox drive pad also owns the A/B/X/Y overrides.
  public boolean useXboxOverride() {
    return driveControlMode() == DriveControlMode.XBOX_CONTROLLER_OVERRIDE;
  }

  // True only in plain XBOX_CONTROLLER mode: the drive pad owns the game buttons while the
  // A/B/X/Y overrides stay on the port-1 Xbox controller. The face-button game mappings
  // (intake, compress, tower/hardstop shoot) live here so they never collide with the
  // override-mode A/Y/B/LB bindings.
  public boolean useXboxPlain() {
    return driveControlMode() == DriveControlMode.XBOX_CONTROLLER;
  }

  // Controllers
  private final CommandXboxController controller =
      new CommandXboxController(HardwareConstants.ControllerConstants.XboxControllerPort);
  private final CommandJoystick thrustmaster =
      new CommandJoystick(HardwareConstants.ControllerConstants.JoystickControllerPort);
  private final CommandXboxController simController =
      new CommandXboxController(HardwareConstants.ControllerConstants.SimControllerPort);
  private final CommandJoystick simKeyboardController =
      new CommandJoystick(HardwareConstants.ControllerConstants.SimKeyboardControllerPort);

  // Alternate single-Xbox drive controller: LT = intake, RT = shoot, RB = pass, sticks = drive.
  private final CommandXboxController driveController =
      new CommandXboxController(HardwareConstants.ControllerConstants.XboxDriveControllerPort);

  // Button mapping triggers.
  // The three game actions the Xbox scheme owns (shoot/intake/pass) are source-gated on
  // useXboxDrive() so exactly one controller is live: Thrustmaster when off, Xbox when on.
  public Trigger shootButton() {
    return thrustmaster
        .button(1)
        .and(() -> !useXboxDrive())
        .or(driveController.rightTrigger().and(this::useXboxDrive));
  }

  // Thrustmaster button(2) always works. In plain XBOX_CONTROLLER mode the drive pad's D-pad up
  // also triggers trench-align. (Not mapped in override mode, which uses these buttons for
  // overrides.)
  public Trigger trenchAlignButton() {
    return thrustmaster.button(2).or(driveController.povUp().and(this::useXboxPlain));
  }

  public Trigger intakeInButton() {
    return thrustmaster
        .button(3)
        .and(() -> !useXboxDrive())
        .or(driveController.povUp().and(this::useXboxOverride)) // override mode keeps D-pad up
        .or(driveController.y().and(this::useXboxPlain)); // plain mode: Y
  }

  public Trigger intakeOutButton() {
    return thrustmaster.button(4).or(driveController.a().and(this::useXboxPlain)); // plain mode: A
  }

  public Trigger intakeRollerButton() {
    return thrustmaster
        .button(5)
        .and(() -> !useXboxDrive())
        .or(driveController.leftTrigger().and(this::useXboxOverride)) // override mode keeps LT
        .or(driveController.x().and(this::useXboxPlain)); // plain mode: X
  }

  public Trigger intakeCompressButton() {
    return thrustmaster.button(6).or(driveController.b().and(this::useXboxPlain)); // plain mode: B
  }

  public Trigger bumpAlignButton() {
    return thrustmaster.button(8);
  }

  public Trigger shootFromTowerButton() {
    // plain mode: D-pad right or RB (RB freed up now that pass is folded into the shoot button)
    return thrustmaster
        .button(10)
        .or(driveController.povRight().and(this::useXboxPlain))
        .or(driveController.rightBumper().and(this::useXboxPlain));
  }

  // Pass is now handled automatically by the zone-aware shoot button (shoots to hub in the
  // alliance zone, passes when out of it), so RB no longer force-passes. This dedicated
  // force-pass stays on the Thrustmaster for THRUSTMASTER mode only.
  public Trigger passButton() {
    return thrustmaster.button(11).and(() -> !useXboxDrive());
  }

  public Trigger hardstopShootButton() {
    // plain mode: LB (override mode keeps LB on auto-X, so gated to plain only — no double-fire)
    return thrustmaster.button(9).or(driveController.leftBumper().and(this::useXboxPlain));
  }

  public Trigger demoDistanceShot() {
    return thrustmaster.button(7);
  }

  public Trigger xWheels() {
    return controller.x();
  }

  public Trigger simShootButton() {
    // return simController.b();
    return simKeyboardController.button(1);
  }

  public Trigger simTrenchAlignButton() {
    // return simController.button(10);
    return simKeyboardController.button(2);
  }

  public Trigger simIntakeInButton() {
    // return simController.button(11);
    return simKeyboardController.button(3);
  }

  public Trigger simIntakeOutButton() {
    // return simController.button(12);
    return simKeyboardController.button(4);
  }

  public Trigger simIntakeRollerButton() {
    // return simController.povUp();
    return simKeyboardController.button(5);
  }

  public Trigger simIntakeCompressButton() {
    // return simController.povDown();
    return simKeyboardController.button(6);
  }

  public Trigger simBumpAlignButton() {
    return controller.y(); // not using this button right now
  }

  public Trigger simShootFromTowerButton() {
    // return simController.button(8);
    return controller.x(); // not using this button right now
  }

  public Trigger simPassButton() {
    // return simController.button(9);
    return simKeyboardController.button(7);
  }

  public Trigger simXWheels() {
    // return simController.button(0);
    return controller.y(); // not using this button right now
  }

  public Trigger simAllianceWinFlipper() {
    // return simController.a();
    return simKeyboardController.button(8);
  }

  public Trigger simShootOnTheMove() {
    return simKeyboardController.button(9);
  }

  public double simXSupplier() {
    return simKeyboardController.getRawAxis(0);
  }

  public double simYSupplier() {
    return simKeyboardController.getRawAxis(1);
  }

  public double simRotationSupplier() {
    return simKeyboardController.getRawAxis(2);
  }

  // Real-robot drive-axis inputs, source-gated on useXboxDrive().
  // Returned raw (matching the Thrustmaster raw-axis convention); callers apply the same
  // negation/scaling they already used, so both controllers share one sign convention:
  // stick pushed forward/left = robot forward/left, right stick right = clockwise.
  public double driveXInput() {
    return useXboxDrive() ? driveController.getLeftX() : thrustmaster.getRawAxis(0); // strafe
  }

  public double driveYInput() {
    return useXboxDrive() ? driveController.getLeftY() : thrustmaster.getRawAxis(1); // forward
  }

  public double driveRotInput() {
    return useXboxDrive() ? driveController.getRightX() : thrustmaster.getRawAxis(2); // twist
  }

  public Trigger tuningButton() {
    return thrustmaster.button(7);
  }

  // Override triggers.
  // In XBOX_CONTROLLER_OVERRIDE mode the single Xbox drive pad also owns these (A/Y/B, plus
  // auto-X on the otherwise-free left bumper); otherwise they stay on their original devices.
  public Trigger allianceWinFlipper() {
    return controller
        .a()
        .and(() -> !useXboxOverride())
        .or(driveController.a().and(this::useXboxOverride));
  }

  public Trigger allianceWinDisabler() {
    return controller
        .y()
        .and(() -> !useXboxOverride())
        .or(driveController.y().and(this::useXboxOverride));
  }

  public Trigger autoXOverride() {
    return thrustmaster
        .button(12)
        .and(() -> !useXboxOverride())
        .or(driveController.leftBumper().and(this::useXboxOverride));
  }

  public Trigger doubleCompressOverride() {
    return controller
        .b()
        .and(() -> !useXboxOverride())
        .or(driveController.b().and(this::useXboxOverride));
  }

  // ==================== STATE-BASED TRIGGERS (cached as final fields) ====================
  // These are constructed once and reused. The lambda inside each LoggedTrigger is evaluated
  // every 20ms by the scheduler — only the trigger *object* is cached, not the result.

  // Returns true if robot is in our alliance zone (able to score from present position)
  public final LoggedTrigger isShootSafeZone =
      new LoggedTrigger(
          "isShootSafeZone",
          () -> {
            HardwareConstants.Zones.broadZone currentZone = RobotState.getInstance().getBroadZone();
            return (currentZone == HardwareConstants.Zones.broadZone.ALLIANCE_ZONE);
          });

  // Returns true if robot is able to score fuel at the current match time, or if timer is disabled,
  // or if demo mode is on
  // BUG FIX (issue 1): values are now read inside the lambda every cycle instead of captured once
  public final LoggedTrigger isShootSafeTime =
      new LoggedTrigger(
          "isShootSafeTime",
          () ->
              HubShiftUtil.getShiftedShiftInfo().active()
                  || HubShiftUtil.disabled
                  || HardwareConstants.TuningConstants.DEMO_MODE);

  // Composite checker for time and zone — true when both time and zone are safe to shoot
  public final LoggedTrigger isShootClear =
      new LoggedTrigger("isShootClear", isShootSafeTime.and(isShootSafeZone));

  public final LoggedTrigger isAlignedForCurrentShot =
      new LoggedTrigger(
              "isAlignedForCurrentShot",
              () -> {
                if (RobotState.getInstance().getBroadZone()
                    == HardwareConstants.Zones.broadZone.ALLIANCE_ZONE) {
                  return RobotState.getInstance().isAlignedToHub();
                } else {
                  return RobotState.getInstance().isAlignedToPass();
                }
              })
          .debounce(0.3, DebounceType.kRising);

  public final LoggedTrigger isAlignedLooser =
      new LoggedTrigger(
          "isAlignedLooser",
          () -> {
            if (RobotState.getInstance().getBroadZone()
                == HardwareConstants.Zones.broadZone.ALLIANCE_ZONE) {
              return RobotState.getInstance().isAlignedToHubLoose();
            } else {
              return RobotState.getInstance().isAlignedToPassLoose();
            }
          });
}