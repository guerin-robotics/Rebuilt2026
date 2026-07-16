package frc.robot;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.LoggedTrigger;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Triggers {

  // Which physical controller drives the real robot's translation/rotation and its
  // shoot/intake/trench buttons. Selectable live from the dashboard (no redeploy needed) — see
  // refreshControlScheme(). Defaults to Thrustmaster, the competition-proven scheme.
  public enum ControlScheme {
    THRUSTMASTER,
    XBOX_CONTROLLER
  }

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

  // Controllers
  private final CommandXboxController controller =
      new CommandXboxController(HardwareConstants.ControllerConstants.XboxControllerPort);
  public final CommandJoystick thrustmaster =
      new CommandJoystick(HardwareConstants.ControllerConstants.JoystickControllerPort);
  private final CommandXboxController simController =
      new CommandXboxController(HardwareConstants.ControllerConstants.SimControllerPort);
  private final CommandJoystick simKeyboardController =
      new CommandJoystick(HardwareConstants.ControllerConstants.SimKeyboardControllerPort);

  // Alternate single-Xbox drive controller. Left stick = translate, right stick = rotate,
  // RT = shoot/pass, LT = intake roller, LB = intake out, RB = intake in, right stick click =
  // trench align. Live only when the "Drive Control Scheme" chooser selects XBOX_CONTROLLER.
  private final CommandXboxController driveController =
      new CommandXboxController(HardwareConstants.ControllerConstants.XboxDriveControllerPort);

  private final LoggedDashboardChooser<ControlScheme> controlSchemeChooser;

  // Cached by refreshControlScheme(), called once per teleop enable from Robot.teleopInit().
  // Never read the chooser directly from a periodic loop — repeated NetworkTables reads at 50 Hz
  // saturated the RIO CPU when DriverPresets tried it (see DriverPresets javadoc / PR #89 revert).
  private boolean xboxDriveActive = false;

  private Triggers() {
    controlSchemeChooser = new LoggedDashboardChooser<>("Drive Control Scheme");
    controlSchemeChooser.addDefaultOption("Thrustmaster", ControlScheme.THRUSTMASTER);
    controlSchemeChooser.addOption("Xbox Controller", ControlScheme.XBOX_CONTROLLER);
  }

  /**
   * Reads the selected drive control scheme and caches it. Call exactly once per teleop enable from
   * {@code Robot.teleopInit()} — never from a periodic loop. To switch schemes before a match: pick
   * it in Elastic/Shuffleboard, then disable and re-enable teleop.
   */
  public void refreshControlScheme() {
    ControlScheme selected = controlSchemeChooser.get();
    xboxDriveActive = selected == ControlScheme.XBOX_CONTROLLER;
    Logger.recordOutput("Triggers/XboxDriveActive", xboxDriveActive);
  }

  private boolean useXboxDrive() {
    return xboxDriveActive;
  }

  // Real-robot drive-axis inputs, source-gated on the cached control scheme. Returned with the
  // same raw sign convention the Thrustmaster already used, so callers keep negating/scaling
  // exactly as before regardless of which controller is live.
  public double driveXInput() {
    return xboxDriveActive ? driveController.getLeftX() : thrustmaster.getX(); // strafe
  }

  public double driveYInput() {
    return xboxDriveActive ? driveController.getLeftY() : thrustmaster.getY(); // forward
  }

  public double driveRotInput() {
    return xboxDriveActive ? driveController.getRightX() : thrustmaster.getTwist(); // twist
  }

  // Button mapping triggers
  // Also fires on the Xbox drive controller's right trigger when that scheme is active. The
  // existing zone-aware shoot logic already decides shoot-to-hub vs. pass based on alliance zone,
  // so one button covers both — no separate Xbox pass binding is needed.
  public Trigger shootButton() {
    return thrustmaster
        .button(1)
        .and(() -> !useXboxDrive())
        .or(driveController.rightTrigger().and(this::useXboxDrive));
  }

  // Also fires on the Xbox drive controller's right stick click when that scheme is active.
  public Trigger trenchAlignButton() {
    return thrustmaster
        .button(2)
        .and(() -> !useXboxDrive())
        .or(driveController.rightStick().and(this::useXboxDrive));
  }

  // Also fires on the Xbox drive controller's right bumper when that scheme is active.
  public Trigger intakeInButton() {
    return thrustmaster
        .button(3)
        .and(() -> !useXboxDrive())
        .or(driveController.rightBumper().and(this::useXboxDrive));
  }

  // Also fires on the Xbox drive controller's left bumper when that scheme is active.
  public Trigger intakeOutButton() {
    return thrustmaster
        .button(4)
        .and(() -> !useXboxDrive())
        .or(driveController.leftBumper().and(this::useXboxDrive));
  }

  // Also fires on the Xbox drive controller's left trigger when that scheme is active.
  public Trigger intakeRollerButton() {
    return thrustmaster
        .button(5)
        .and(() -> !useXboxDrive())
        .or(driveController.leftTrigger().and(this::useXboxDrive));
  }

  public Trigger intakeCompressButton() {
    return thrustmaster.button(6);
  }

  public Trigger bumpAlignButton() {
    return thrustmaster.button(8);
  }

  public Trigger shootFromTowerButton() {
    return thrustmaster.button(10);
  }

  public Trigger passButton() {
    return thrustmaster.button(11);
  }

  public Trigger hardstopShootButton() {
    return thrustmaster.button(9);
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

  public Trigger tuningButton() {
    return thrustmaster.button(7);
  }

  // Override triggers
  public Trigger allianceWinFlipper() {
    return controller.a();
  }

  public Trigger allianceWinDisabler() {
    return controller.y();
  }

  public Trigger autoXOverride() {
    return thrustmaster.button(12);
  }

  public Trigger doubleCompressOverride() {
    return controller.b();
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
