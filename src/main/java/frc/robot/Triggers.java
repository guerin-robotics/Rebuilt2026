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

  // Controllers
  private final CommandXboxController controller =
      new CommandXboxController(HardwareConstants.ControllerConstants.XboxControllerPort);
  public final CommandJoystick thrustmaster =
      new CommandJoystick(HardwareConstants.ControllerConstants.JoystickControllerPort);
  private final CommandXboxController simController =
      new CommandXboxController(HardwareConstants.ControllerConstants.SimControllerPort);
  private final CommandJoystick simKeyboardController =
      new CommandJoystick(HardwareConstants.ControllerConstants.SimKeyboardControllerPort);

  // Button mapping triggers
  public Trigger shootButton() {
    return thrustmaster.button(1);
  }

  public Trigger trenchAlignButton() {
    return thrustmaster.button(2);
  }

  public Trigger intakeInButton() {
    return thrustmaster.button(3);
  }

  public Trigger intakeOutButton() {
    return thrustmaster.button(4);
  }

  public Trigger intakeRollerButton() {
    return thrustmaster.button(5);
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
