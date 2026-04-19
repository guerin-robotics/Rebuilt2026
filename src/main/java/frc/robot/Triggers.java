package frc.robot;

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
  private final CommandJoystick thrustmaster =
      new CommandJoystick(HardwareConstants.ControllerConstants.JoystickControllerPort);
  private final CommandXboxController simController =
      new CommandXboxController(HardwareConstants.ControllerConstants.SimControllerPort);

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

  public Trigger xWheels() {
    return controller.x();
  }

  public Trigger simShootButton() {
    return thrustmaster.button(1);
    // return controller.b();
  }

  public Trigger simTrenchAlignButton() {
    // return simController.button(2);
    return controller.a(); // not using this button right now
  }

  public Trigger simIntakeInButton() {
    // return simController.button(3);
    return thrustmaster.button(3);
  }

  public Trigger simIntakeOutButton() {
    // return simController.button(4);
    return thrustmaster.button(4);
  }

  public Trigger simIntakeRollerButton() {
    // return simController.button(5);
    return simController.povUp();
  }

  public Trigger simIntakeCompressButton() {
    // return simController.button(6);
    return simController.povDown();
  }

  public Trigger simBumpAlignButton() {
    // return simController.button(7);
    return controller.y(); // not using this button right now
  }

  public Trigger simShootFromTowerButton() {
    // return simController.button(8);
    return controller.x(); // not using this button right now
  }

  public Trigger simPassButton() {
    // return simController.button(9);
    return controller.b(); // not using this button right now
  }

  public Trigger simXWheels() {
    // return simController.button(0);
    return controller.y(); // not using this button right now
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

  // Returns true if robot is able to score fuel at the current match time, or if timer is disabled
  // BUG FIX (issue 1): values are now read inside the lambda every cycle instead of captured once
  public final LoggedTrigger isShootSafeTime =
      new LoggedTrigger(
          "isShootSafeTime",
          () -> HubShiftUtil.getShiftedShiftInfo().active() || HubShiftUtil.disabled);

  // Returns true if time is safe, but NOT if timer is disabled (used for smart idle speeds)
  // BUG FIX (issue 2): value is now read inside the lambda every cycle instead of captured once
  public final LoggedTrigger isShootSafeTimeSure =
      new LoggedTrigger("isShootSafeTimeSure", () -> HubShiftUtil.getShiftedShiftInfo().active());

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
          });
}
