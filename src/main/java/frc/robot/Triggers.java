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
  // Private on purpose: all drive-axis reads must go through driveXSupplier()/driveYSupplier()/
  // driveRotSupplier() so they respect the XBOX_DRIVE_MODE gate.
  private final CommandJoystick thrustmaster =
      new CommandJoystick(HardwareConstants.ControllerConstants.JoystickControllerPort);
  private final CommandXboxController simController =
      new CommandXboxController(HardwareConstants.ControllerConstants.SimControllerPort);
  private final CommandJoystick simKeyboardController =
      new CommandJoystick(HardwareConstants.ControllerConstants.SimKeyboardControllerPort);

  // ==================== DRIVE-SOURCE GATING ====================
  // Every driver-facing trigger below is built once and routed at poll time based on the
  // latched HardwareConstants.ControllerConstants.XBOX_DRIVE_MODE flag.
  //
  // Why a branch instead of `thrustmaster.button(1).or(controller.rightTrigger())`:
  // an .or() polls BOTH sources every loop. This branch polls exactly one, so the loop
  // cost is identical to the single-controller scheme we had before, plus one boolean
  // field read per trigger.
  //
  // Side benefit: bindings that collide across modes (Xbox A is bumpAlign in drive mode
  // but allianceWinFlipper in override mode) are mutually exclusive for free — only one
  // meaning is ever live.
  private static boolean xboxDrive() {
    return HardwareConstants.ControllerConstants.XBOX_DRIVE_MODE;
  }

  /**
   * Routes a function between the control it lives on in each mode. Note the parameters are
   * MODE-based, not device-based: drive functions move Thrustmaster -> Xbox, while override
   * functions move the opposite way, Xbox -> Thrustmaster.
   */
  private Trigger sourced(Trigger whenThrustmasterDrives, Trigger whenXboxDrives) {
    return new Trigger(
        () -> xboxDrive() ? whenXboxDrives.getAsBoolean() : whenThrustmasterDrives.getAsBoolean());
  }

  // Analog-trigger press threshold for the Xbox shoot/intake triggers.
  private static final double TRIGGER_THRESHOLD = 0.5;

  // ==================== DRIVE AXES ====================
  // Thrustmaster mode: joystick X/Y/twist. Xbox mode: left stick translation, right stick X
  // rotation. RobotContainer applies the sign flips and scaling, same as before.
  public double driveXSupplier() {
    return xboxDrive() ? controller.getLeftX() : thrustmaster.getX();
  }

  public double driveYSupplier() {
    return xboxDrive() ? controller.getLeftY() : thrustmaster.getY();
  }

  public double driveRotSupplier() {
    return xboxDrive() ? controller.getRightX() : thrustmaster.getTwist();
  }

  // Button mapping triggers
  public Trigger shootButton() {
    // RT in Xbox mode
    return sourced(thrustmaster.button(1), controller.rightTrigger(TRIGGER_THRESHOLD));
  }

  public Trigger trenchAlignButton() {
    // R3 in Xbox mode
    return sourced(thrustmaster.button(2), controller.rightStick());
  }

  public Trigger intakeInButton() {
    // LB in Xbox mode
    return sourced(thrustmaster.button(3), controller.leftBumper());
  }

  public Trigger intakeOutButton() {
    // RB in Xbox mode
    return sourced(thrustmaster.button(4), controller.rightBumper());
  }

  public Trigger intakeRollerButton() {
    // LT in Xbox mode
    return sourced(thrustmaster.button(5), controller.leftTrigger(TRIGGER_THRESHOLD));
  }

  public Trigger intakeCompressButton() {
    // No Xbox-mode home — auto-compress on shoot still works in Xbox mode.
    return thrustmaster.button(6);
  }

  public Trigger bumpAlignButton() {
    // A in Xbox mode
    return sourced(thrustmaster.button(8), controller.a());
  }

  public Trigger shootFromTowerButton() {
    // Y in Xbox mode
    return sourced(thrustmaster.button(10), controller.y());
  }

  public Trigger passButton() {
    // No Xbox-mode home.
    return thrustmaster.button(11);
  }

  public Trigger hardstopShootButton() {
    // Only referenced by commented-out code.
    return thrustmaster.button(9);
  }

  public Trigger demoDistanceShot() {
    // DEMO_MODE only.
    return thrustmaster.button(7);
  }

  public Trigger xWheels() {
    // Sim-only binding; not wired in configureStateBindings().
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
  // These move the OPPOSITE direction from the drive functions above: in Xbox drive mode the
  // Thrustmaster becomes the override controller, so alliance-win control moves onto it.
  // This also resolves the A/Y collision — Xbox A is bumpAlign in drive mode and
  // allianceWinFlipper in Thrustmaster mode, and only one is ever live.
  public Trigger allianceWinFlipper() {
    // TM buttons 3 and 4 are BOTH mapped to the flipper in Xbox drive mode, per drive team
    // request. flipWinner() is a toggle, so pressing both in succession is a no-op round trip.
    return sourced(controller.a(), thrustmaster.button(3).or(thrustmaster.button(4)));
  }

  public Trigger allianceWinDisabler() {
    return sourced(controller.y(), thrustmaster.button(2));
  }

  public Trigger autoXOverride() {
    // Already an override on the Thrustmaster; unchanged in both modes.
    return thrustmaster.button(12);
  }

  public Trigger doubleCompressOverride() {
    // Xbox B is unassigned in the Xbox drive layout, so this works in both modes unchanged.
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
