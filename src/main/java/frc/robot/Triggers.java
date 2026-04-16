package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.prestage.Prestage;
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

  // ==================== SUBSYSTEM REFERENCES (for shoot-readiness) ====================
  // Populated once by configure() before the robot starts operating.
  private Flywheel flywheel;
  private Prestage prestage;

  /**
   * Wires up the Flywheel and Prestage subsystem references that the shoot-readiness triggers
   * depend on. Call this once from RobotContainer after both subsystems are created:
   *
   * <pre>Triggers.getInstance().configure(flywheel, prestage);</pre>
   *
   * @param flywheel The robot's flywheel subsystem
   * @param prestage The robot's prestage subsystem
   */
  public void configure(Flywheel flywheel, Prestage prestage) {
    this.flywheel = flywheel;
    this.prestage = prestage;
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
    return simController.b();
    // return controller.b();
  }

  public Trigger simTrenchAlignButton() {
    // return simController.button(2);
    return controller.a(); // not using this button right now
  }

  public Trigger simIntakeInButton() {
    // return simController.button(3);
    return simController.povLeft();
  }

  public Trigger simIntakeOutButton() {
    // return simController.button(4);
    return simController.povRight();
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

  // ==================== STATE-BASED TRIGGERS (cached as final fields) ====================
  // These are constructed once and reused. The lambda inside each LoggedTrigger is evaluated
  // every 20ms by the scheduler — only the trigger *object* is cached, not the result.

  // Returns true if robot is in our alliance zone (able to score from present position)
  public final LoggedTrigger isShootSafeZone =
      new LoggedTrigger(
          "isShootSafeZone",
          () -> {
            HardwareConstants.Zones.broadZone currentZone =
                RobotState.getInstance().getBroadZone(RobotState.getInstance().getEstimatedPose());
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

  // REMOVED: isHoodSafe(Pose2d) — was commented out at its only call site (Hood.java line 42).
  // Also had a design flaw: created a NEW LoggedTrigger object every call, which means GC
  // pressure every loop if it were ever re-enabled. The Pose2d parameter was also captured
  // once rather than re-evaluated, so the trigger would never update.

  // ==================== SHOOT-READINESS TRIGGERS ====================
  // These gates prevent the feeder and transport from starting until the flywheel and prestage
  // are within an acceptable RPM tolerance of their closed-loop setpoints.
  //
  // Tolerances are tunable constants in HardwareConstants.CompConstants.Thresholds.
  // A safety timeout (readyToShootTimeoutSeconds) is applied wherever these triggers are used
  // with Commands.waitUntil() so that shooting is never permanently blocked.

  /**
   * True when the flywheel and prestage are within hub-shot tolerance of their setpoints.
   *
   * <p>Used to gate feeder/transport commands for hub shots. Defaults to {@code true} if {@link
   * #configure} has not been called yet (fail-safe).
   */
  public final LoggedTrigger isReadyToShootHub =
      new LoggedTrigger(
          "isReadyToShootHub",
          () -> {
            if (flywheel == null || prestage == null) return true;
            return flywheel.isFlywheelAtSetpoint(
                    HardwareConstants.CompConstants.Thresholds.hubFlywheelToleranceRPM)
                && prestage.isPrestageAtSetpoint(
                    HardwareConstants.CompConstants.Thresholds.prestageToleranceRPM);
          });

  /**
   * True when the flywheel and prestage are within pass-shot tolerance of their setpoints.
   *
   * <p>Used to gate feeder/transport commands for pass shots. Defaults to {@code true} if {@link
   * #configure} has not been called yet (fail-safe).
   */
  public final LoggedTrigger isReadyToPass =
      new LoggedTrigger(
          "isReadyToPass",
          () -> {
            if (flywheel == null || prestage == null) return true;
            return flywheel.isFlywheelAtSetpoint(
                    HardwareConstants.CompConstants.Thresholds.passFlywheelToleranceRPM)
                && prestage.isPrestageAtSetpoint(
                    HardwareConstants.CompConstants.Thresholds.prestageToleranceRPM);
          });
}
