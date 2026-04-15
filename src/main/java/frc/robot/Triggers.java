package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.AllianceFlipUtil;
import frc.lib.FieldConstants;
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

  // ── Intake safety sub-triggers (private, used only by isIntakeSafe) ──────────

  private final LoggedTrigger tooCloseToAllianceHub =
      new LoggedTrigger(
          "tooCloseToAllianceHub",
          () ->
              (RobotState.getInstance().getDistanceToAllianceHub().magnitude()
                  < HardwareConstants.hubDangerZone.intakeOffset));

  private final LoggedTrigger tooCloseToOpposingHub =
      new LoggedTrigger(
          "tooCloseToOpposingHub",
          () ->
              (RobotState.getInstance().getDistanceToOpposingHub().magnitude()
                  < HardwareConstants.hubDangerZone.intakeOffset));

  // BUG FIX (issue 4): use current estimated pose instead of a null parameter
  private final LoggedTrigger facingAllianceHub =
      new LoggedTrigger(
          "facingAllianceHub",
          () -> {
            Pose2d pose = RobotState.getInstance().getEstimatedPose();
            double heading = AllianceFlipUtil.apply(pose.getRotation()).getDegrees();
            HardwareConstants.Zones.specificZone currentSpecificZone =
                RobotState.getInstance().getSpecificZone(pose);
            HardwareConstants.Zones.broadZone currentBroadZone =
                RobotState.getInstance().getBroadZone(pose);
            if (((currentBroadZone == HardwareConstants.Zones.broadZone.ALLIANCE_ZONE)
                    && (Math.abs(heading) < 90))
                || ((currentBroadZone == HardwareConstants.Zones.broadZone.NEUTRAL)
                    && (Math.abs(heading) >= 90))
                || (currentSpecificZone == HardwareConstants.Zones.specificZone.ALLIANCE_BUMP_FAR
                    && (heading < 0))
                || (currentSpecificZone == HardwareConstants.Zones.specificZone.ALLIANCE_BUMP_NEAR
                    && (heading > 0))) {
              return true;
            } else {
              return false;
            }
          });

  // BUG FIX (issue 4): use current estimated pose instead of a null parameter
  private final LoggedTrigger facingOpposingHub =
      new LoggedTrigger(
          "facingOpposingHub",
          () -> {
            Pose2d pose = RobotState.getInstance().getEstimatedPose();
            double heading = AllianceFlipUtil.apply(pose.getRotation()).getDegrees();
            HardwareConstants.Zones.specificZone currentSpecificZone =
                RobotState.getInstance().getSpecificZone(pose);
            HardwareConstants.Zones.broadZone currentBroadZone =
                RobotState.getInstance().getBroadZone(pose);
            if (((currentBroadZone == HardwareConstants.Zones.broadZone.NEUTRAL)
                    && (Math.abs(heading) < 90))
                || ((currentBroadZone == HardwareConstants.Zones.broadZone.OPPOSING_ZONE)
                    && (Math.abs(heading) >= 90))
                || (currentSpecificZone == HardwareConstants.Zones.specificZone.OPPOSING_BUMP_FAR
                    && (heading < 0))
                || (currentSpecificZone == HardwareConstants.Zones.specificZone.OPPOSING_BUMP_NEAR
                    && (heading > 0))) {
              return true;
            } else {
              return false;
            }
          });

  private final LoggedTrigger movingTowardAllianceHub =
      new LoggedTrigger(
          "movingTowardAllianceHub",
          () -> {
            HardwareConstants.Zones.specificZone currentSpecificZone =
                RobotState.getInstance()
                    .getSpecificZone(RobotState.getInstance().getEstimatedPose());
            HardwareConstants.Zones.broadZone currentBroadZone =
                RobotState.getInstance().getBroadZone(RobotState.getInstance().getEstimatedPose());
            double currentXVelocity =
                RobotState.getInstance().getFieldRelativeVelocity().vxMetersPerSecond;
            double currentYVelocity =
                RobotState.getInstance().getFieldRelativeVelocity().vyMetersPerSecond;
            if (((currentBroadZone == HardwareConstants.Zones.broadZone.ALLIANCE_ZONE)
                    && AllianceFlipUtil.applyX(currentXVelocity) >= AllianceFlipUtil.applyX(0))
                || ((currentBroadZone == HardwareConstants.Zones.broadZone.NEUTRAL)
                    && (currentXVelocity) <= AllianceFlipUtil.applyX(0))
                || (currentSpecificZone == HardwareConstants.Zones.specificZone.ALLIANCE_BUMP_NEAR
                    && AllianceFlipUtil.applyY(currentYVelocity) >= AllianceFlipUtil.applyY(0))
                || (currentSpecificZone == HardwareConstants.Zones.specificZone.ALLIANCE_BUMP_FAR
                    && (currentYVelocity) <= AllianceFlipUtil.applyY(0))) {
              return true;
            } else {
              return false;
            }
          });

  private final LoggedTrigger movingTowardOpposingHub =
      new LoggedTrigger(
          "movingTowardOpposingHub",
          () -> {
            HardwareConstants.Zones.specificZone currentSpecificZone =
                RobotState.getInstance()
                    .getSpecificZone(RobotState.getInstance().getEstimatedPose());
            HardwareConstants.Zones.broadZone currentBroadZone =
                RobotState.getInstance().getBroadZone(RobotState.getInstance().getEstimatedPose());
            double currentXVelocity =
                RobotState.getInstance().getFieldRelativeVelocity().vxMetersPerSecond;
            double currentYVelocity =
                RobotState.getInstance().getFieldRelativeVelocity().vyMetersPerSecond;
            if (((currentBroadZone == HardwareConstants.Zones.broadZone.OPPOSING_ZONE)
                    && AllianceFlipUtil.applyX(currentXVelocity) <= AllianceFlipUtil.applyX(0))
                || ((currentBroadZone == HardwareConstants.Zones.broadZone.NEUTRAL)
                    && AllianceFlipUtil.applyX(currentXVelocity) >= AllianceFlipUtil.applyX(0))
                || (currentSpecificZone == HardwareConstants.Zones.specificZone.OPPOSING_BUMP_NEAR
                    && AllianceFlipUtil.applyY(currentYVelocity) >= AllianceFlipUtil.applyY(0))
                || (currentSpecificZone == HardwareConstants.Zones.specificZone.OPPOSING_BUMP_FAR
                    && AllianceFlipUtil.applyY(currentYVelocity) <= AllianceFlipUtil.applyY(0))) {
              return true;
            } else {
              return false;
            }
          });

  // ── Intake safety composite trigger ──────────────────────────────────────────
  // BUG FIX (issue 5): Returns true when intake IS safe (not in danger of crashing into a hub).
  // The sub-conditions detect the DANGER state (close + facing + moving toward hub), then we
  // negate so that true = safe. Callers (e.g., driveLucasProof) expect true = safe.
  public final LoggedTrigger isIntakeSafe =
      new LoggedTrigger(
          "isIntakeSafe",
          (tooCloseToAllianceHub.and(facingAllianceHub).and(movingTowardAllianceHub))
              .or(tooCloseToOpposingHub.and(facingOpposingHub).and(movingTowardOpposingHub))
              .negate());

  // Returns true if the robot's estimated pose is NOT in a trench zone and NOT moving towards one
  public LoggedTrigger isHoodSafe(Pose2d pose) {
    return new LoggedTrigger(
        "isHoodSafe",
        () -> {
          HardwareConstants.Zones.approachingZoneComposite currentSpecificZone =
              RobotState.getInstance().getApproachingZone(pose);
          HardwareConstants.Zones.broadZone currentBroadZone =
              RobotState.getInstance().getBroadZone(pose);
          boolean unsafe =
              ((currentBroadZone == HardwareConstants.Zones.broadZone.ALLIANCE_TRENCH)
                  || (currentBroadZone == HardwareConstants.Zones.broadZone.OPPOSING_TRENCH)
                  || (currentSpecificZone
                      == HardwareConstants.Zones.approachingZoneComposite
                          .APPROACHING_ALLIANCE_TRENCH)
                  || (currentSpecificZone
                      == HardwareConstants.Zones.approachingZoneComposite
                          .APPROACHING_OPPOSING_TRENCH));
          return !unsafe;
        });
  }

  // ==================== SHOOTING ALIGNMENT TRIGGERS ====================

  // True when the robot's heading is within hubAlignmentToleranceDegrees of the alliance hub.
  public final LoggedTrigger isAlignedToHubTarget =
      new LoggedTrigger(
          "isAlignedToHubTarget", () -> RobotState.getInstance().isAlignedToHub());

  // True when the robot's heading is within passAlignmentToleranceDegrees of the pass target.
  public final LoggedTrigger isAlignedToPassTarget =
      new LoggedTrigger(
          "isAlignedToPassTarget", () -> RobotState.getInstance().isAlignedToPass());

  /**
   * True when the robot is aligned well enough to start feeding for the current shot type.
   *
   * <ul>
   *   <li>In alliance zone → checks hub alignment
   *   <li>Outside alliance zone → checks pass-target alignment
   * </ul>
   *
   * <p>Used as the "ready to feed" gate in the shooting sequence. Feeding will start when this
   * returns true, or after {@code alignmentTimeoutSeconds} regardless, to prevent stalling a shot.
   */
  public final LoggedTrigger isAlignedForCurrentShot =
      new LoggedTrigger(
          "isAlignedForCurrentShot",
          () -> {
            if (isShootSafeZone.getAsBoolean()) {
              // In our alliance zone — we're doing a hub shot, check hub alignment
              return RobotState.getInstance().isAlignedToHub();
            } else {
              // Outside alliance zone — we're doing a pass, check pass target alignment
              return RobotState.getInstance().isAlignedToPass();
            }
          });

  // ==================== ZONE TRIGGERS (cached as final fields) ====================
  // All zone checkers require review - not practical for high-velocity zone changes

  // In trench - position-based
  public final LoggedTrigger isRobotInTrench =
      new LoggedTrigger(
          "isRobotInTrench",
          () -> {
            HardwareConstants.Zones.broadZone currentZone =
                RobotState.getInstance().getBroadZone(RobotState.getInstance().getEstimatedPose());
            switch (currentZone) {
              case ALLIANCE_TRENCH, OPPOSING_TRENCH:
                return true;
              default:
                return false;
            }
          });

  // Close to trench - currently position-based, consider adding velocity-based
  public final LoggedTrigger isRobotApproachingTrench =
      new LoggedTrigger(
          "isRobotApproachingTrench",
          () -> {
            HardwareConstants.Zones.approachingZoneComposite currentZone =
                RobotState.getInstance()
                    .getApproachingZone(RobotState.getInstance().getEstimatedPose());
            switch (currentZone) {
              case APPROACHING_ALLIANCE_TRENCH, APPROACHING_OPPOSING_TRENCH:
                return true;
              default:
                return false;
            }
          });

  // On bump - position checker
  public final LoggedTrigger isRobotOnBump =
      new LoggedTrigger(
          "isRobotOnBump",
          () -> {
            HardwareConstants.Zones.specificZone currentZone =
                RobotState.getInstance()
                    .getSpecificZone(RobotState.getInstance().getEstimatedPose());
            switch (currentZone) {
              case ALLIANCE_BUMP_FAR, ALLIANCE_BUMP_NEAR, OPPOSING_BUMP_FAR, OPPOSING_BUMP_NEAR:
                return true;
              default:
                return false;
            }
          });

  // Near bump - currently position-based, consider adding velocity-based
  public final LoggedTrigger isRobotApproachingBump =
      new LoggedTrigger(
          "isRobotApproachingBump",
          () -> {
            HardwareConstants.Zones.approachingZoneComposite currentZone =
                RobotState.getInstance()
                    .getApproachingZone(RobotState.getInstance().getEstimatedPose());
            switch (currentZone) {
              case APPROACHING_ALLIANCE_BUMP, APPROACHING_OPPOSING_BUMP:
                return true;
              default:
                return false;
            }
          });

  // In tower - position-based
  public final LoggedTrigger isRobotInTower =
      new LoggedTrigger(
          "isRobotInTower",
          () -> {
            HardwareConstants.Zones.specificZone currentZone =
                RobotState.getInstance()
                    .getSpecificZone(RobotState.getInstance().getEstimatedPose());
            switch (currentZone) {
              case ALLIANCE_TOWER, OPPOSING_TOWER:
                return true;
              default:
                return false;
            }
          });

  // Near tower - currently position-based, consider adding velocity-based
  public final LoggedTrigger isRobotApproachingTower =
      new LoggedTrigger(
          "isRobotApproachingTower",
          () -> {
            HardwareConstants.Zones.approachingZoneComposite currentZone =
                RobotState.getInstance()
                    .getApproachingZone(RobotState.getInstance().getEstimatedPose());
            switch (currentZone) {
              case APPROACHING_ALLIANCE_TOWER, APPROACHING_OPPOSING_TOWER:
                return true;
              default:
                return false;
            }
          });

  // At or approaching wall - position checker, split into at and approaching, consider adding
  // velocity-based
  public final LoggedTrigger isRobotAtWall =
      new LoggedTrigger(
          "isRobotAtWall",
          () -> {
            double currentX = RobotState.getInstance().getEstimatedPose().getX();
            double currentY = RobotState.getInstance().getEstimatedPose().getY();
            if (((currentX > (FieldConstants.fieldLength - HardwareConstants.Zones.zoneOffset))
                    && (currentY
                            < (FieldConstants.Tower.oppRightUpright.getY()
                                - HardwareConstants.Zones.zoneOffset)
                        || currentY
                            > (FieldConstants.Tower.oppLeftUpright.getY()
                                - HardwareConstants.Zones.zoneOffset)))
                || ((currentX < HardwareConstants.Zones.zoneOffset)
                    && (currentY
                            < (FieldConstants.Tower.oppRightUpright.getY()
                                - HardwareConstants.Zones.zoneOffset)
                        || currentY
                            > (FieldConstants.Tower.oppLeftUpright.getY()
                                - HardwareConstants.Zones.zoneOffset)))
                || (currentY > (FieldConstants.fieldWidth - HardwareConstants.Zones.zoneOffset))
                || (currentY < HardwareConstants.Zones.zoneOffset)) {
              return true;
            } else {
              return false;
            }
          });
}
