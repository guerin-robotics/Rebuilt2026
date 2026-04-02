package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

  // Button mapping triggers
  public LoggedTrigger shootButton() {
    return new LoggedTrigger("Triggers/shootButton", thrustmaster.button(1));
  }

  public LoggedTrigger trenchAlignButton() {
    return new LoggedTrigger("Triggers/trenchAlignButton", thrustmaster.button(2));
  }

  public LoggedTrigger intakeInButton() {
    return new LoggedTrigger("Triggers/intakeInButton", thrustmaster.button(3));
  }

  public LoggedTrigger intakeOutButton() {
    return new LoggedTrigger("Triggers/intakeOutButton", thrustmaster.button(4));
  }

  public LoggedTrigger intakeRollerButton() {
    return new LoggedTrigger("Triggers/intakeRollerButton", thrustmaster.button(5));
  }

  public LoggedTrigger intakeCompressButton() {
    return new LoggedTrigger("Triggers/intakeCompressButton", thrustmaster.button(6));
  }

  public LoggedTrigger bumpAlignButton() {
    return new LoggedTrigger("Triggers/bumpAlignButton", thrustmaster.button(8));
  }

  public LoggedTrigger shootFromTowerButton() {
    return new LoggedTrigger("Triggers/shootFromTowerButton", thrustmaster.button(10));
  }

  public LoggedTrigger passButton() {
    return new LoggedTrigger("Triggers/passButton", thrustmaster.button(11));
  }

  public LoggedTrigger xWheels() {
    return new LoggedTrigger("Triggers/xWheels", controller.x());
  }

  public LoggedTrigger simButton() {
    return new LoggedTrigger("Triggers/simButton", controller.b());
  }

  // Override triggers
  public LoggedTrigger allianceWinFlipper() {
    return new LoggedTrigger("Triggers/allianceWinFlipper", controller.a());
  }

  public LoggedTrigger allianceWinDisabler() {
    return new LoggedTrigger("Triggers/allianceWinDisabler", controller.y());
  }

  // Returns true if robot is able to score fuel from its present position
  public LoggedTrigger isShootSafeZone() {
    return new LoggedTrigger(
        "isShootSafeZone",
        () -> {
          HardwareConstants.Zones.broadZone currentZone =
              RobotState.getInstance().getBroadZone(RobotState.getInstance().getEstimatedPose());
          return (currentZone == HardwareConstants.Zones.broadZone.ALLIANCE_ZONE);
        });
  }

  public LoggedTrigger isShootSafeTimeSure() {
    // BUG FIX: Previously `safe` was captured outside the lambda, freezing its value at
    // construction time. Now evaluated inside the lambda so it updates every loop.
    return new LoggedTrigger(
        "isShootSafeTimeSure", () -> HubShiftUtil.getShiftedShiftInfo().active());
  }

  // Returns true if robot is able to score fuel at the current match time
  public LoggedTrigger isShootSafeTime() {
    // BUG FIX: Previously `disabled` and `safe` were captured outside the lambda, freezing their
    // values at construction time. Now evaluated inside the lambda so they update every loop.
    return new LoggedTrigger(
        "isShootSafeTime",
        () -> (HubShiftUtil.getShiftedShiftInfo().active() || HubShiftUtil.disabled));
  }

  // Composite checker for time and zone
  // BUG FIX: Previously called isShootSafeTime() and isShootSafeZone() which each created NEW
  // LoggedTrigger instances on every call. Now uses a single lambda that evaluates both conditions.
  public LoggedTrigger isShootClear() {
    return new LoggedTrigger(
        "isShootClear",
        () -> {
          boolean timeSafe = HubShiftUtil.getShiftedShiftInfo().active() || HubShiftUtil.disabled;
          HardwareConstants.Zones.broadZone currentZone =
              RobotState.getInstance().getBroadZone(RobotState.getInstance().getEstimatedPose());
          boolean zoneSafe = (currentZone == HardwareConstants.Zones.broadZone.ALLIANCE_ZONE);
          return timeSafe && zoneSafe;
        });
  }

  // Needs review and practice
  // Position, heading, and velocity checker that returns true if intake is safe (not in danger of
  // crashing into hub)
  // BUG FIX: Previously had inverted logic (returned true when UNSAFE) and called
  // facingAllianceHub(null)/facingOpposingHub(null) which would cause a NullPointerException.
  // Now uses current estimated pose and negates the unsafe condition to return true when safe.
  public LoggedTrigger isIntakeSafe() {
    return new LoggedTrigger(
        "isIntakeSafe",
        () -> {
          Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
          boolean dangerFromAlliance =
              tooCloseToAllianceHub().getAsBoolean()
                  && facingAllianceHub(currentPose).getAsBoolean()
                  && movingTowardAllianceHub().getAsBoolean();
          boolean dangerFromOpposing =
              tooCloseToOpposingHub().getAsBoolean()
                  && facingOpposingHub(currentPose).getAsBoolean()
                  && movingTowardOpposingHub().getAsBoolean();
          // Return true when safe (NOT in danger)
          return !(dangerFromAlliance || dangerFromOpposing);
        });
  }

  // Needs review
  // Returns false if robot's estimated pose is in a trench zone or is moving towards it
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

  // Alignment triggers - intended for use in preventing hub crashes
  public LoggedTrigger tooCloseToAllianceHub() {
    return new LoggedTrigger(
        "tooCloseToAllianceHub",
        () ->
            (RobotState.getInstance().getDistanceToAllianceHub().magnitude()
                < HardwareConstants.hubDangerZone.intakeOffset));
  }

  public LoggedTrigger tooCloseToOpposingHub() {
    return new LoggedTrigger(
        "tooCloseToOpposingHub",
        () ->
            (RobotState.getInstance().getDistanceToOpposingHub().magnitude()
                < HardwareConstants.hubDangerZone.intakeOffset));
  }

  public LoggedTrigger facingAllianceHub(Pose2d pose) {
    return new LoggedTrigger(
        "facingAllianceHub",
        () -> {
          // Use provided pose, or fall back to current estimated pose if null
          Pose2d effectivePose =
              (pose != null) ? pose : RobotState.getInstance().getEstimatedPose();
          double heading = AllianceFlipUtil.apply(effectivePose.getRotation()).getDegrees();
          HardwareConstants.Zones.specificZone currentSpecificZone =
              RobotState.getInstance().getSpecificZone(effectivePose);
          HardwareConstants.Zones.broadZone currentBroadZone =
              RobotState.getInstance().getBroadZone(effectivePose);
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
  }

  public LoggedTrigger facingOpposingHub(Pose2d pose) {
    return new LoggedTrigger(
        "facingOpposingHub",
        () -> {
          // Use provided pose, or fall back to current estimated pose if null
          Pose2d effectivePose =
              (pose != null) ? pose : RobotState.getInstance().getEstimatedPose();
          double heading = AllianceFlipUtil.apply(effectivePose.getRotation()).getDegrees();
          HardwareConstants.Zones.specificZone currentSpecificZone =
              RobotState.getInstance().getSpecificZone(effectivePose);
          HardwareConstants.Zones.broadZone currentBroadZone =
              RobotState.getInstance().getBroadZone(effectivePose);
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
  }

  public LoggedTrigger movingTowardAllianceHub() {
    return new LoggedTrigger(
        "movingTowardAllianceHub",
        () -> {
          HardwareConstants.Zones.specificZone currentSpecificZone =
              RobotState.getInstance().getSpecificZone(RobotState.getInstance().getEstimatedPose());
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
  }

  public LoggedTrigger movingTowardOpposingHub() {
    return new LoggedTrigger(
        "movingTowardOpposingHub",
        () -> {
          HardwareConstants.Zones.specificZone currentSpecificZone =
              RobotState.getInstance().getSpecificZone(RobotState.getInstance().getEstimatedPose());
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
  }

  // All zone checkers require review - not practical for high-velocity zone changes
  // In trench - position-based
  public LoggedTrigger isRobotInTrench() {
    return new LoggedTrigger(
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
  }

  // Close to trench - currently position-based, consider adding velocity-based
  public LoggedTrigger isRobotApproachingTrench() {
    return new LoggedTrigger(
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
  }

  // On bump - position checker
  public LoggedTrigger isRobotOnBump() {
    return new LoggedTrigger(
        "isRobotOnBump",
        () -> {
          HardwareConstants.Zones.specificZone currentZone =
              RobotState.getInstance().getSpecificZone(RobotState.getInstance().getEstimatedPose());
          switch (currentZone) {
            case ALLIANCE_BUMP_FAR, ALLIANCE_BUMP_NEAR, OPPOSING_BUMP_FAR, OPPOSING_BUMP_NEAR:
              return true;
            default:
              return false;
          }
        });
  }

  // Near bump - currently position-based, consider adding velocity-based
  public LoggedTrigger isRobotApproachingBump() {
    return new LoggedTrigger(
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
  }

  // In tower - position-based
  public LoggedTrigger isRobotInTower() {
    return new LoggedTrigger(
        "isRobotInTower",
        () -> {
          HardwareConstants.Zones.specificZone currentZone =
              RobotState.getInstance().getSpecificZone(RobotState.getInstance().getEstimatedPose());
          switch (currentZone) {
            case ALLIANCE_TOWER, OPPOSING_TOWER:
              return true;
            default:
              return false;
          }
        });
  }

  // Near tower - currently position-based, consider adding velocity-based
  public LoggedTrigger isRobotApproachingTower() {
    return new LoggedTrigger(
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
  }

  // At or approaching wall - position checker, split into at and approaching, consider adding
  // velocity-based
  public LoggedTrigger isRobotAtWall() {
    return new LoggedTrigger(
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
}
