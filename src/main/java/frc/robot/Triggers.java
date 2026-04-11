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
    return controller.b();
  }

  public Trigger simTrenchAlignButton() {
    return simController.button(2);
  }

  public Trigger simIntakeInButton() {
    return simController.button(3);
  }

  public Trigger simIntakeOutButton() {
    return simController.button(4);
  }

  public Trigger simIntakeRollerButton() {
    return simController.button(5);
  }

  public Trigger simIntakeCompressButton() {
    return simController.button(6);
  }

  public Trigger simBumpAlignButton() {
    return simController.button(7);
  }

  public Trigger simShootFromTowerButton() {
    return simController.button(8);
  }

  public Trigger simPassButton() {
    return simController.button(9);
  }

  public Trigger simXWheels() {
    return simController.button(0);
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

  // Returns true if robot is able to score fuel at the current match time, or if timer is disabled
  // NOTE: The boolean values must be read INSIDE the lambda so they are evaluated every loop.
  // Capturing them outside the lambda freezes the value at construction time.
  public LoggedTrigger isShootSafeTime() {
    return new LoggedTrigger(
        "isShootSafeTime",
        () -> (HubShiftUtil.getShiftedShiftInfo().active() || HubShiftUtil.disabled));
  }

  // Adaptation of above trigger that returns true if time is safe, but not if timer is disabled
  // (Used for smart idle speeds)
  public LoggedTrigger isShootSafeTimeSure() {
    return new LoggedTrigger(
        "isShootSafeTimeSure", () -> HubShiftUtil.getShiftedShiftInfo().active());
  }

  // Composite checker for time and zone
  public LoggedTrigger isShootClear() {
    return new LoggedTrigger("isShootClear", isShootSafeTime().and(isShootSafeZone()));
  }

  // Needs review and practice
  // Position, heading, and velocity checker that returns false if intake is in danger of crashing
  // into hub
  public LoggedTrigger isIntakeSafe() {
    return new LoggedTrigger(
        "isIntakeSafe",
        (tooCloseToAllianceHub().and(facingAllianceHub(null)).and(movingTowardAllianceHub()))
            .or(
                tooCloseToOpposingHub()
                    .and(facingOpposingHub(null))
                    .and(movingTowardOpposingHub())));
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
  }

  public LoggedTrigger facingOpposingHub(Pose2d pose) {
    return new LoggedTrigger(
        "facingOpposingHub",
        () -> {
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
