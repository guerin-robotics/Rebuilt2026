package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.AllianceFlipUtil;
import frc.lib.FieldConstants;
import frc.robot.HardwareConstants.Zones.Zone;
import frc.robot.util.HubShiftUtil;
import org.littletonrobotics.junction.Logger;

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

  public Trigger simTrenchButton() {
    return controller.b();
  }

  // Override triggers
  public Trigger allianceWinFlipper() {
    return controller.a();
  }

  public Trigger allianceWinDisabler() {
    return controller.y();
  }

  // Returns true if robot is able to score fuel from its present position
  public Trigger isShootSafeZone() {
    Trigger safe =
        new Trigger(
            () ->
            {
              HardwareConstants.Zones.Zone currentZone =
                RobotState.getInstance().getRobotZone(RobotState.getInstance().getEstimatedPose());
              return (currentZone == HardwareConstants.Zones.Zone.ALLIANCE_ZONE
                    || currentZone == Zone.ALLIANCE_ZONE_TRENCH_BORDER);
                }); 
    Logger.recordOutput("RobotState/zoneSafeToShoot", safe.getAsBoolean());
    return safe;
  }

  // Returns true if robot is able to score fuel at the current match time
  public Trigger isShootSafeTime() {
    boolean disabled = HubShiftUtil.disabled;
    boolean safe = HubShiftUtil.getShiftedShiftInfo().active();
    Logger.recordOutput("RobotState/timeSafeToShoot", safe);
    return new Trigger(() -> (safe || disabled));
  }

  // Composite checker for time and zone
  public Trigger isShootClear() {
    Trigger clear =
        new Trigger(() -> (isShootSafeTime().getAsBoolean() && isShootSafeZone().getAsBoolean()));
    Logger.recordOutput("RobotState/clearToShoot", clear.getAsBoolean());
    return clear;
  }

  // Needs review and practice
  // Position, heading, and velocity checker that returns false if intake is in danger of crashing
  // into hub
  public Trigger isIntakeSafe() {
    if ((tooCloseToAllianceHub().getAsBoolean()
            && facingAllianceHub(RobotState.getInstance().getEstimatedPose()).getAsBoolean()
            && movingTowardAllianceHub().getAsBoolean())
        || (tooCloseToOpposingHub().getAsBoolean()
            && facingOpposingHub(RobotState.getInstance().getEstimatedPose()).getAsBoolean()
            && movingTowardOpposingHub().getAsBoolean())) {
      return new Trigger(() -> false);
    } else {
      return new Trigger(() -> true);
    }
  }

  // Needs review
  // Returns false if robot's estimated pose is in a trench zone or is moving towards it
  public Trigger isHoodSafe(Pose2d pose) {
    Trigger safe = new Trigger(() -> {
      HardwareConstants.Zones.Zone currentZone = RobotState.getInstance().getRobotZone(pose);
      double vxMetersPerSecond =
          RobotState.getInstance().getFieldRelativeVelocity().vxMetersPerSecond;
      boolean unsafe =
          (currentZone == HardwareConstants.Zones.Zone.ALLIANCE_TRENCH_NEAR)
              || (currentZone == HardwareConstants.Zones.Zone.ALLIANCE_TRENCH_FAR)
              || (currentZone == HardwareConstants.Zones.Zone.OPPOSING_TRENCH_NEAR)
              || (currentZone == HardwareConstants.Zones.Zone.OPPOSING_TRENCH_FAR)
              || (currentZone == HardwareConstants.Zones.Zone.ALLIANCE_ZONE_TRENCH_BORDER
                  && vxMetersPerSecond > 0)
              || (currentZone == Zone.ALLIANCE_NEUTRAL_TRENCH_BORDER && vxMetersPerSecond < 0)
              || (currentZone == HardwareConstants.Zones.Zone.OPPOSING_ZONE_TRENCH_BORDER
                  && vxMetersPerSecond < 0)
              || (currentZone == Zone.OPPOSING_NEUTRAL_TRENCH_BORDER && vxMetersPerSecond > 0);
        Logger.recordOutput("RobotState/isHoodSafe", !unsafe);
        return !unsafe;
    });
    return safe;
  }

  // Alignment triggers - intended for use in preventing hub crashes
  public Trigger tooCloseToAllianceHub() {
    return new Trigger(
        () ->
            (RobotState.getInstance().getDistanceToAllianceHub().magnitude()
                < HardwareConstants.hubDangerZone.intakeOffset));
  }

  public Trigger tooCloseToOpposingHub() {
    return new Trigger(
        () ->
            (RobotState.getInstance().getDistanceToOpposingHub().magnitude()
                < HardwareConstants.hubDangerZone.intakeOffset));
  }

  public Trigger facingAllianceHub(Pose2d pose) {
    return new Trigger(() -> {
      double heading = AllianceFlipUtil.apply(pose.getRotation()).getDegrees();
      HardwareConstants.Zones.Zone currentZone = RobotState.getInstance().getRobotZone(pose);
      if ((currentZone == Zone.ALLIANCE_ZONE || currentZone == Zone.ALLIANCE_ZONE_TRENCH_BORDER)
          && (Math.abs(heading) < 90)) {
        return true;
      } else if ((currentZone == Zone.NEUTRAL || currentZone == Zone.ALLIANCE_NEUTRAL_TRENCH_BORDER)
          && (Math.abs(heading) >= 90)) {
        return true;
      } else if (currentZone == Zone.ALLIANCE_BUMP_FAR && (heading < 0)) {
        return true;
      } else if (currentZone == Zone.ALLIANCE_BUMP_NEAR && (heading > 0)) {
        return true;
      } else {
        return false;
      }
    });
  }

  public Trigger facingOpposingHub(Pose2d pose) {
      return new Trigger(() -> {
      double heading = AllianceFlipUtil.apply(pose.getRotation()).getDegrees();
      HardwareConstants.Zones.Zone currentZone = RobotState.getInstance().getRobotZone(pose);
      if ((currentZone == HardwareConstants.Zones.Zone.NEUTRAL
              || currentZone == HardwareConstants.Zones.Zone.OPPOSING_NEUTRAL_TRENCH_BORDER)
          && (Math.abs(heading) < 90)) {
        return true;
      } else if ((currentZone == HardwareConstants.Zones.Zone.OPPOSING_ZONE
              || currentZone == Zone.OPPOSING_ZONE_TRENCH_BORDER)
          && (Math.abs(heading) >= 90)) {
        return true;
      } else if (currentZone == HardwareConstants.Zones.Zone.OPPOSING_BUMP_FAR && (heading < 0)) {
        return true;
      } else if (currentZone == HardwareConstants.Zones.Zone.OPPOSING_BUMP_NEAR && (heading > 0)) {
        return true;
      } else {
        return false;
      }
    });
  }

  public Trigger movingTowardAllianceHub() {
    return new Trigger(() -> {
      HardwareConstants.Zones.Zone currentZone =
          RobotState.getInstance().getRobotZone(RobotState.getInstance().getEstimatedPose());
      double currentXVelocity = RobotState.getInstance().getFieldRelativeVelocity().vxMetersPerSecond;
      double currentYVelocity = RobotState.getInstance().getFieldRelativeVelocity().vyMetersPerSecond;
      if ((currentZone == HardwareConstants.Zones.Zone.ALLIANCE_ZONE
              || currentZone == HardwareConstants.Zones.Zone.ALLIANCE_ZONE_TRENCH_BORDER)
          && AllianceFlipUtil.applyX(currentXVelocity) >= AllianceFlipUtil.applyX(0)) {
        return true;
      } else if ((currentZone == HardwareConstants.Zones.Zone.NEUTRAL
              || currentZone == HardwareConstants.Zones.Zone.ALLIANCE_NEUTRAL_TRENCH_BORDER)
          && (currentXVelocity) <= AllianceFlipUtil.applyX(0)) {
        return true;
      } else if (currentZone == HardwareConstants.Zones.Zone.ALLIANCE_BUMP_NEAR
          && AllianceFlipUtil.applyY(currentYVelocity) >= AllianceFlipUtil.applyY(0)) {
        return true;
      } else if (currentZone == HardwareConstants.Zones.Zone.ALLIANCE_BUMP_FAR
          && (currentYVelocity) <= AllianceFlipUtil.applyY(0)) {
        return true;
      } else {
        return false;
      }
    });
  }

  public Trigger movingTowardOpposingHub() {
    return new Trigger(() -> {
      HardwareConstants.Zones.Zone currentZone =
          RobotState.getInstance().getRobotZone(RobotState.getInstance().getEstimatedPose());
      double currentXVelocity = RobotState.getInstance().getFieldRelativeVelocity().vxMetersPerSecond;
      double currentYVelocity = RobotState.getInstance().getFieldRelativeVelocity().vyMetersPerSecond;
      if ((currentZone == HardwareConstants.Zones.Zone.OPPOSING_ZONE
              || currentZone == HardwareConstants.Zones.Zone.OPPOSING_ZONE_TRENCH_BORDER)
          && AllianceFlipUtil.applyX(currentXVelocity) <= AllianceFlipUtil.applyX(0)) {
        return true;
      } else if ((currentZone == HardwareConstants.Zones.Zone.NEUTRAL
              || currentZone == HardwareConstants.Zones.Zone.OPPOSING_NEUTRAL_TRENCH_BORDER)
          && AllianceFlipUtil.applyX(currentXVelocity) >= AllianceFlipUtil.applyX(0)) {
        return true;
      } else if (currentZone == HardwareConstants.Zones.Zone.OPPOSING_BUMP_NEAR
          && AllianceFlipUtil.applyY(currentYVelocity) >= AllianceFlipUtil.applyY(0)) {
        return true;
      } else if (currentZone == HardwareConstants.Zones.Zone.OPPOSING_BUMP_FAR
          && AllianceFlipUtil.applyY(currentYVelocity) <= AllianceFlipUtil.applyY(0)) {
        return true;
      } else {
        return false;
      }
  });
  }

  // All zone checkers require review - not practical for high-velocity zone changes
  // In trench - position-based
  public Trigger isRobotInTrench() {
    return new Trigger(() -> {
      HardwareConstants.Zones.Zone currentZone =
          RobotState.getInstance().getRobotZone(RobotState.getInstance().getEstimatedPose());
      switch (currentZone) {
        case ALLIANCE_TRENCH_FAR, ALLIANCE_TRENCH_NEAR, OPPOSING_TRENCH_FAR, OPPOSING_TRENCH_NEAR:
          return true;
        default:
          return false;
      }
  });
  }

  // Close to trench - currently position-based, consider adding velocity-based
  public Trigger isRobotApproachingTrench() {
    return new Trigger(() -> {
      HardwareConstants.Zones.Zone currentZone =
          RobotState.getInstance().getRobotZone(RobotState.getInstance().getEstimatedPose());
      double currentY = RobotState.getInstance().getEstimatedPose().getY();
      boolean yOk =
          (currentY > FieldConstants.LinesHorizontal.leftBumpStart
              || currentY < FieldConstants.LinesHorizontal.rightBumpEnd);
      if (yOk) {
        switch (currentZone) {
          case ALLIANCE_ZONE_TRENCH_BORDER,
              ALLIANCE_NEUTRAL_TRENCH_BORDER,
              OPPOSING_ZONE_TRENCH_BORDER,
              OPPOSING_NEUTRAL_TRENCH_BORDER:
            return true;
          default:
            return false;
        }
      } else {
        return false;
      }
  });
  }

  // On bump - position checker
  public Trigger isRobotOnBump() {
    return new Trigger(() -> {
      HardwareConstants.Zones.Zone currentZone =
          RobotState.getInstance().getRobotZone(RobotState.getInstance().getEstimatedPose());
      switch (currentZone) {
        case ALLIANCE_BUMP_FAR, ALLIANCE_BUMP_NEAR, OPPOSING_BUMP_FAR, OPPOSING_BUMP_NEAR:
          return true;
        default:
          return false;
      }
    });
  }

  // Near bump - currently position-based, consider adding velocity-based
  public Trigger isRobotApproachingBump() {
    return new Trigger(() -> {
      HardwareConstants.Zones.Zone currentZone =
          RobotState.getInstance().getRobotZone(RobotState.getInstance().getEstimatedPose());
      double currentY = RobotState.getInstance().getEstimatedPose().getY();
      boolean yOk =
          (currentY < FieldConstants.LinesHorizontal.leftBumpStart
              && currentY > FieldConstants.LinesHorizontal.rightBumpEnd);
      if (yOk) {
        switch (currentZone) {
          case ALLIANCE_ZONE_TRENCH_BORDER,
              ALLIANCE_NEUTRAL_TRENCH_BORDER,
              OPPOSING_ZONE_TRENCH_BORDER,
              OPPOSING_NEUTRAL_TRENCH_BORDER:
            return true;
          default:
            return false;
        }
      } else {
        return false;
      }
  });
  }

  // In tower - position-based
  public Trigger isRobotInTower() {
    return new Trigger(() -> {
      double currentX = RobotState.getInstance().getEstimatedPose().getX();
      double currentY = RobotState.getInstance().getEstimatedPose().getY();
      if ((currentX < FieldConstants.Tower.leftUpright.getX()
              && currentY > FieldConstants.Tower.rightUpright.getY()
              && currentY < FieldConstants.Tower.leftUpright.getY())
          || (currentX > FieldConstants.Tower.oppLeftUpright.getX()
              && currentY > FieldConstants.Tower.oppRightUpright.getY()
              && currentY < FieldConstants.Tower.oppLeftUpright.getY())) {
        return true;
      } else {
        return false;
      }
    });
  }

  // Near tower - currently position-based, consider adding velocity-based
  public Trigger isRobotApproachingTower() {
    return new Trigger(() -> {
      double currentX = RobotState.getInstance().getEstimatedPose().getX();
      double currentY = RobotState.getInstance().getEstimatedPose().getY();
      if ((currentX < (FieldConstants.Tower.leftUpright.getX())
              && currentY
                  > (FieldConstants.Tower.rightUpright.getY() - HardwareConstants.Zones.zoneOffset)
              && currentY
                  < (FieldConstants.Tower.leftUpright.getY() + HardwareConstants.Zones.zoneOffset))
          || (currentX > (FieldConstants.Tower.oppLeftUpright.getX())
              && currentY
                  > (FieldConstants.Tower.oppRightUpright.getY() - HardwareConstants.Zones.zoneOffset)
              && currentY
                  < (FieldConstants.Tower.oppLeftUpright.getY()
                      + HardwareConstants.Zones.zoneOffset))) {
        return true;
      } else {
        return false;
      }
  });
  }

  // At or approaching wall - position checker, split into at and approaching, consider adding
  // velocity-based
  public Trigger isRobotAtWall() {
    return new Trigger(() -> {
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