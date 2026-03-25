package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.FieldConstants;
import frc.robot.HardwareConstants.Zones.Zone;
import org.littletonrobotics.junction.Logger;

public class Triggers {

  /* These are all booleans that we'll use later to determine when commands should be called - mostly for safe
  driving. They use pose, velocity, and zone estimation from RobotState. */

  private static Triggers instance;

  public static Triggers getInstance() {
    if (instance == null) {
      instance = new Triggers();
    }
    return instance;
  }

  public boolean isShootSafe() {
    HardwareConstants.Zones.Zone futureZone =
        RobotState.getInstance().getRobotZone(RobotState.getInstance().getEstimatedPose());
    boolean safe = (futureZone == HardwareConstants.Zones.Zone.ALLIANCE_ZONE);
    Logger.recordOutput("RobotState/zoneSafeToShoot", safe);
    return safe;
  }

  public boolean isIntakeSafe() {
    if ((RobotState.getInstance().tooCloseToAllianceHub()
            && RobotState.getInstance()
                .facingAllianceHub(RobotState.getInstance().getEstimatedPose())
            && RobotState.getInstance().movingTowardAllianceHub())
        || (RobotState.getInstance().tooCloseToOpposingHub()
            && RobotState.getInstance()
                .facingOpposingHub(RobotState.getInstance().getEstimatedPose())
            && RobotState.getInstance().movingTowardOpposingHub())) {
      return false;
    } else {
      return true;
    }
  }

  // Returns false if robot's estimated pose is in a trench zone or is moving towards it
  public boolean ishoodsafe(Pose2d pose) {
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

    Logger.recordOutput("RobotState/ishoodsafe", !unsafe);
    return !unsafe;
  }

  public boolean isRobotInTrench() {
    HardwareConstants.Zones.Zone currentZone =
        RobotState.getInstance().getRobotZone(RobotState.getInstance().getEstimatedPose());
    switch (currentZone) {
      case ALLIANCE_TRENCH_FAR, ALLIANCE_TRENCH_NEAR, OPPOSING_TRENCH_FAR, OPPOSING_TRENCH_NEAR:
        return true;
      default:
        return false;
    }
  }

  public boolean isRobotApproachingTrench() {
    HardwareConstants.Zones.Zone currentZone =
        RobotState.getInstance().getRobotZone(RobotState.getInstance().getEstimatedPose());
    double currentY = RobotState.getInstance().getEstimatedPose().getY();
    ChassisSpeeds velocity = RobotState.getInstance().getFieldRelativeVelocity();
    if (currentZone == HardwareConstants.Zones.Zone.ALLIANCE_ZONE
        && (currentY > FieldConstants.LinesHorizontal.leftBumpEnd
            || currentY < FieldConstants.LinesHorizontal.rightBumpEnd)
        && velocity.vxMetersPerSecond > 0.0) {
      return true;
    } else if (currentZone == HardwareConstants.Zones.Zone.NEUTRAL
        && (currentY > FieldConstants.LinesHorizontal.leftBumpEnd
            || currentY < FieldConstants.LinesHorizontal.rightBumpEnd)
        && velocity.vxMetersPerSecond < 0.0) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isRobotOnBump() {
    HardwareConstants.Zones.Zone currentZone =
        RobotState.getInstance().getRobotZone(RobotState.getInstance().getEstimatedPose());
    switch (currentZone) {
      case ALLIANCE_BUMP_FAR, ALLIANCE_BUMP_NEAR, OPPOSING_BUMP_FAR, OPPOSING_BUMP_NEAR:
        return true;
      default:
        return false;
    }
  }

  public boolean isRobotApproachingBump() {
    HardwareConstants.Zones.Zone currentZone =
        RobotState.getInstance().getRobotZone(RobotState.getInstance().getEstimatedPose());
    double currentY = RobotState.getInstance().getEstimatedPose().getY();
    ChassisSpeeds velocity = RobotState.getInstance().getFieldRelativeVelocity();
    if (currentZone == HardwareConstants.Zones.Zone.ALLIANCE_ZONE
        && currentY < FieldConstants.LinesHorizontal.leftBumpEnd
        && currentY > FieldConstants.LinesHorizontal.rightBumpEnd
        && velocity.vxMetersPerSecond > 0.0) {
      return true;
    } else if (currentZone == HardwareConstants.Zones.Zone.NEUTRAL
        && currentY < FieldConstants.LinesHorizontal.leftBumpEnd
        && currentY > FieldConstants.LinesHorizontal.rightBumpEnd
        && velocity.vxMetersPerSecond < 0.0) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isRobotInTower() {
    HardwareConstants.Zones.Zone currentZone =
        RobotState.getInstance().getRobotZone(RobotState.getInstance().getEstimatedPose());
    double currentX = RobotState.getInstance().getEstimatedPose().getX();
    double currentY = RobotState.getInstance().getEstimatedPose().getY();
    if (currentZone == Zone.ALLIANCE_ZONE
        && currentX < FieldConstants.Tower.frontFaceX
        && currentY > FieldConstants.Tower.rightUpright.getY()
        && currentY < FieldConstants.Tower.leftUpright.getY()) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isRobotApproachingTower() {
    HardwareConstants.Zones.Zone currentZone =
        RobotState.getInstance().getRobotZone(RobotState.getInstance().getEstimatedPose());
    double currentX = RobotState.getInstance().getEstimatedPose().getX();
    double currentY = RobotState.getInstance().getEstimatedPose().getY();
    if (currentZone == Zone.ALLIANCE_ZONE
        && currentX < FieldConstants.Tower.frontFaceX
        && currentY
            > (FieldConstants.Tower.rightUpright.getY() - HardwareConstants.Zones.zoneOffset)
        && currentY
            > (FieldConstants.Tower.leftUpright.getY() - HardwareConstants.Zones.zoneOffset)) {
      return true;
    } else {
      return false;
    }
  }
}
