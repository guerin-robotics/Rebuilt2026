package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

  public boolean alignmentOverride(CommandXboxController controller) {
    return controller.b().getAsBoolean();
  }

  public boolean driveLucasProofOverride(CommandXboxController controller) {
    return controller.x().getAsBoolean();
  }

  public boolean isShootSafeZone() {
    HardwareConstants.Zones.Zone currentZone =
        RobotState.getInstance().getRobotZone(RobotState.getInstance().getEstimatedPose());
    boolean safe =
        (currentZone == HardwareConstants.Zones.Zone.ALLIANCE_ZONE
            || currentZone == Zone.ALLIANCE_ZONE_TRENCH_BORDER);
    Logger.recordOutput("RobotState/zoneSafeToShoot", safe);
    return safe;
  }

  public boolean isShootSafeTime() {
    boolean safe = (HubShiftUtil.getShiftedShiftInfo().active());
    Logger.recordOutput("RobotState/timeSafeToShoot", safe);
    return safe;
  }

  // Composite checker for time and zone
  public boolean isShootClear() {
    boolean clear = (isShootSafeTime() && isShootSafeZone());
    Logger.recordOutput("RobotState/clearToShoot", clear);
    return clear;
  }

  // Needs review and practice
  // Position, heading, and velocity checker that returns false if intake is in danger of crashing
  // into hub
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

  // Needs review
  // Returns false if robot's estimated pose is in a trench zone or is moving towards it
  public boolean isHoodSafe(Pose2d pose) {
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
  }

  // All zone checkers require review - not practical for high-velocity zone changes
  // In trench - position-based
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

  // Close to trench - currently position-based, consider adding velocity-based
  public boolean isRobotApproachingTrench() {
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
  }

  // On bump - position checker
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

  // Near bump - currently position-based, consider adding velocity-based
  public boolean isRobotApproachingBump() {
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
  }

  // In tower - position-based
  public boolean isRobotInTower() {
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
  }

  // Near tower - currently position-based, consider adding velocity-based
  public boolean isRobotApproachingTower() {
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
  }

  // At or approaching wall - position checker, split into at and approaching, consider adding
  // velocity-based
  public boolean isRobotAtWall() {
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
  }
}
