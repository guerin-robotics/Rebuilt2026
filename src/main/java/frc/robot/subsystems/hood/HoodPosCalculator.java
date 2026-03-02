package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class HoodPosCalculator {

  private static HoodPosCalculator instance;

  public static HoodPosCalculator getInstance() {
    if (instance == null) {
      instance = new HoodPosCalculator();
    }
    return instance;
  }

  private HoodPosCalculator() {}

  // Calculations are similar to those in ShotCalculator: general get angle for distance, helper for
  // when distance is to a specific target, helper for when target is the hub.

  public Double getHoodPosForDistance(Distance distance) {
    double distanceMeters = distance.in(Meters);
    double hoodPos = HoodConstants.HoodMap.ANGLE_MAP.get(distanceMeters);

    Logger.recordOutput("Flywheel/ShotCalculator/HoodPos", hoodPos);

    if (hoodPos > HoodConstants.Mechanical.hoodMaxPos) {
        hoodPos = HoodConstants.Mechanical.hoodMaxPos;
    } else if (hoodPos < HoodConstants.Mechanical.hoodMinPos) {
        hoodPos = HoodConstants.Mechanical.hoodMinPos;
    }

    return hoodPos;
  }

  public Double getHoodPosForTarget(Translation3d target) {
    Translation2d target2d = new Translation2d(target.getX(), target.getY());
    Distance distanceMeters = RobotState.getInstance().getDistanceToPoint(target2d);

    return getHoodPosForDistance(distanceMeters);
  }
  
  public Double getHoodPosForHub() {
    Translation3d hub3d = RobotState.getInstance().getAllianceHubTarget();
    return getHoodPosForTarget(hub3d);
  }

}
