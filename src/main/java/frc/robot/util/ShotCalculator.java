package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.HardwareConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.flywheel.FlywheelConstants;
import frc.robot.subsystems.hood.HoodConstants;

/**
 * Calculates the flywheel speed and hood position needed to hit a target on the field.
 *
 * <p>This is a singleton class that replaces the old flywheel ShotCalculator and HoodPosCalculator.
 * Both values are looked up from the same distance, so the flywheel and hood can never disagree
 * about where the robot is.
 *
 * <p><b>How Distance-Based Shooting Works:</b>
 *
 * <ol>
 *   <li>Get robot's current position from RobotState
 *   <li>Calculate 2D distance to the target (horizontal distance)
 *   <li>Look up flywheel RPM and hood degrees from the interpolation tables in
 *       HardwareConstants.ShotMaps
 *   <li>Clamp both values to their safe min/max limits
 * </ol>
 *
 * <p><b>Usage:</b>
 *
 * <pre>
 * ShotSolution shot = ShotCalculator.getInstance().getShotForHub();
 * flywheel uses shot.flywheelSpeed(), hood uses shot.hoodPos()
 * </pre>
 */
public class ShotCalculator {

  /**
   * The result of a shot calculation.
   *
   * @param distance Horizontal distance to the target
   * @param flywheelSpeed Required flywheel angular velocity
   * @param hoodPos Required hood position
   */
  public record ShotSolution(Distance distance, AngularVelocity flywheelSpeed, Angle hoodPos) {}

  private static ShotCalculator instance;

  /**
   * Returns the singleton instance of ShotCalculator.
   *
   * @return The ShotCalculator singleton
   */
  public static ShotCalculator getInstance() {
    if (instance == null) {
      instance = new ShotCalculator();
    }
    return instance;
  }

  /** Private constructor - use getInstance() instead. */
  private ShotCalculator() {}

  // ==================== HUB SHOTS ====================

  /**
   * Calculates the hub shot for a specific distance.
   *
   * <p>This is the lower-level method that can be used for testing or for shooting at specific
   * distances without relying on RobotState.
   *
   * @param distance The horizontal distance to the target
   * @return The flywheel speed and hood position for the shot
   */
  public ShotSolution getShotForDistance(Distance distance) {
    double distanceMeters = distance.in(Meters);

    AngularVelocity flywheelSpeed =
        RPM.of(HardwareConstants.ShotMaps.HUB_SPEED_MAP.get(distanceMeters));
    Angle hoodPos = Degrees.of(HardwareConstants.ShotMaps.HUB_HOOD_MAP.get(distanceMeters));

    return new ShotSolution(distance, clampFlywheelSpeed(flywheelSpeed), clampHoodPos(hoodPos));
  }

  /**
   * Calculates the hub shot for a target position.
   *
   * <p><b>Note:</b> Uses 2D distance only. The target's Z coordinate (height) is ignored.
   *
   * @param target The target position to shoot at (Translation3d in field coordinates)
   * @return The flywheel speed and hood position for the shot
   */
  public ShotSolution getShotForTarget(Translation3d target) {
    Translation2d target2d = target.toTranslation2d();
    Distance distance = RobotState.getInstance().getDistanceToPoint(target2d);

    return getShotForDistance(distance);
  }

  /**
   * Calculates the shot for the alliance hub.
   *
   * <p>This is a convenience method that automatically uses the correct hub target based on
   * alliance color (handled by RobotState).
   *
   * @return The flywheel speed and hood position to hit the alliance hub
   */
  public ShotSolution getShotForHub() {
    Translation3d hubTarget = RobotState.getInstance().getAllianceHubTarget();
    return getShotForTarget(hubTarget);
  }

  // ==================== PASS SHOTS ====================

  /**
   * Calculates the pass shot for a specific distance, using the passing tables.
   *
   * @param distance The horizontal distance to the pass target
   * @return The flywheel speed and hood position for the pass
   */
  public ShotSolution getPassShotForDistance(Distance distance) {
    double distanceMeters = distance.in(Meters);

    AngularVelocity flywheelSpeed =
        RPM.of(HardwareConstants.ShotMaps.PASS_SPEED_MAP.get(distanceMeters));
    Angle hoodPos = Degrees.of(HardwareConstants.ShotMaps.PASS_HOOD_MAP.get(distanceMeters));

    return new ShotSolution(distance, clampFlywheelSpeed(flywheelSpeed), clampHoodPos(hoodPos));
  }

  /**
   * Calculates the pass shot for the current pass target (handled by RobotState).
   *
   * @return The flywheel speed and hood position for the pass
   */
  public ShotSolution getShotForPass() {
    Translation2d passTarget2d = RobotState.getInstance().getPassTarget().toTranslation2d();
    Distance distance = RobotState.getInstance().getDistanceToPoint(passTarget2d);

    return getPassShotForDistance(distance);
  }

  // ==================== CLAMPING ====================

  private AngularVelocity clampFlywheelSpeed(AngularVelocity speed) {
    if (speed.gt(FlywheelConstants.Limits.MAX_SPEED)) {
      speed = FlywheelConstants.Limits.MAX_SPEED;
    } else if (speed.lt(FlywheelConstants.Limits.MIN_SPEED)) {
      speed = FlywheelConstants.Limits.MIN_SPEED;
    }

    return speed;
  }

  private Angle clampHoodPos(Angle hoodPos) {
    if (hoodPos.gt(HoodConstants.Mechanical.hoodMaxPos)) {
      hoodPos = HoodConstants.Mechanical.hoodMaxPos;
    } else if (hoodPos.lt(HoodConstants.Mechanical.hoodMinPos)) {
      hoodPos = HoodConstants.Mechanical.hoodMinPos;
    }

    return hoodPos;
  }
}
