package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

/**
 * Calculates the required flywheel speed to hit a target on the field.
 *
 * <p>This is a singleton class that handles all shot calculations for the shooter. It uses the
 * centralized RobotState to get the robot's current position and calculates the distance to the
 * target.
 *
 * <p><b>How Distance-Based Shooting Works:</b>
 *
 * <ol>
 *   <li>Get robot's current position from RobotState
 *   <li>Calculate 2D distance to the target (horizontal distance)
 *   <li>Look up required flywheel RPM from the interpolation table
 *   <li>The table automatically interpolates between characterization data points
 * </ol>
 *
 * <p><b>Characterization:</b> To use this system effectively, you need to characterize your shooter
 * by:
 *
 * <ol>
 *   <li>Placing the robot at known distances from the target
 *   <li>Adjusting RPM until shots consistently score
 *   <li>Recording the (distance, RPM) pairs in ShooterConstants.DistanceMap.SPEED_MAP
 * </ol>
 *
 * <p><b>Future Enhancements:</b> This class is designed to be extended for shooting-while-moving.
 * The structure allows for:
 *
 * <ul>
 *   <li>Robot velocity compensation
 *   <li>Lead time calculations
 * </ul>
 *
 * <p><b>Usage:</b>
 *
 * <pre>
 * // Get the speed needed to hit the alliance hub
 * Translation3d hubTarget = RobotState.getInstance().getAllianceHubTarget();
 * AngularVelocity speed = ShotCalculator.getInstance().getFlywheelSpeedForTarget(hubTarget);
 *
 * // Or use the convenience method
 * AngularVelocity speed = ShotCalculator.getInstance().getFlywheelSpeedForAllianceHub();
 * </pre>
 */
public class ShotCalculator {

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

  // ==================== SHOT CALCULATION ====================

  /**
   * Calculates the required flywheel speed to hit a target.
   *
   * <p>This method:
   *
   * <ol>
   *   <li>Gets the robot's current position from RobotState
   *   <li>Calculates the 2D (horizontal) distance to the target
   *   <li>Looks up the required RPM from the interpolation table
   *   <li>Clamps the result to safe min/max speeds
   * </ol>
   *
   * <p><b>Note:</b> Currently uses 2D distance only. The target's Z coordinate (height) is used for
   * logging but doesn't affect the calculation. This could be enhanced for trajectory-based
   * shooting.
   *
   * @param target The target position to shoot at (Translation3d in field coordinates)
   * @return The required flywheel angular velocity
   */
  public AngularVelocity getFlywheelSpeedForTarget(Translation3d target) {
    Translation2d robotPosition = RobotState.getInstance().getEstimatedPose().getTranslation();

    Translation2d targetPosition2d = target.toTranslation2d();
    double distanceMeters = robotPosition.getDistance(targetPosition2d);

    Logger.recordOutput("Flywheel/ShotCalculator/TargetPosition", target);
    Logger.recordOutput("Flywheel/ShotCalculator/RobotPosition", robotPosition);

    return getFlywheelSpeedForDistance(Meters.of(distanceMeters));
  }

  /**
   * Calculates the required flywheel speed to hit a target at a specific distance.
   *
   * <p>This is a lower-level method that can be used for testing or for shooting at specific
   * distances without relying on RobotState. It looks up the required RPM from the interpolation
   * table and clamps it to safe limits.
   *
   * @param distance The horizontal distance to the target
   * @return The required flywheel angular velocity
   */
  public AngularVelocity getFlywheelSpeedForDistance(Distance distance) {
    double distanceMeters = distance.in(Meters);
    double speedRPM = FlywheelConstants.DistanceMap.SPEED_MAP.get(distanceMeters);

    double minRPM = FlywheelConstants.Limits.MIN_SPEED.in(RevolutionsPerSecond) * 60.0;
    double maxRPM = FlywheelConstants.Limits.MAX_SPEED.in(RevolutionsPerSecond) * 60.0;
    speedRPM = Math.max(minRPM, Math.min(maxRPM, speedRPM));

    Logger.recordOutput("Flywheel/ShotCalculator/Distance_m", distanceMeters);
    Logger.recordOutput("Flywheel/ShotCalculator/CalculatedSpeed_RPM", speedRPM);

    return RPM.of(speedRPM);
  }

  /**
   * Calculates the required flywheel speed to hit the alliance hub.
   *
   * <p>This is a convenience method that automatically uses the correct hub target based on
   * alliance color (handled by RobotState).
   *
   * @return The required flywheel angular velocity to hit the alliance hub
   */
  public AngularVelocity getFlywheelSpeedForAllianceHub() {
    Translation3d hubTarget = RobotState.getInstance().getAllianceHubTarget();
    return getFlywheelSpeedForTarget(hubTarget);
  }

  /**
   * Returns the 2D distance from the robot to a target.
   *
   * <p>Useful for logging and decision-making (e.g., "should we even try to shoot from here?")
   *
   * @param target The target position
   * @return The horizontal distance to the target
   */
  public Distance getDistanceToTarget(Translation3d target) {
    Translation2d robotPosition = RobotState.getInstance().getEstimatedPose().getTranslation();
    Translation2d targetPosition2d = target.toTranslation2d();
    double distanceMeters = robotPosition.getDistance(targetPosition2d);
    return Meters.of(distanceMeters);
  }

  /**
   * Checks if the robot is within effective shooting range of the alliance hub.
   *
   * <p>This can be used to provide driver feedback (LEDs, rumble) or to gate shooting commands.
   *
   * <p>The effective range is determined by the data points in the interpolation table - shooting
   * from distances outside the characterized range may be inaccurate.
   *
   * @return true if the robot is within effective shooting range
   */
  public boolean isInShootingRange() {
    Distance distance = RobotState.getInstance().getDistanceToAllianceHub();
    double distanceMeters = distance.in(Meters);

    // Check if within the characterized range
    // The DISTANCE_TO_SPEED_MAP defines the range we've tested
    boolean inRange = distanceMeters >= 0.5 && distanceMeters <= 15.0;

    Logger.recordOutput("Flywheel/ShotCalculator/InShootingRange", inRange);
    return inRange;
  }
}
