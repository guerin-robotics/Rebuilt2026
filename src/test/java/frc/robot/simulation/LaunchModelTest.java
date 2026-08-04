package frc.robot.simulation;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.flywheel.FlywheelConstants;
import frc.robot.subsystems.hood.HoodConstants;
import org.junit.jupiter.api.Test;

/**
 * Checks the simulated launch model against the robot's own shot characterization.
 *
 * <p>{@code SPEED_MAP} and {@code ANGLE_MAP} were tuned on the real robot until shots scored, so
 * every (distance, RPM, hood) triple in them is a known-good shot. If the sim's launch model is
 * right, firing those exact inputs should put the Fuel through the Hub. This test flies each one
 * and asserts it arrives at Hub height.
 *
 * <p>This is what catches the failure mode that motivated the model: feeding the hood's mechanism
 * angle in as launch elevation produces a 1°–12° shot that falls far short, and nothing else in the
 * build would notice.
 */
public class LaunchModelTest {

  /** Hub center height, from MapleSim's {@code RebuiltHub.blueHubPose}. */
  private static final double HUB_HEIGHT_METERS = 1.5748;

  private static final double LAUNCH_HEIGHT_METERS =
      SimulationConstants.LAUNCH_HEIGHT.in(edu.wpi.first.units.Units.Meters);

  /**
   * The Fuel leaves from the back of the robot, which faces the Hub when shooting, so it starts
   * this much closer to the target than the robot pose the distance maps are keyed on.
   */
  private static final double SHOOTER_OFFSET_METERS =
      SimulationConstants.SHOOTER_EXIT_DISTANCE.in(edu.wpi.first.units.Units.Meters);

  private static final double DRUM_RADIUS_METERS =
      FlywheelConstants.TrajectoryVisualization.DRUM_RADIUS_METERS;
  private static final double GRAVITY = 9.81;

  /**
   * Allowed vertical error at the Hub. The fitted linear model's worst case is 0.072 m; this leaves
   * headroom for small retunes of the maps without becoming so loose it stops catching a broken
   * model (the raw-hood-angle bug misses by well over a meter).
   */
  private static final double TOLERANCE_METERS = 0.20;

  /** Distances characterized in both maps, in inches. */
  private static final double[] DISTANCES_INCHES = {75, 85, 110, 130, 145, 160, 175, 180, 190};

  @Test
  public void everyCharacterizedShotReachesHubHeight() {
    for (double distanceInches : DISTANCES_INCHES) {
      double distanceMeters = inchesToMeters(distanceInches);
      double rpm = FlywheelConstants.DistanceMap.SPEED_MAP.get(distanceMeters);
      double hoodDegrees = HoodConstants.HoodMap.ANGLE_MAP.get(distanceMeters);

      double heightAtHub = heightAtDistance(distanceMeters, rpm, hoodDegrees);
      double miss = Math.abs(heightAtHub - HUB_HEIGHT_METERS);

      assertTrue(
          miss <= TOLERANCE_METERS,
          String.format(
              "Shot from %.0f in (%.0f RPM, hood %.2f deg) arrives at %.2f m but the Hub is at"
                  + " %.2f m — off by %.2f m. The sim launch model no longer matches the robot's"
                  + " characterization.",
              distanceInches, rpm, hoodDegrees, heightAtHub, HUB_HEIGHT_METERS, miss));
    }
  }

  @Test
  public void everyShotIsDescendingWhenItReachesTheHub() {
    // A shot still rising at the Hub clips the front of the goal structure instead of dropping
    // in. This is the failure the launch-position bug produced, so it is worth guarding directly.
    for (double distanceInches : DISTANCES_INCHES) {
      double distanceMeters = inchesToMeters(distanceInches);
      double rpm = FlywheelConstants.DistanceMap.SPEED_MAP.get(distanceMeters);
      double hoodDegrees = HoodConstants.HoodMap.ANGLE_MAP.get(distanceMeters);

      double verticalSpeed = verticalSpeedAtDistance(distanceMeters, rpm, hoodDegrees);

      assertTrue(
          verticalSpeed < 0.0,
          String.format(
              "Shot from %.0f in is still rising (%.2f m/s upward) when it reaches the Hub, so it"
                  + " would hit the front of the goal rather than fall into it.",
              distanceInches, verticalSpeed));
    }
  }

  @Test
  public void launchAngleIsPhysicallyPlausible() {
    // Guards the specific regression: hood mechanism angle used directly as elevation.
    for (double distanceInches : DISTANCES_INCHES) {
      double hoodDegrees = HoodConstants.HoodMap.ANGLE_MAP.get(inchesToMeters(distanceInches));
      double elevation = launchElevationDegrees(hoodDegrees);

      assertTrue(
          elevation > 20.0 && elevation < 75.0,
          String.format(
              "Launch elevation of %.1f deg at %.0f in is not a plausible shot angle. A value"
                  + " near the raw hood reading (%.1f deg) means the mechanism-angle-to-elevation"
                  + " conversion was lost.",
              elevation, distanceInches, hoodDegrees));
    }
  }

  private static double launchElevationDegrees(double hoodDegrees) {
    return SimulationConstants.LAUNCH_ANGLE_OFFSET_DEGREES
        + SimulationConstants.LAUNCH_ANGLE_PER_HOOD_DEGREE * hoodDegrees;
  }

  /**
   * Height of the Fuel when it reaches the Hub. No drag, matching MapleSim's projectile model.
   *
   * @param distanceMeters robot-pose-to-Hub distance, as the characterization maps are keyed
   */
  private static double heightAtDistance(double distanceMeters, double rpm, double hoodDegrees) {
    double theta = Math.toRadians(launchElevationDegrees(hoodDegrees));
    double speed = exitSpeed(rpm);
    double flightTime = flightDistance(distanceMeters) / (speed * Math.cos(theta));
    return LAUNCH_HEIGHT_METERS
        + speed * Math.sin(theta) * flightTime
        - 0.5 * GRAVITY * flightTime * flightTime;
  }

  /** Vertical speed at the Hub. Negative means descending into the goal. */
  private static double verticalSpeedAtDistance(
      double distanceMeters, double rpm, double hoodDegrees) {
    double theta = Math.toRadians(launchElevationDegrees(hoodDegrees));
    double speed = exitSpeed(rpm);
    double flightTime = flightDistance(distanceMeters) / (speed * Math.cos(theta));
    return speed * Math.sin(theta) - GRAVITY * flightTime;
  }

  /** The Fuel starts at the shooter, not the robot center, and the shooter faces the Hub. */
  private static double flightDistance(double distanceMeters) {
    return distanceMeters - SHOOTER_OFFSET_METERS;
  }

  private static double exitSpeed(double rpm) {
    return rpm
        * 2.0
        * Math.PI
        / 60.0
        * DRUM_RADIUS_METERS
        * SimulationConstants.LAUNCH_VELOCITY_FACTOR;
  }
}
