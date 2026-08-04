package frc.robot.simulation;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.flywheel.FlywheelConstants;
import frc.robot.subsystems.hood.HoodConstants;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltHub;
import org.junit.jupiter.api.Test;

/**
 * Checks the simulated launch model against the robot's own shot characterization.
 *
 * <p>{@code SPEED_MAP} and {@code ANGLE_MAP} were tuned on the real robot until shots scored, so
 * every (distance, RPM, hood) triple in them is a known-good shot. If the sim's launch model is
 * right, firing those exact inputs should drop Fuel into the Hub.
 *
 * <p><b>What "scores" means here.</b> Accuracy is judged by where the Fuel is when it <i>descends
 * through</i> Hub height, and whether that point is within the goal radius of the Hub center. An
 * earlier version asserted height at a fixed horizontal distance, which passed happily for a
 * degenerate flat trajectory that skimmed into the side of the goal rather than falling into it.
 */
public class LaunchModelTest {

  /** Hub center height, from MapleSim's {@code RebuiltHub.blueHubPose}. */
  private static final double HUB_HEIGHT_METERS = 1.5748;

  private static final double LAUNCH_HEIGHT_METERS = SimulationConstants.LAUNCH_HEIGHT.in(Meters);

  /**
   * The Fuel leaves from the back of the robot, which faces the Hub when shooting, so it starts
   * this much closer to the target than the robot pose the distance maps are keyed on.
   */
  private static final double SHOOTER_OFFSET_METERS =
      SimulationConstants.SHOOTER_EXIT_DISTANCE.in(Meters);

  private static final double DRUM_RADIUS_METERS =
      FlywheelConstants.TrajectoryVisualization.DRUM_RADIUS_METERS;

  /**
   * MapleSim's gravity, which is 11.0 m/s² rather than Earth's 9.81 — the library inflates it as a
   * rough stand-in for the air drag it does not simulate.
   *
   * <p>Read from the library rather than hardcoded, so that if a maple-sim update changes it, this
   * test recompiles against the new value and fails instead of silently validating a launch model
   * that no longer scores.
   */
  private static final double GRAVITY = GamePieceProjectile.GRAVITY;

  /** Radius of the Hub's scoring volume, from MapleSim. */
  private static final double GOAL_RADIUS_METERS = RebuiltHub.GoalRadius;

  /**
   * Fraction of the goal radius a shot may be off by. The fitted model's worst case is 0.222 m,
   * about 37% of the radius; 70% leaves room for retuning the maps while still failing long before
   * shots actually stop scoring.
   */
  private static final double ALLOWED_ERROR_FRACTION = 0.70;

  /** Distances characterized in both maps, in inches. */
  private static final double[] DISTANCES_INCHES = {75, 85, 110, 130, 145, 160, 175, 180, 190};

  @Test
  public void everyCharacterizedShotLandsInsideTheGoal() {
    for (double distanceInches : DISTANCES_INCHES) {
      Shot shot = shotFrom(distanceInches);
      double allowed = GOAL_RADIUS_METERS * ALLOWED_ERROR_FRACTION;

      assertTrue(
          Math.abs(shot.horizontalErrorMeters) <= allowed,
          String.format(
              "Shot from %.0f in descends through Hub height %.2f m from the Hub center, outside"
                  + " the %.2f m allowance (goal radius %.2f m). The launch model no longer matches"
                  + " the robot's characterization.",
              distanceInches, Math.abs(shot.horizontalErrorMeters), allowed, GOAL_RADIUS_METERS));
    }
  }

  @Test
  public void everyShotArcsAboveTheHub() {
    // The failure this guards: a trajectory whose apex sits at or below Hub height cannot fall
    // into the goal, only skim its side. Fuel must get above the Hub before coming down.
    for (double distanceInches : DISTANCES_INCHES) {
      Shot shot = shotFrom(distanceInches);

      assertTrue(
          shot.apexMeters > HUB_HEIGHT_METERS + 0.10,
          String.format(
              "Shot from %.0f in peaks at %.2f m, which is not meaningfully above the Hub at"
                  + " %.2f m. It would skim into the side of the goal rather than drop in.",
              distanceInches, shot.apexMeters, HUB_HEIGHT_METERS));
    }
  }

  @Test
  public void everyShotDropsIntoTheGoalSteeply() {
    for (double distanceInches : DISTANCES_INCHES) {
      Shot shot = shotFrom(distanceInches);

      assertTrue(
          shot.entryAngleDegrees >= 30.0,
          String.format(
              "Shot from %.0f in arrives at only %.1f deg below horizontal. Anything this shallow"
                  + " is grazing the goal rather than falling into it.",
              distanceInches, shot.entryAngleDegrees));
    }
  }

  @Test
  public void launchAngleIsPhysicallyPlausible() {
    // Guards the original regression: hood mechanism angle used directly as elevation.
    for (double distanceInches : DISTANCES_INCHES) {
      double hoodDegrees = HoodConstants.HoodMap.ANGLE_MAP.get(inchesToMeters(distanceInches));
      double elevation = launchElevationDegrees(hoodDegrees);

      assertTrue(
          elevation > 20.0 && elevation < 80.0,
          String.format(
              "Launch elevation of %.1f deg at %.0f in is not a plausible shot angle. A value near"
                  + " the raw hood reading (%.1f deg) means the mechanism-angle-to-elevation"
                  + " conversion was lost.",
              elevation, distanceInches, hoodDegrees));
    }
  }

  // ── Trajectory maths (no drag, matching MapleSim's projectile model) ────────

  private record Shot(double horizontalErrorMeters, double apexMeters, double entryAngleDegrees) {}

  private static Shot shotFrom(double distanceInches) {
    double distanceMeters = inchesToMeters(distanceInches);
    double rpm = FlywheelConstants.DistanceMap.SPEED_MAP.get(distanceMeters);
    double hoodDegrees = HoodConstants.HoodMap.ANGLE_MAP.get(distanceMeters);

    double theta = Math.toRadians(launchElevationDegrees(hoodDegrees));
    double speed = exitSpeed(rpm);
    double verticalSpeed = speed * Math.sin(theta);
    double horizontalSpeed = speed * Math.cos(theta);

    double apex = LAUNCH_HEIGHT_METERS + verticalSpeed * verticalSpeed / (2.0 * GRAVITY);

    // Later root of the height equation: the Fuel on its way back down through Hub height.
    double discriminant =
        verticalSpeed * verticalSpeed - 2.0 * GRAVITY * (HUB_HEIGHT_METERS - LAUNCH_HEIGHT_METERS);
    if (discriminant < 0.0) {
      // Never reaches Hub height at all. Report it as a total miss rather than throwing.
      return new Shot(Double.MAX_VALUE, apex, 0.0);
    }
    double descentTime = (verticalSpeed + Math.sqrt(discriminant)) / GRAVITY;

    double travelled = horizontalSpeed * descentTime;
    double target = distanceMeters - SHOOTER_OFFSET_METERS;
    double entryAngle =
        Math.toDegrees(Math.atan2(GRAVITY * descentTime - verticalSpeed, horizontalSpeed));

    return new Shot(target - travelled, apex, entryAngle);
  }

  private static double launchElevationDegrees(double hoodDegrees) {
    return SimulationConstants.LAUNCH_ANGLE_OFFSET_DEGREES
        + SimulationConstants.LAUNCH_ANGLE_PER_HOOD_DEGREE * hoodDegrees;
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
