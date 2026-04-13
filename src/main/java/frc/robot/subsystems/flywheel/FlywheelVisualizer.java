package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.RobotState;
import frc.robot.subsystems.flywheel.FlywheelConstants.TrajectoryVisualization;
import org.littletonrobotics.junction.Logger;

/**
 * Generates a predicted trajectory for fuel launched from the flywheel.
 *
 * <p>Uses parabolic (projectile motion) equations to sample points along the predicted flight path.
 * The trajectory is logged as a {@code Translation3d[]} for AdvantageScope's 3D viewer.
 *
 * <p><b>Physics — Projectile Motion:</b>
 *
 * <pre>
 *   x(t) = x₀ + vₓ × t           (forward — no drag)
 *   y(t) = y₀ + vᵧ × t           (lateral — no drag)
 *   z(t) = z₀ + vᵤ × t − ½g × t² (vertical — gravity pulls down)
 * </pre>
 *
 * <p>The trajectory is computed in field-relative coordinates using the robot's current pose and
 * heading. Robot velocity is intentionally NOT included — this shows where the fuel would land if
 * the robot were stationary.
 */
public class FlywheelVisualizer {

  /** Gravitational acceleration (m/s²). */
  private static final double GRAVITY = 9.81;

  /** Reusable array for the trajectory points — avoids allocations every loop. */
  private final Translation3d[] trajectory;

  /** Empty array logged when the flywheel is below the RPM threshold. */
  private static final Translation3d[] EMPTY_TRAJECTORY = new Translation3d[0];

  public FlywheelVisualizer() {
    trajectory = new Translation3d[TrajectoryVisualization.TRAJECTORY_POINTS];
  }

  /**
   * Converts flywheel angular velocity to linear velocity at the drum surface.
   *
   * <p><b>v = ω × r</b> where ω is in rad/s and r is the drum radius.
   *
   * @param flywheelSpeed The flywheel angular velocity
   * @return Linear speed in m/s at the drum edge
   */
  private double toLinearVelocity(AngularVelocity flywheelSpeed) {
    // Convert to rad/s, multiply by drum radius, apply fudge factor
    double omegaRadPerSec = flywheelSpeed.in(edu.wpi.first.units.Units.RadiansPerSecond);
    return omegaRadPerSec
        * TrajectoryVisualization.DRUM_RADIUS_METERS
        * TrajectoryVisualization.VELOCITY_FUDGE_FACTOR;
  }

  /**
   * Updates the trajectory visualization based on current flywheel speed and hood angle.
   *
   * <p>Call this every loop from {@link Flywheel#periodic()}.
   *
   * <p>If the flywheel RPM is below {@link TrajectoryVisualization#MIN_RPM_FOR_TRAJECTORY}, an
   * empty trajectory is logged so the AdvantageScope trail disappears.
   *
   * @param flywheelSpeed The current measured flywheel angular velocity
   * @param hoodAngle The current hood angle (mechanism position as an Angle)
   */
  public void updateTrajectory(AngularVelocity flywheelSpeed, Angle hoodAngle) {
    double currentRPM = flywheelSpeed.in(RPM);

    // --- Robot pose (needed for both the trajectory and the launch origin) ---
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    double robotX = robotPose.getX();
    double robotY = robotPose.getY();
    double headingRad = robotPose.getRotation().getRadians();

    // The shooter faces backward (-X in robot frame), so the launch direction
    // is 180° from the robot heading.
    double launchHeadingRad = headingRad + Math.PI;

    // Hood angle: positive = tilting up toward Z axis from horizontal
    double hoodAngleRad = hoodAngle.in(Radians);

    // --- Launch origin Pose3d ---
    // Rotate the robot-relative shooter exit offset into field coordinates
    double cosHeading = Math.cos(headingRad);
    double sinHeading = Math.sin(headingRad);
    double exitX =
        robotX
            + TrajectoryVisualization.SHOOTER_EXIT_X_METERS * cosHeading
            - TrajectoryVisualization.SHOOTER_EXIT_Y_METERS * sinHeading;
    double exitY =
        robotY
            + TrajectoryVisualization.SHOOTER_EXIT_X_METERS * sinHeading
            + TrajectoryVisualization.SHOOTER_EXIT_Y_METERS * cosHeading;

    // The launch origin pose:
    //   Translation = shooter exit point at launch height
    //   Rotation    = yaw pointing in launch direction, pitch tilted up by hood angle
    //                 (negative pitch = nose up in WPILib convention)
    Pose3d launchOriginPose =
        new Pose3d(
            new Translation3d(exitX, exitY, TrajectoryVisualization.LAUNCH_HEIGHT_METERS),
            new Rotation3d(0.0, -hoodAngleRad, launchHeadingRad));

    // Always log the launch origin so it tracks the robot even when not shooting
    Logger.recordOutput("Flywheel/Visualizer/LaunchOriginPose", launchOriginPose);

    // Below the minimum RPM threshold — log an empty trajectory and return
    if (Math.abs(currentRPM) < TrajectoryVisualization.MIN_RPM_FOR_TRAJECTORY) {
      Logger.recordOutput("Flywheel/Visualizer/Trajectory", EMPTY_TRAJECTORY);
      return;
    }

    // --- Compute launch velocity components ---
    double launchSpeed = toLinearVelocity(flywheelSpeed); // m/s at the drum edge

    // Decompose into horizontal and vertical components
    double vHorizontal = launchSpeed * Math.cos(hoodAngleRad); // speed along ground plane
    double vVertical = launchSpeed * Math.sin(hoodAngleRad); // speed upward

    // Field-relative horizontal velocity components
    double vFieldX = vHorizontal * Math.cos(launchHeadingRad);
    double vFieldY = vHorizontal * Math.sin(launchHeadingRad);

    // --- Sample the parabolic trajectory ---
    double timeStep =
        TrajectoryVisualization.TRAJECTORY_TIME_SPAN / TrajectoryVisualization.TRAJECTORY_POINTS;
    double launchZ = TrajectoryVisualization.LAUNCH_HEIGHT_METERS;

    for (int i = 0; i < trajectory.length; i++) {
      double t = timeStep * (i + 1); // time at this sample point

      // Parabolic equations of motion (no drag)
      // x(t) = x₀ + vₓ × t
      // y(t) = y₀ + vᵧ × t
      // z(t) = z₀ + vᵤ × t − ½g × t²
      double x = exitX + vFieldX * t;
      double y = exitY + vFieldY * t;
      double z = launchZ + vVertical * t - 0.5 * GRAVITY * t * t;

      // Clamp z to ground level so the trajectory doesn't go underground
      trajectory[i] = new Translation3d(x, y, Math.max(z, 0.0));
    }

    Logger.recordOutput("Flywheel/Visualizer/Trajectory", trajectory);
  }
}
