package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import frc.lib.AllianceFlipUtil;
import frc.lib.FieldConstants;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Centralized robot state container that tracks the robot's position and velocity on the field.
 *
 * <p>This is a singleton class that provides a single source of truth for:
 *
 * <ul>
 *   <li>Robot pose (position and rotation on the field)
 *   <li>Robot velocity (field-relative and robot-relative)
 *   <li>Distance calculations to field elements (hubs)
 * </ul>
 *
 * <p><b>Why use a centralized RobotState?</b>
 *
 * <ul>
 *   <li>Decouples subsystems - Shooter doesn't need a reference to Drive
 *   <li>Single source of truth - All subsystems see the same robot state
 *   <li>Easier testing - Can mock robot state for unit tests
 *   <li>Cleaner architecture - Subsystems don't need pose/speed suppliers
 * </ul>
 *
 * <p><b>Usage:</b>
 *
 * <pre>
 * // Get the singleton instance
 * RobotState state = RobotState.getInstance();
 *
 * // Get current pose
 * Pose2d pose = state.getEstimatedPose();
 *
 * // Get distance to alliance hub (auto-flips for red alliance)
 * Distance distance = state.getDistanceToAllianceHub();
 * </pre>
 *
 * <p><b>Odometry Updates:</b> The Drive subsystem calls addOdometryObservation() during its
 * periodic loop to update the pose estimator with wheel encoder and gyro data.
 */
public class RobotState {

  // ==================== SINGLETON PATTERN ====================

  private static RobotState instance;

  /**
   * Returns the singleton instance of RobotState.
   *
   * <p>Creates the instance on first call (lazy initialization).
   *
   * @return The RobotState singleton
   */
  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  // ==================== POSE ESTIMATION ====================

  /** Kinematics for converting between chassis speeds and module states */
  private final SwerveDriveKinematics kinematics;

  /** Pose estimator that fuses odometry and vision measurements */
  private final SwerveDrivePoseEstimator poseEstimator;

  /** Last known gyro rotation (used for odometry updates) */
  private Rotation2d rawGyroRotation = new Rotation2d();

  /** Last known module positions (used for delta calculations) */
  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  /** Current module states for velocity calculation */
  private SwerveModuleState[] currentModuleStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  // ==================== CONSTRUCTOR ====================

  /** Private constructor - use getInstance() instead. */
  private RobotState() {
    // Initialize kinematics with module positions from DriveConstants
    kinematics = new SwerveDriveKinematics(DriveConstants.moduleTranslations);

    // Initialize pose estimator at origin
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
  }

  // ==================== POSE GETTERS ====================

  /**
   * Returns the current estimated robot pose on the field.
   *
   * <p>The pose includes:
   *
   * <ul>
   *   <li>X position - distance along the field length (meters)
   *   <li>Y position - distance along the field width (meters)
   *   <li>Rotation - which direction the robot is facing
   * </ul>
   *
   * <p>The origin (0, 0) is at the blue alliance corner, with positive X toward the red alliance
   * and positive Y to the left when looking from the blue alliance.
   *
   * @return The robot's current estimated pose
   */
  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Returns the robot's current rotation (heading) on the field.
   *
   * @return The robot's current rotation
   */
  public Rotation2d getRotation() {
    return getEstimatedPose().getRotation();
  }

  // ==================== VELOCITY GETTERS ====================

  /**
   * Returns the robot's velocity in field-relative coordinates.
   *
   * <p>Field-relative means the velocity is measured from the field's perspective:
   *
   * <ul>
   *   <li>vx = velocity toward the red alliance (positive X direction)
   *   <li>vy = velocity toward the left side of the field (positive Y direction)
   *   <li>omega = rotational velocity (counterclockwise positive)
   * </ul>
   *
   * <p>This is used by the shooter to compensate for robot motion when calculating shot
   * trajectories.
   *
   * @return Field-relative chassis speeds
   */
  @AutoLogOutput(key = "RobotState/FieldRelativeVelocity")
  public ChassisSpeeds getFieldRelativeVelocity() {
    ChassisSpeeds robotRelative = getRobotRelativeVelocity();
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, getRotation());
  }

  /**
   * Returns the robot's velocity in robot-relative coordinates.
   *
   * <p>Robot-relative means the velocity is measured from the robot's perspective:
   *
   * <ul>
   *   <li>vx = forward velocity (positive = driving forward)
   *   <li>vy = left velocity (positive = strafing left)
   *   <li>omega = rotational velocity (counterclockwise positive)
   * </ul>
   *
   * @return Robot-relative chassis speeds
   */
  @AutoLogOutput(key = "RobotState/RobotRelativeVelocity")
  public ChassisSpeeds getRobotRelativeVelocity() {
    return kinematics.toChassisSpeeds(currentModuleStates);
  }

  // ==================== DISTANCE CALCULATIONS ====================

  /**
   * Returns the 2D distance from the robot to the alliance hub.
   *
   * <p>This automatically flips based on alliance color:
   *
   * <ul>
   *   <li>Blue alliance → returns distance to blue hub
   *   <li>Red alliance → returns distance to red hub
   * </ul>
   *
   * <p>The distance is calculated to the top center point of the hub (the scoring target).
   *
   * @return Distance to the alliance hub as a Distance measure
   */
  @AutoLogOutput(key = "RobotState/DistanceToAllianceHub_m")
  public Distance getDistanceToAllianceHub() {
    Translation3d hubTarget = getAllianceHubTarget();
    return getDistanceToPoint(hubTarget.toTranslation2d());
  }

  /**
   * Returns the 2D distance from the robot to the opposing alliance hub.
   *
   * <p>This automatically flips based on alliance color:
   *
   * <ul>
   *   <li>Blue alliance → returns distance to red hub
   *   <li>Red alliance → returns distance to blue hub
   * </ul>
   *
   * @return Distance to the opposing hub as a Distance measure
   */
  @AutoLogOutput(key = "RobotState/DistanceToOpposingHub_m")
  public Distance getDistanceToOpposingHub() {
    Translation3d hubTarget = getOpposingHubTarget();
    return getDistanceToPoint(hubTarget.toTranslation2d());
  }

  /**
   * Returns the 2D distance from the robot to an arbitrary point on the field.
   *
   * @param point The target point (2D field coordinates)
   * @return Distance to the point
   */
  public Distance getDistanceToPoint(Translation2d point) {
    Translation2d robotPosition = getEstimatedPose().getTranslation();
    double distanceMeters = robotPosition.getDistance(point);
    return Meters.of(distanceMeters);
  }

  /**
   * Returns the 3D position of the alliance hub target (top center).
   *
   * <p>Auto-flips based on alliance color.
   *
   * @return The alliance hub target position
   */
  public Translation3d getAllianceHubTarget() {
    // FieldConstants.Hub.topCenterPoint is defined from blue alliance perspective
    // AllianceFlipUtil.apply() flips it if we're on red alliance
    return AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint);
  }

  /**
   * Returns the 3D position of the opposing alliance hub target.
   *
   * <p>Auto-flips based on alliance color.
   *
   * @return The opposing hub target position
   */
  public Translation3d getOpposingHubTarget() {
    // oppTopCenterPoint is the opposing hub from blue perspective
    // Flip it if we're on red alliance
    return AllianceFlipUtil.apply(FieldConstants.Hub.oppTopCenterPoint);
  }

  /**
   * Returns the angle the robot should face to point at the alliance hub.
   *
   * <p>This calculates the bearing from the robot's current position to the alliance hub target.
   * The returned angle is automatically normalized to choose the shortest rotation path.
   *
   * <p><b>How it works:</b>
   *
   * <ol>
   *   <li>Gets the robot's current position
   *   <li>Gets the alliance hub target position (auto-flips for red alliance)
   *   <li>Calculates the angle from robot to hub using atan2
   *   <li>Returns a Rotation2d that can be passed to joystickDriveAtAngle
   * </ol>
   *
   * <p><b>Continuous Input Handling:</b> The ProfiledPIDController in joystickDriveAtAngle has
   * continuous input enabled (-π to π), so it will automatically choose the shortest rotation path.
   * For example, if the robot is at 170° and the hub is at -170°, it will rotate 20° instead of
   * 340°.
   *
   * <p><b>Usage:</b>
   *
   * <pre>
   * // In a command - continuously update heading to point at hub
   * DriveCommands.joystickDriveAtAngle(
   *   drive,
   *   xSupplier,
   *   ySupplier,
   *   () -> RobotState.getInstance().getAngleToAllianceHub()
   * );
   * </pre>
   *
   * @return The heading the robot should face to point at the alliance hub
   */
  @AutoLogOutput(key = "RobotState/AngleToAllianceHub")
  public Rotation2d getAngleToAllianceHub() {
    // Get current robot position
    Pose2d currentPose = getEstimatedPose();

    // Get alliance hub target (2D position on the field)
    Translation3d hubTarget3d = getAllianceHubTarget();
    Translation2d hubTarget2d = hubTarget3d.toTranslation2d();

    // Calculate the vector from robot to hub
    Translation2d robotToHub = hubTarget2d.minus(currentPose.getTranslation());

    // Calculate the angle using atan2
    // This gives us the direction we need to face to point at the hub
    return new Rotation2d(robotToHub.getX(), robotToHub.getY());
  }

  // ==================== ODOMETRY UPDATES ====================

  /**
   * Adds an odometry observation to update the pose estimate.
   *
   * <p>This should be called by the Drive subsystem every loop with the latest gyro reading and
   * module positions.
   *
   * <p><b>How it works:</b>
   *
   * <ol>
   *   <li>Calculates how far each wheel has moved since last update
   *   <li>Combines wheel movements with gyro rotation
   *   <li>Updates the pose estimator with the new data
   * </ol>
   *
   * @param gyroRotation The current gyro rotation
   * @param modulePositions The current module positions (distance and angle for each module)
   * @param moduleStates The current module states (velocity and angle for each module)
   */
  public void addOdometryObservation(
      Rotation2d gyroRotation,
      SwerveModulePosition[] modulePositions,
      SwerveModuleState[] moduleStates) {

    // Store for next iteration
    rawGyroRotation = gyroRotation;
    lastModulePositions = modulePositions;
    currentModuleStates = moduleStates;

    // Update pose estimator
    poseEstimator.update(gyroRotation, modulePositions);

    // Log the update
    Logger.recordOutput("RobotState/GyroRotation", gyroRotation);
  }

  /**
   * Adds an odometry observation with a specific timestamp.
   *
   * <p>This version is used when processing high-frequency odometry samples that were captured at
   * different times during a single loop.
   *
   * @param timestamp The timestamp when the observation was captured (seconds)
   * @param gyroRotation The gyro rotation at that timestamp
   * @param modulePositions The module positions at that timestamp
   * @param moduleStates The module states for velocity calculation
   */
  public void addOdometryObservation(
      double timestamp,
      Rotation2d gyroRotation,
      SwerveModulePosition[] modulePositions,
      SwerveModuleState[] moduleStates) {

    // Store for next iteration
    rawGyroRotation = gyroRotation;
    lastModulePositions = modulePositions;
    currentModuleStates = moduleStates;

    // Update pose estimator with timestamp
    poseEstimator.updateWithTime(timestamp, gyroRotation, modulePositions);
  }

  /**
   * Adds a vision measurement to improve the pose estimate.
   *
   * <p>Vision measurements help correct for odometry drift. The standard deviations control how
   * much the vision data is trusted compared to odometry.
   *
   * @param visionPose The pose measured by vision
   * @param timestampSeconds When the measurement was captured
   * @param stdDevs Standard deviations for x, y, and rotation (smaller = more trust)
   */
  public void addVisionMeasurement(
      Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
  }

  /**
   * Resets the pose estimator to a specific pose.
   *
   * <p>Use this to set the robot's starting position at the beginning of autonomous.
   *
   * @param pose The pose to reset to
   */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, lastModulePositions, pose);
  }
}
