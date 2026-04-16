package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import frc.lib.AllianceFlipUtil;
import frc.lib.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

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

  /**
   * Supplier that provides the current estimated pose from Drive's pose estimator.
   *
   * <p>RobotState does NOT maintain its own pose estimator. Instead, it delegates to Drive's single
   * SwerveDrivePoseEstimator via this supplier. This eliminates the dual-estimator divergence bug
   * where two independent estimators would drift apart and cause pose jumps when vision was lost.
   */
  private Supplier<Pose2d> poseSupplier = Pose2d::new;

  /** Kinematics for converting between chassis speeds and module states */
  private final SwerveDriveKinematics kinematics;

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
    // Kinematics is still needed for velocity calculations (toChassisSpeeds).
    // Drive.getModuleTranslations() reads from TunerConstants — the single source of truth.
    kinematics = new SwerveDriveKinematics(Drive.getModuleTranslations());
  }

  /**
   * Sets the pose supplier that RobotState uses to get the current estimated pose.
   *
   * <p>This should be called once during initialization (e.g., in Drive's constructor) to wire
   * RobotState to Drive's single pose estimator.
   *
   * @param poseSupplier A supplier that returns the current estimated pose (e.g., drive::getPose)
   */
  public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
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
    return poseSupplier.get();
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
    // CPU FIX: removed redundant Logger.recordOutput — @AutoLogOutput already logs the return value
    return getDistanceToPoint(hubTarget.toTranslation2d());
  }

  // REMOVED: getDistanceToOpposingHub() — was never called from any other code and was
  // annotated with @AutoLogOutput, meaning AdvantageKit called it every loop for nothing.
  // That computed AllianceFlipUtil.apply + getEstimatedPose + getDistance every 20ms with no
  // consumer.

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

  // REMOVED: getOpposingHubTarget() — only caller was getDistanceToOpposingHub(), which was
  // itself dead code. Ran AllianceFlipUtil.apply() every call for no consumer.

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
    // Add 180° because the shooter faces the back of the robot
    return new Rotation2d(robotToHub.getX(), robotToHub.getY()).plus(Rotation2d.kPi);
  }

  // Gets angle to any target
  public Rotation2d getAngleToTarget(Translation2d target) {
    // Get current robot position
    Pose2d currentPose = getEstimatedPose();

    // Calculate the vector from robot to target
    Translation2d robotToTarget = target.minus(currentPose.getTranslation());

    // Calculate the angle using atan2
    // This gives us the direction we need to face to point at the target
    // Add 180° because the shooter faces the back of the robot
    return new Rotation2d(robotToTarget.getX(), robotToTarget.getY()).plus(Rotation2d.kPi);
  }

  // ==================== MODULE STATE UPDATES ====================

  /**
   * Updates the current module states used for velocity calculation.
   *
   * <p>Called once per cycle from Drive.periodic() after the high-frequency odometry loop. Only the
   * module states (velocity + angle) are needed — pose estimation is handled entirely by Drive's
   * single SwerveDrivePoseEstimator.
   *
   * @param moduleStates The current module states (velocity and angle for each module)
   */
  public void updateModuleStates(SwerveModuleState[] moduleStates) {
    currentModuleStates = moduleStates;
  }

  // SHOOTING ALIGNMENT

  /**
   * Returns true if the robot's current heading is within the hub alignment tolerance.
   *
   * <p>Used as the "ready to fire" condition for hub shots. The tolerance is defined in {@code
   * HardwareConstants.CompConstants.Thresholds.hubAlignmentToleranceDegrees} and can be tuned
   * without changing any command logic.
   *
   * @return true when the robot is pointing at the hub within the allowed tolerance
   */
  @AutoLogOutput(key = "RobotState/IsAlignedToHub")
  public boolean isAlignedToHub() {
    // Calculate the error between our current heading and the desired hub-facing angle.
    // Rotation2d.minus() handles wrap-around automatically (e.g., 179° - (-179°) = 2°, not 358°).
    double errorDegrees =
        Math.abs(
            getAngleToAllianceHub().minus(getEstimatedPose().getRotation()).getDegrees());
    return errorDegrees < HardwareConstants.CompConstants.Thresholds.hubAlignmentToleranceDegrees;
  }

  /**
   * Returns true if the robot's current heading is within the pass alignment tolerance.
   *
   * <p>Used as the "ready to fire" condition for pass shots. The tolerance is defined in {@code
   * HardwareConstants.CompConstants.Thresholds.passAlignmentToleranceDegrees} and can be tuned
   * without changing any command logic.
   *
   * @return true when the robot is pointing at the pass target within the allowed tolerance
   */
  @AutoLogOutput(key = "RobotState/IsAlignedToPass")
  public boolean isAlignedToPass() {
    Translation2d passTarget2d =
        new Translation2d(getPassTarget().getX(), getPassTarget().getY());
    double errorDegrees =
        Math.abs(
            getAngleToTarget(passTarget2d).minus(getEstimatedPose().getRotation()).getDegrees());
    return errorDegrees < HardwareConstants.CompConstants.Thresholds.passAlignmentToleranceDegrees;
  }

  // Finds pass target based on position
  // CPU FIX: replaced DriverStation.getAlliance() (creates Optional every call) with cached
  // AllianceFlipUtil.shouldFlip(). Also replaced RobotState.getInstance() self-call with
  // direct getEstimatedPose() since we're already inside the singleton.
  public Translation3d getPassTarget() {
    Translation3d passTarget;
    double poseY = getEstimatedPose().getY();
    // shouldFlip() returns true for Red alliance (cached per-loop in AllianceFlipUtil.refresh())
    if (AllianceFlipUtil.shouldFlip()) {
      // Red alliance
      if (poseY > (FieldConstants.fieldWidth / 2)) {
        passTarget =
            new Translation3d(Meters.of(13.373).magnitude(), Meters.of(6.136).magnitude(), 0);
      } else {
        passTarget =
            new Translation3d(Meters.of(12.950).magnitude(), Meters.of(2.293).magnitude(), 0);
      }
    } else {
      // Blue alliance (or unknown — defaults to blue)
      if (poseY < (FieldConstants.fieldWidth / 2)) {
        passTarget =
            new Translation3d(Meters.of(2.993).magnitude(), Meters.of(2.114).magnitude(), 0);
      } else {
        passTarget =
            new Translation3d(Meters.of(3.135).magnitude(), Meters.of(6.129).magnitude(), 0);
      }
    }
    return passTarget;
  }

  // REMOVED: getShootAngleForZoneAndTime() and getShootAngleForZone()
  // Neither method was called from any other code. Both called getAngleToAllianceHub(),
  // getPassTarget(), getAngleToTarget(), and Triggers.getInstance() — all expensive.
  // getShootAngleForZoneAndTime() also called getPassTarget() twice on line 375 in the same
  // expression: new Translation2d(getPassTarget().getX(), getPassTarget().getY()).

  // ZONE CALCULATIONS

  // Returns a "broad" zone - alliance zone, alliance trench, neutral, opposing trench, opposing
  // zone
  public HardwareConstants.Zones.broadZone getBroadZone(Pose2d pose) {
    double poseX = AllianceFlipUtil.applyX(pose.getX());
    if (poseX < FieldConstants.LinesVertical.allianceZone) {
      return HardwareConstants.Zones.broadZone.ALLIANCE_ZONE;
    } else if (poseX < FieldConstants.LinesVertical.neutralZoneNear) {
      return HardwareConstants.Zones.broadZone.ALLIANCE_TRENCH;
    } else if (poseX < FieldConstants.LinesVertical.neutralZoneFar) {
      return HardwareConstants.Zones.broadZone.NEUTRAL;
    } else if (poseX < FieldConstants.LinesVertical.oppAllianceZone) {
      return HardwareConstants.Zones.broadZone.OPPOSING_TRENCH;
    } else {
      return HardwareConstants.Zones.broadZone.OPPOSING_ZONE;
    }
  }

  // Returns a "specific" zone - alliance/opposing tower/trench/bump near/far
  // Returns null if in neutral zone
  public HardwareConstants.Zones.specificZone getSpecificZone(Pose2d pose) {
    double poseX = AllianceFlipUtil.applyX(pose.getX());
    double poseY = AllianceFlipUtil.applyY(pose.getY());
    // CPU FIX: cache getBroadZone result — was called up to 4 times for the same pose
    HardwareConstants.Zones.broadZone broadZone = getBroadZone(pose);
    if ((broadZone == HardwareConstants.Zones.broadZone.ALLIANCE_ZONE)
        && (poseX < FieldConstants.Tower.frontFaceX)
        && (poseY < FieldConstants.Tower.leftUpright.getY())
        && poseY > FieldConstants.Tower.rightUpright.getY()) {
      return (HardwareConstants.Zones.specificZone.ALLIANCE_TOWER);
    } else if (broadZone == HardwareConstants.Zones.broadZone.ALLIANCE_TRENCH) {
      if (poseY < FieldConstants.RightTrench.openingTopLeft.getY()) {
        return HardwareConstants.Zones.specificZone.ALLIANCE_TRENCH_NEAR;
      } else if (poseY < FieldConstants.RightBump.nearLeftCorner.getY()) {
        return HardwareConstants.Zones.specificZone.ALLIANCE_BUMP_NEAR;
      } else if (poseY < FieldConstants.LeftTrench.openingTopRight.getY()) {
        return HardwareConstants.Zones.specificZone.ALLIANCE_HUB;
      } else if (poseY < FieldConstants.LeftTrench.openingTopLeft.getY()) {
        return HardwareConstants.Zones.specificZone.ALLIANCE_BUMP_FAR;
      } else {
        return HardwareConstants.Zones.specificZone.ALLIANCE_TRENCH_FAR;
      }
    } else if (broadZone == HardwareConstants.Zones.broadZone.OPPOSING_TRENCH) {
      if (poseY < FieldConstants.RightTrench.openingTopLeft.getY()) {
        return HardwareConstants.Zones.specificZone.OPPOSING_TRENCH_NEAR;
      } else if (poseY < FieldConstants.RightBump.nearLeftCorner.getY()) {
        return HardwareConstants.Zones.specificZone.OPPOSING_BUMP_NEAR;
      } else if (poseY < FieldConstants.LeftTrench.openingTopRight.getY()) {
        return HardwareConstants.Zones.specificZone.OPPOSING_HUB;
      } else if (poseY < FieldConstants.LeftTrench.openingTopLeft.getY()) {
        return HardwareConstants.Zones.specificZone.OPPOSING_BUMP_FAR;
      } else {
        return HardwareConstants.Zones.specificZone.OPPOSING_TRENCH_FAR;
      }
    } else if ((broadZone == HardwareConstants.Zones.broadZone.OPPOSING_ZONE)
        && (poseX < FieldConstants.Tower.oppLeftUpright.getX())
        && (poseY < FieldConstants.Tower.oppLeftUpright.getY())
        && (poseY > FieldConstants.Tower.oppRightUpright.getY())) {
      return HardwareConstants.Zones.specificZone.OPPOSING_TOWER;
    } else {
      return null;
    }
  }

  public HardwareConstants.Zones.approachingZoneX getApproachingZoneX(Pose2d pose) {
    double poseX = AllianceFlipUtil.applyX(pose.getX());
    if (poseX
        < (FieldConstants.Tower.leftUpright.getX() + HardwareConstants.Zones.approachingXOffset)) {
      return HardwareConstants.Zones.approachingZoneX.APPROACHING_ALLIANCE_TOWER;
    } else if ((poseX
            > FieldConstants.LinesVertical.allianceZone
                - HardwareConstants.Zones.approachingXOffset)
        && (poseX
            < FieldConstants.LinesVertical.neutralZoneNear
                + HardwareConstants.Zones.approachingXOffset)) {
      return HardwareConstants.Zones.approachingZoneX.APPROACHING_ALLIANCE_TRENCH;
    } else if ((poseX
            > FieldConstants.LinesVertical.neutralZoneFar
                - HardwareConstants.Zones.approachingXOffset)
        && (poseX
            < FieldConstants.LinesVertical.oppAllianceZone
                + HardwareConstants.Zones.approachingXOffset)) {
      return HardwareConstants.Zones.approachingZoneX.APPROACHING_OPPOSING_TRENCH;
    } else if (poseX
        > FieldConstants.Tower.oppLeftUpright.getX() - HardwareConstants.Zones.approachingXOffset) {
      return HardwareConstants.Zones.approachingZoneX.APPROACHING_OPPOSING_TOWER;
    } else {
      return null;
    }
  }

  public HardwareConstants.Zones.approachingZoneY getApproachingZoneY(Pose2d pose) {
    double poseY = AllianceFlipUtil.applyY(pose.getY());
    // CPU FIX: cache getBroadZone result — was called up to 2 times for the same pose
    HardwareConstants.Zones.broadZone broadZone = getBroadZone(pose);
    if ((broadZone == HardwareConstants.Zones.broadZone.ALLIANCE_ZONE)
        && ((poseY
                > (FieldConstants.Tower.leftUpright.getY()
                    - HardwareConstants.Zones.approachingYOffset))
            || (poseY
                < (FieldConstants.Tower.rightUpright.getY()
                    + HardwareConstants.Zones.approachingYOffset)))) {
      return HardwareConstants.Zones.approachingZoneY.APPROACHING_ALLIANCE_TOWER;
    } else if ((broadZone == HardwareConstants.Zones.broadZone.OPPOSING_ZONE)
        && ((poseY
                > (FieldConstants.Tower.oppLeftUpright.getY()
                    - HardwareConstants.Zones.approachingYOffset))
            || (poseY
                < (FieldConstants.Tower.oppRightUpright.getY()
                    + HardwareConstants.Zones.approachingYOffset)))) {
      return HardwareConstants.Zones.approachingZoneY.APPROACHING_OPPOSING_TOWER;
    } else if ((poseY < FieldConstants.RightBump.farLeftCorner.getY())
        || (poseY > FieldConstants.LeftBump.farRightCorner.getY())) {
      return HardwareConstants.Zones.approachingZoneY.APPROACHING_BUMP;
    } else if ((poseY > FieldConstants.RightBump.farLeftCorner.getY())
        || (poseY < FieldConstants.LeftBump.farRightCorner.getY())) {
      return HardwareConstants.Zones.approachingZoneY.APPROACHING_TRENCH;
    } else {
      return null;
    }
  }

  public HardwareConstants.Zones.approachingZoneComposite getApproachingZone(Pose2d pose) {
    // CPU FIX: cache results — getApproachingZoneX was called up to 6 times
    // and getApproachingZoneY up to 6 times for the same pose. Each call internally
    // called getBroadZone() and AllianceFlipUtil.applyX/Y() again.
    HardwareConstants.Zones.approachingZoneX zoneX = getApproachingZoneX(pose);
    HardwareConstants.Zones.approachingZoneY zoneY = getApproachingZoneY(pose);

    if ((zoneX == HardwareConstants.Zones.approachingZoneX.APPROACHING_ALLIANCE_TRENCH)
        && (zoneY == HardwareConstants.Zones.approachingZoneY.APPROACHING_TRENCH)) {
      return HardwareConstants.Zones.approachingZoneComposite.APPROACHING_ALLIANCE_TRENCH;
    } else if ((zoneX == HardwareConstants.Zones.approachingZoneX.APPROACHING_ALLIANCE_TRENCH)
        && zoneY == HardwareConstants.Zones.approachingZoneY.APPROACHING_BUMP) {
      return HardwareConstants.Zones.approachingZoneComposite.APPROACHING_ALLIANCE_BUMP;
    } else if ((zoneX == HardwareConstants.Zones.approachingZoneX.APPROACHING_OPPOSING_TRENCH)
        && zoneY == HardwareConstants.Zones.approachingZoneY.APPROACHING_TRENCH) {
      return HardwareConstants.Zones.approachingZoneComposite.APPROACHING_OPPOSING_TRENCH;
    } else if ((zoneX == HardwareConstants.Zones.approachingZoneX.APPROACHING_OPPOSING_TRENCH)
        && zoneY == HardwareConstants.Zones.approachingZoneY.APPROACHING_BUMP) {
      return HardwareConstants.Zones.approachingZoneComposite.APPROACHING_OPPOSING_BUMP;
    } else if ((zoneX == HardwareConstants.Zones.approachingZoneX.APPROACHING_ALLIANCE_TOWER)
        && (zoneY == HardwareConstants.Zones.approachingZoneY.APPROACHING_ALLIANCE_TOWER)) {
      return HardwareConstants.Zones.approachingZoneComposite.APPROACHING_ALLIANCE_TOWER;
    } else if ((zoneX == HardwareConstants.Zones.approachingZoneX.APPROACHING_OPPOSING_TOWER)
        && (zoneY == HardwareConstants.Zones.approachingZoneY.APPROACHING_OPPOSING_TOWER)) {
      return HardwareConstants.Zones.approachingZoneComposite.APPROACHING_OPPOSING_TOWER;
    } else {
      return null;
    }
  }
}
