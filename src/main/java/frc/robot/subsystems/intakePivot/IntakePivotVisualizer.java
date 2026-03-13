package frc.robot.subsystems.intakePivot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * Visualizes the intake pivot mechanism using Mechanism2d and logs a Pose3d for AdvantageScope 3D
 * visualization.
 *
 * <p><b>Physical geometry:</b> The intake pivots around a fixed joint. The position is expressed in
 * rotations, where 0 = stowed/retracted and positive values rotate the arm outward/downward.
 *
 * <p>The Mechanism2d canvas shows the arm as a bar that rotates from the pivot point:
 *
 * <ul>
 *   <li><b>Green bar:</b> Current measured angle
 *   <li><b>Yellow bar:</b> Goal / target angle
 *   <li><b>White lines:</b> Min/max angular bounds
 * </ul>
 *
 * <p>The Pose3d output shows the roller assembly at the tip of the arm in 3D space for
 * AdvantageScope's 3D robot viewer.
 */
public class IntakePivotVisualizer {

  // ---------- Mechanism2d canvas ----------
  private static final double CANVAS_WIDTH = 1.0; // meters
  private static final double CANVAS_HEIGHT = 1.0; // meters
  private static final double ROOT_X = 0.5; // root X on canvas (centered)
  private static final double ROOT_Y = 0.5; // root Y on canvas (centered)

  // Mechanism2d components
  private final LoggedMechanism2d _mechanism;
  private final LoggedMechanismLigament2d _measuredArm;
  private final LoggedMechanismLigament2d _goalArm;
  private final LoggedMechanismLigament2d _minBound;
  private final LoggedMechanismLigament2d _maxBound;

  // ---------- Config ----------
  private final String _name;

  /**
   * Creates a new IntakePivotVisualizer.
   *
   * @param name The name used for logging keys (e.g., "IntakePivot")
   */
  public IntakePivotVisualizer(String name) {
    _name = name;

    double armLength = IntakePivotConstants.Visualization.ARM_LENGTH_DISPLAY;

    _mechanism = new LoggedMechanism2d(CANVAS_WIDTH, CANVAS_HEIGHT, new Color8Bit(Color.kBlack));

    // Root is at the pivot joint (center of canvas)
    LoggedMechanismRoot2d root = _mechanism.getRoot(name + "_Root", ROOT_X, ROOT_Y);

    // Min bound indicator — a thin arm at the minimum angle
    double minAngleDeg =
        Units.rotationsToDegrees(IntakePivotConstants.SoftwareConstants.softwareLowerRotationLimit);
    _minBound =
        new LoggedMechanismLigament2d(
            name + "_MinBound",
            armLength * 0.5, // shorter for visibility
            minAngleDeg,
            2,
            new Color8Bit(Color.kWhite));

    // Max bound indicator — a thin arm at the maximum angle
    double maxAngleDeg =
        Units.rotationsToDegrees(IntakePivotConstants.SoftwareConstants.softwareUpperRotationLimit);
    _maxBound =
        new LoggedMechanismLigament2d(
            name + "_MaxBound",
            armLength * 0.5, // shorter for visibility
            maxAngleDeg,
            2,
            new Color8Bit(Color.kWhite));

    // Goal arm (yellow, slightly thinner — drawn first so measured draws on top)
    _goalArm =
        new LoggedMechanismLigament2d(
            name + "_Goal",
            armLength,
            0.0, // angle updated each loop
            4,
            new Color8Bit(Color.kYellow));

    // Measured arm (green, thicker)
    _measuredArm =
        new LoggedMechanismLigament2d(
            name + "_Measured",
            armLength,
            0.0, // angle updated each loop
            6,
            new Color8Bit(Color.kGreen));

    // Attach all ligaments to the root
    root.append(_minBound);
    root.append(_maxBound);
    root.append(_goalArm);
    root.append(_measuredArm);
  }

  /**
   * Updates the visualizer with the current pivot state.
   *
   * <p>Call this from {@link IntakePivot#periodic()}.
   *
   * @param currentPositionRotations The current pivot position in rotations (0 = stowed)
   * @param goalPositionRotations The goal / setpoint position in rotations
   * @param atGoal Whether the mechanism is within the deadband of its goal
   */
  public void update(
      double currentPositionRotations, double goalPositionRotations, boolean atGoal) {
    // Clamp to valid range for display
    double lowerLimit = IntakePivotConstants.SoftwareConstants.softwareLowerRotationLimit;
    double upperLimit = IntakePivotConstants.SoftwareConstants.softwareUpperRotationLimit;
    double currentRotations = Math.max(lowerLimit, Math.min(currentPositionRotations, upperLimit));
    double goalRotations = Math.max(lowerLimit, Math.min(goalPositionRotations, upperLimit));

    // Convert rotations to degrees for Mechanism2d ligament angles
    double currentDeg = Units.rotationsToDegrees(currentRotations);
    double goalDeg = Units.rotationsToDegrees(goalRotations);

    // Update arm angles
    _measuredArm.setAngle(currentDeg);
    _goalArm.setAngle(goalDeg);

    // Background color: dark green when at goal, black otherwise
    _mechanism.setBackgroundColor(
        atGoal ? new Color8Bit(Color.kDarkGreen) : new Color8Bit(Color.kBlack));

    // Publish Mechanism2d to SmartDashboard and AdvantageKit
    SmartDashboard.putData(_name + " Visualizer", _mechanism);
    Logger.recordOutput(_name + "/Visualizer/Mechanism2d", _mechanism);

    // --- 3D Pose ---
    // Convert the current angle from rotations to radians for the 3D pose
    double pivotAngleRad = Units.rotationsToRadians(currentRotations);

    // The arm tip position relative to the pivot base
    double armLength = IntakePivotConstants.Visualization.ARM_LENGTH_DISPLAY;
    double dx = armLength * Math.cos(pivotAngleRad); // forward along robot X
    double dz = armLength * Math.sin(pivotAngleRad); // up/down along robot Z

    Translation3d baseTranslation = IntakePivotConstants.Visualization.PIVOT_BASE_OFFSET;
    Pose3d armTipPose =
        new Pose3d(
            new Translation3d(
                baseTranslation.getX() + dx, baseTranslation.getY(), baseTranslation.getZ() + dz),
            // The arm rotates around the Y axis (pitch) in the robot frame
            new Rotation3d(0.0, -pivotAngleRad, 0.0));

    Logger.recordOutput(_name + "/Visualizer/Pose3d", armTipPose);

    // Scalar logs for easy graphing
    Logger.recordOutput(_name + "/Visualizer/CurrentPosition_rot", currentRotations);
    Logger.recordOutput(_name + "/Visualizer/GoalPosition_rot", goalRotations);
    Logger.recordOutput(_name + "/Visualizer/CurrentAngle_deg", currentDeg);
    Logger.recordOutput(_name + "/Visualizer/GoalAngle_deg", goalDeg);
    Logger.recordOutput(_name + "/Visualizer/AtGoal", atGoal);
  }
}
