package frc.robot.util;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.intakePivot.IntakePivotConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Publishes the articulated component poses for the AdvantageScope 3D robot model ("Robot_Omega").
 *
 * <p>Logs a single {@code Pose3d[]} at {@code RobotModel/ComponentPoses}. In AdvantageScope, drag
 * that field onto the robot object in the 3D view as its "Components" source. The array order MUST
 * match the {@code components} list in {@code Advantage Scope Assets/Robot_Omega/config.json}:
 *
 * <ol>
 *   <li>{@code model_0.glb} — Intake (slap-down), pitches about its pivot shaft
 *   <li>{@code model_1.glb} — Hood, pitches about the drum axis
 *   <li>{@code model_2.glb} — Drum, spins about the drum axis
 *   <li>{@code model_3.glb} — Hopper, slides as a function of the intake pivot angle
 * </ol>
 *
 * <p><b>Reference pose:</b> the CAD was exported with the intake DEPLOYED (pivot = 0 rotations, the
 * encoder zero) and the hood stowed, so a zero mechanism reading reproduces the exported CAD pose
 * exactly.
 *
 * <p>All positions below were measured from the CAD export (robot frame: X forward toward the
 * intake, Y left, Z up). Signs and the hopper travel are visual estimates — nudge them while
 * watching AdvantageScope; they have no effect on robot behavior.
 */
public class RobotModelVisualizer {

  /** Intake pivot shaft axis in the robot frame (from CAD: the full-width 27.5" hex shaft). */
  private static final Translation3d INTAKE_PIVOT_POSITION = new Translation3d(0.2858, 0.0, 0.1429);

  /**
   * Pitch per rotation of measured intake angle. Measured angle 0 = deployed (down); increasing
   * angle stows the arm upward, which is a negative pitch for a front-mounted mechanism.
   */
  private static final double INTAKE_PITCH_SIGN = -1.0;

  /** Drum / hood pivot axis in the robot frame (from CAD: the 26.375" steel hex shaft). */
  private static final Translation3d SHOOTER_AXIS_POSITION = new Translation3d(-0.2762, 0.0, 0.415);

  /** Pitch direction of the hood as its measured angle increases. Tune visually. */
  private static final double HOOD_PITCH_SIGN = 1.0;

  /** Spin direction of the drum. Cosmetic only. */
  private static final double DRUM_SPIN_SIGN = 1.0;

  /**
   * Hopper displacement (robot frame) when the intake is fully STOWED, relative to the exported CAD
   * pose (intake deployed). The slot-and-pin linkage pulls the hopper inward as the intake stows.
   * Tune visually.
   */
  private static final Translation3d HOPPER_STOWED_TRAVEL =
      new Translation3d(Units.inchesToMeters(-9.0), 0.0, 0.0);

  private final Supplier<Angle> intakePivotAngle;
  private final Supplier<Angle> hoodAngle;
  private final Supplier<Angle> flywheelAngle;

  /** Reused each loop; only the Pose3d elements are reallocated. */
  private final Pose3d[] componentPoses = new Pose3d[4];

  /**
   * @param intakePivotAngle Measured intake pivot position (0 = deployed/down)
   * @param hoodAngle Measured hood position (0 = stowed)
   * @param flywheelAngle Accumulated flywheel leader angle, used to spin the drum
   */
  public RobotModelVisualizer(
      Supplier<Angle> intakePivotAngle, Supplier<Angle> hoodAngle, Supplier<Angle> flywheelAngle) {
    this.intakePivotAngle = intakePivotAngle;
    this.hoodAngle = hoodAngle;
    this.flywheelAngle = flywheelAngle;
  }

  /** Computes and logs all component poses. Call once per loop from robotPeriodic(). */
  public void update() {
    double intakeRad = intakePivotAngle.get().in(Radians);
    double hoodRad = hoodAngle.get().in(Radians);
    double drumRad = flywheelAngle.get().in(Radians);

    // Intake: rotate about its pivot shaft (robot Y axis)
    componentPoses[0] =
        new Pose3d(INTAKE_PIVOT_POSITION, new Rotation3d(0.0, INTAKE_PITCH_SIGN * intakeRad, 0.0));

    // Hood: rotates about the drum axis
    componentPoses[1] =
        new Pose3d(SHOOTER_AXIS_POSITION, new Rotation3d(0.0, HOOD_PITCH_SIGN * hoodRad, 0.0));

    // Drum: spins about the same axis (will shimmer at high RPM — logs are 50 Hz)
    componentPoses[2] =
        new Pose3d(SHOOTER_AXIS_POSITION, new Rotation3d(0.0, DRUM_SPIN_SIGN * drumRad, 0.0));

    // Hopper: slides linearly with intake pivot travel (slot-and-pin linkage, no sensor)
    double stowFraction =
        MathUtil.clamp(
            intakePivotAngle.get().in(Rotations)
                / IntakePivotConstants.SoftwareConstants.softwareUpperRotationLimit,
            0.0,
            1.0);
    componentPoses[3] = new Pose3d(HOPPER_STOWED_TRAVEL.times(stowFraction), Rotation3d.kZero);

    Logger.recordOutput("RobotModel/ComponentPoses", componentPoses);
  }
}
