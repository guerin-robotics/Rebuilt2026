package frc.robot.subsystems.drive;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public class DriveConstants {
  public static double frontOffset = inchesToMeters(11);
  public static double leftOffset = inchesToMeters(11);
  // Module translations: FL (+X, +Y), FR (+X, -Y), BL (-X, +Y), BR (-X, -Y)
  // These must match the positions in TunerConstants (ALPHA/COMP).
  public static Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(frontOffset, leftOffset), // Front Left
        new Translation2d(frontOffset, -leftOffset), // Front Right
        new Translation2d(-frontOffset, leftOffset), // Back Left
        new Translation2d(-frontOffset, -leftOffset) // Back Right
      };

  public static double limitedVelo = 0.1;

  public static AngularVelocity maxModuleRotationSpeed = RadiansPerSecond.of(12);
  public static final int numberOfSwerveModules = 4;

  // Precomputed headings for an 'X' braking formation: FL=45, FR=-45, BL=-45, BR=45
  public static final Rotation2d[] xHeadings;

  static {
    xHeadings = new Rotation2d[numberOfSwerveModules];
    // Pattern to create an X relative to each module's translation angle.
    // FL/FR/BL/BR -> signs + - - + (matching earlier hardcoded values)
    int[] signs = new int[] {1, -1, -1, 1};
    for (int i = 0; i < numberOfSwerveModules; i++) {
      Rotation2d base = moduleTranslations[i].getAngle();
      xHeadings[i] = base.plus(Rotation2d.fromDegrees(45.0 * signs[i]));
    }
  }
}
