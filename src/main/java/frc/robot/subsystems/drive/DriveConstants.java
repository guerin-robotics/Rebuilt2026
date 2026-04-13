package frc.robot.subsystems.drive;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.geometry.Translation2d;

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
}
