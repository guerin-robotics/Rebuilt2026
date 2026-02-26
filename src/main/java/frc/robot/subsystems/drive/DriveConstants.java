package frc.robot.subsystems.drive;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.geometry.Translation2d;

public class DriveConstants {
  public static double frontOffset = inchesToMeters(11);
  public static double leftOffset = inchesToMeters(11);
  public static Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(leftOffset, frontOffset),
        new Translation2d(leftOffset, frontOffset),
        new Translation2d(leftOffset, frontOffset),
        new Translation2d(leftOffset, frontOffset)
      };
}
