// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.Logger;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "SwerveRight";
  public static String camera1Name = "SwerveLeft";
  public static String camera2Name = "ShooterRight";
  public static String camera3Name = "ShooterLeft";

  // Right Cam
  // 10.75 x
  // 4.75
  // 8.75 y

  public static final Transform3d robotToCamera0 =
      new Transform3d(
          new Translation3d(Inches.of(-11.859), Inches.of(-11.508), Inches.of(8.605)),
          new Rotation3d(Radians.zero(), Degrees.of(-20), Degrees.of(210)));

  // Left Cam
  public static final Transform3d robotToCamera1 =
      new Transform3d(
          new Translation3d(Inches.of(-11.859), Inches.of(11.508), Inches.of(8.605)),
          new Rotation3d(Radians.zero(), Degrees.of(-20), Degrees.of(150)));

  // Shooter right
  public static final Transform3d robotToCamera2 =
      new Transform3d(
          new Translation3d(Inches.of(3.945), Inches.of(-1.417), Inches.of(19.233388)),
          new Rotation3d(Radians.zero(), Degrees.of(-10), Degrees.of(-40)));

  // Shooter left
  public static final Transform3d robotToCamera3 =
      new Transform3d(
          new Translation3d(Inches.of(3.945), Inches.of(1.417), Inches.of(19.233388)),
          new Rotation3d(Radians.zero(), Degrees.of(-10), Degrees.of(40)));

  // new Rotation3d(Radians.zero(), Radians.of(-0.314159), Radians.of((2 * Math.PI) / 3)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.2;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        10.0, // Camera 0
        10.0, // Camera 1
        20.0, // Camera 2
        20.0 // Camera 3
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  // Logging
  static {
    Logger.recordOutput("Vision/Camera0/name", VisionConstants.camera0Name);
    Logger.recordOutput("Vision/Camera0/robot_position", VisionConstants.robotToCamera0);
    Logger.recordOutput("Vision/Camera1/name", VisionConstants.camera1Name);
    Logger.recordOutput("Vision/Camera1/robot_position", VisionConstants.robotToCamera1);
    Logger.recordOutput("Vision/Camera2/name", VisionConstants.camera2Name);
    Logger.recordOutput("Vision/Camera2/robot_position", VisionConstants.robotToCamera2);
    Logger.recordOutput("Vision/Camera3/name", VisionConstants.camera3Name);
    Logger.recordOutput("Vision/Camera3/robot_position", VisionConstants.robotToCamera3);
  }
}
