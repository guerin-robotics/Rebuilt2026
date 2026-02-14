// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

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
  public static String camera0Name = "EagleEyeRight";
  public static String camera1Name = "EagleEyeLeft";

  // Right Cam
  // 10.75 x
  // 4.75
  // 8.75 y
  public static final Transform3d robotToCamera0 =
      new Transform3d(
          new Translation3d(Inches.of(10.75), Inches.of(-8.75), Inches.of(7.75)),
          new Rotation3d(Radians.zero(), Radians.of(-0.3490659), Radians.of(0.1745329)));

  // Left Cam
  public static final Transform3d robotToCamera1 =
      new Transform3d(
          new Translation3d(Inches.of(10.75), Inches.of(8.75), Inches.of(7.75)),
          new Rotation3d(Radians.zero(), Radians.of(-0.3490659), Radians.of(-0.1745329)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
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
  }
}
