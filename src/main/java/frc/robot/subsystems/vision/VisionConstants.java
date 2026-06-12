// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.Logger;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded); // .k2026RebuiltAndymark

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "RobotRight";
  public static String camera1Name = "RobotLeft";
  public static String camera2Name = "ShooterRight";
  public static String camera3Name = "ShooterLeft";

  // Robot-to-camera transforms (values from robot-spec).
  // WPILib conventions: X forward, Y left, Z up (Inches.of(...) converts to meters).
  // Rotation3d argument order is (roll, pitch, yaw).
  // Pitch is rotation about +Y: NEGATIVE pitch tilts the camera UPWARD.
  // Yaw is rotation about +Z, counterclockwise positive (90 = left, 270/-90 = right).

  // RobotRight: front-right of robot, facing right (yaw 270), tilted up 15 deg
  public static final Transform3d robotToCamera0 =
      new Transform3d(
          new Translation3d(Inches.of(1.0), Inches.of(-12.171), Inches.of(6.438)),
          new Rotation3d(Degrees.of(0.0), Degrees.of(-15.0), Degrees.of(270.0)));

  // RobotLeft: front-left of robot, facing left (yaw 90), tilted up 15 deg
  public static final Transform3d robotToCamera1 =
      new Transform3d(
          new Translation3d(Inches.of(1.0), Inches.of(12.421), Inches.of(6.438)),
          new Rotation3d(Degrees.of(0.0), Degrees.of(-15.0), Degrees.of(90.0)));

  // ShooterRight: rear-right of robot, facing backward (yaw 180), tilted up 15 deg
  public static final Transform3d robotToCamera2 =
      new Transform3d(
          new Translation3d(Inches.of(-12.572), Inches.of(-6.125), Inches.of(12.509)),
          new Rotation3d(Degrees.of(0.0), Degrees.of(-15.0), Degrees.of(180)));

  // ShooterLeft: rear-left of robot, facing backward (yaw 180), tilted up 15 deg
  public static final Transform3d robotToCamera3 =
      new Transform3d(
          new Translation3d(Inches.of(-12.572), Inches.of(5.375), Inches.of(12.509)),
          new Rotation3d(Degrees.of(0.0), Degrees.of(-15.0), Degrees.of(180)));

  // ---- Filtering thresholds ----

  // Single-tag ambiguity above this is rejected (multi-tag is always trusted).
  public static double maxAmbiguity = 0.4;

  // Estimated pose Z (height) must be below this to be realistic
  public static double maxZError = 2;

  // Not below the floor
  public static double floorError = inchesToMeters(6);

  // Tags farther than this are unreliable — reject the observation entirely.
  // At long range, small pixel errors become large pose errors.
  public static double maxDistanceMeters = 6.0; // 4.0

  // If the robot is spinning faster than this (rad/s), vision is unreliable
  // because motion blur and timestamp misalignment degrade the estimate.
  public static double maxAngularVelocityRadPerSec = 4.0; // 2.0

  // Maximum pitch or roll (radians) allowed in an estimated pose.
  // A real robot on flat carpet should never be tilted more than ~10°.
  // Large pitch/roll in the estimate means the solve is wrong.
  public static double maxPitchRollRadians = Math.toRadians(25.0); // 10.0

  // If a vision observation would move the estimated pose by more than this
  // distance (meters), reject it as an outlier. Catches bad single-tag solves
  // that pick the wrong PnP ambiguity solution.
  // public static double maxPoseJumpMeters = 10; // ~20 inches

  // ---- Standard deviation baselines ----
  // For 1 meter distance and 1 tag. Automatically scaled by distance² / tagCount.
  public static double linearStdDevBaseline = 0.01; // Meters
  public static double angularStdDevBaseline = 0.03; // Radians

  // Extra multiplier applied to single-tag observations (tagCount == 1).
  // Single-tag PnP is inherently less constrained than multi-tag, so we
  // trust it less. This prevents a single ambiguous solve from yanking the
  // pose estimator.
  public static double singleTagStdDevMultiplier = 2.0;

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.0, // Camera 2
        1.0 // Camera 3
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
