// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.io.VisionIO;
import frc.robot.subsystems.vision.io.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.io.VisionIOInputsAutoLogged;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Vision subsystem that processes AprilTag observations from multiple cameras and feeds validated
 * pose estimates into the drivetrain's pose estimator.
 *
 * <p><b>Change log (2026-03-20):</b> Added three new rejection filters
 *
 * <ul>
 *   <li><b>Average tag distance filter</b> — observations where all tags are farther than {@link
 *       VisionConstants#maxDistanceMeters} are rejected because pixel errors grow with distance.
 *   <li><b>Angular velocity filter</b> — all observations are rejected when the robot is spinning
 *       faster than {@link VisionConstants#maxAngularVelocityRadPerSec} because motion blur and
 *       timestamp misalignment degrade accuracy.
 *   <li><b>Pitch/roll filter</b> — estimated poses with pitch or roll larger than {@link
 *       VisionConstants#maxPitchRollRadians} are rejected because a robot on flat carpet should
 *       never be significantly tilted; large tilt means the solve is wrong.
 * </ul>
 */
public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // --- Angular velocity pre-filter ---
    // If the robot is spinning fast, vision estimates are unreliable due to
    // motion blur and timestamp misalignment. Reject ALL observations this cycle.
    boolean robotSpinningTooFast =
        Math.abs(RobotState.getInstance().getFieldRelativeVelocity().omegaRadiansPerSecond)
            > maxAngularVelocityRadPerSec;

    // Initialize logging values — ArrayList is faster than LinkedList for iteration and toArray()
    List<Pose3d> allTagPoses = new ArrayList<>();
    List<Pose3d> allRobotPoses = new ArrayList<>();
    List<Pose3d> allRobotPosesAccepted = new ArrayList<>();
    List<Pose3d> allRobotPosesRejected = new ArrayList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values — ArrayList is faster than LinkedList for iteration and toArray()
      List<Pose3d> tagPoses = new ArrayList<>(inputs[cameraIndex].tagIds.length);
      List<Pose3d> robotPoses = new ArrayList<>(inputs[cameraIndex].poseObservations.length);
      List<Pose3d> robotPosesAccepted =
          new ArrayList<>(inputs[cameraIndex].poseObservations.length);
      List<Pose3d> robotPosesRejected = new ArrayList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {

        // Extract pitch and roll from the estimated pose rotation
        double pitch = Math.abs(observation.pose().getRotation().getY());
        double roll = Math.abs(observation.pose().getRotation().getX());

        // Pose-jump filter: reject if the vision observation is too far from
        // the current estimated pose. Catches bad single-tag PnP solutions.
        Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
        double poseJump =
            currentPose.getTranslation().getDistance(observation.pose().toPose2d().getTranslation());

        // Check whether to reject pose
        boolean rejectPose =
            robotSpinningTooFast // Robot spinning too fast — vision unreliable
                || observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || observation.pose().getZ() < -0.1 // Robot cannot be below the floor
                || observation.pose().getZ()
                    > maxZError // Must have realistic Z coordinate
                || observation.averageTagDistance()
                    > maxDistanceMeters // Tags too far away — pose error grows with distance
                || pitch > maxPitchRollRadians // Pitch too large — robot is on flat ground
                || roll > maxPitchRollRadians // Roll too large — robot is on flat ground
                || poseJump > maxPoseJumpMeters // Vision would yank pose too far — likely bad solve

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (observation.tagCount() == 1) {
          linearStdDev *= singleTagStdDevMultiplier;
          angularStdDev *= singleTagStdDevMultiplier;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera metadata
      String cameraKey = "Vision/Camera" + Integer.toString(cameraIndex);
      Logger.recordOutput(cameraKey + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(cameraKey + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(cameraKey + "/RobotPosesAccepted", robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(cameraKey + "/RobotPosesRejected", robotPosesRejected.toArray(new Pose3d[0]));
      Logger.recordOutput(cameraKey + "/TagCount", inputs[cameraIndex].tagIds.length);
      Logger.recordOutput(cameraKey + "/IsMultiTag", inputs[cameraIndex].tagIds.length > 1);
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
