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

      // Latest per-camera debug values for logging (last observation processed wins)
      String lastRejectionReason = "";
      double lastAmbiguity = 0.0;
      double lastAverageTagDistance = 0.0;
      int lastTagCount = 0;
      double lastLinearStdDev = 0.0;
      double lastAngularStdDev = 0.0;

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {

        // Extract pitch and roll from the estimated pose rotation
        double pitch = Math.abs(observation.pose().getRotation().getY());
        double roll = Math.abs(observation.pose().getRotation().getX());

        // Determine rejection reason ("" = accepted). Checked in priority order.
        String rejectionReason = "";
        if (robotSpinningTooFast) {
          rejectionReason = "AngularVelocityTooHigh"; // Vision unreliable while spinning
        } else if (observation.tagCount() == 0) {
          rejectionReason = "NoTags"; // Must have at least one tag
        } else if (observation.timestamp() <= 0.0) {
          rejectionReason = "InvalidTimestamp"; // Timestamp must be positive
        } else if (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity) {
          rejectionReason = "HighAmbiguity"; // Single-tag solve picked between two solutions
        } else if (observation.pose().getZ() < -floorError) {
          rejectionReason = "BelowFloor"; // Robot cannot be below the floor
        } else if (observation.pose().getZ() > maxZError) {
          rejectionReason = "ZTooHigh"; // Must have realistic Z coordinate
        } else if (observation.averageTagDistance() > maxDistanceMeters) {
          rejectionReason = "TagsTooFar"; // Pose error grows with distance
        } else if (pitch > maxPitchRollRadians || roll > maxPitchRollRadians) {
          rejectionReason = "PitchRollTooLarge"; // Robot is on flat ground — solve is wrong
        } else if (observation.pose().getX() < 0.0
            || observation.pose().getX() > aprilTagLayout.getFieldLength()
            || observation.pose().getY() < 0.0
            || observation.pose().getY() > aprilTagLayout.getFieldWidth()) {
          rejectionReason = "OutsideField"; // Must be within the field boundaries
        }
        boolean rejectPose = !rejectionReason.isEmpty();

        lastRejectionReason = rejectionReason;
        lastAmbiguity = observation.ambiguity();
        lastAverageTagDistance = observation.averageTagDistance();
        lastTagCount = observation.tagCount();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
          continue;
        }
        robotPosesAccepted.add(observation.pose());

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
        lastLinearStdDev = linearStdDev;
        lastAngularStdDev = angularStdDev;

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
      Logger.recordOutput(
          cameraKey + "/RobotPosesAccepted", robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          cameraKey + "/RobotPosesRejected", robotPosesRejected.toArray(new Pose3d[0]));
      Logger.recordOutput(cameraKey + "/TagIds", inputs[cameraIndex].tagIds);
      Logger.recordOutput(cameraKey + "/TagCount", lastTagCount);
      Logger.recordOutput(cameraKey + "/IsMultiTag", lastTagCount > 1);
      Logger.recordOutput(cameraKey + "/RejectionReason", lastRejectionReason);
      Logger.recordOutput(cameraKey + "/Ambiguity", lastAmbiguity);
      Logger.recordOutput(cameraKey + "/AverageTagDistance", lastAverageTagDistance);
      Logger.recordOutput(cameraKey + "/LinearStdDev", lastLinearStdDev);
      Logger.recordOutput(cameraKey + "/AngularStdDev", lastAngularStdDev);
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
    Logger.recordOutput("Vision/Summary/AcceptedObservationCount", allRobotPosesAccepted.size());
    Logger.recordOutput("Vision/Summary/RejectedObservationCount", allRobotPosesRejected.size());
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
