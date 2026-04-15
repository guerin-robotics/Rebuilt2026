// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision.io;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

/**
 * IO implementation for real PhotonVision hardware.
 *
 * <p>Uses PhotonVision's {@link PhotonPoseEstimator} to compute robot pose from AprilTag
 * detections. The estimator tries the coprocessor multi-tag solve first (most accurate when
 * multiple tags are visible), then falls back to the single-tag lowest-ambiguity strategy.
 *
 * <p><b>Change log (2026-03-20):</b> Replaced manual transform-chain math with {@link
 * PhotonPoseEstimator}. The estimator handles the robotToCamera transform internally, reducing the
 * chance of transform-chain bugs. Multi-tag results are preferred, with a fallback to
 * lowest-ambiguity single-tag when multi-tag is unavailable.
 */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  protected final PhotonPoseEstimator poseEstimator;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;

    // Create a PhotonPoseEstimator using the non-deprecated 2-arg constructor.
    // We call estimateCoprocMultiTagPose() and estimateLowestAmbiguityPose()
    // individually below instead of relying on the deprecated strategy-based update().
    this.poseEstimator = new PhotonPoseEstimator(aprilTagLayout, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation (used for simple target-tracking, not pose estimation)
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
      }

      // Skip results with no targets — nothing to estimate from
      if (!result.hasTargets()) {
        continue;
      }

      // --- Pose estimation using PhotonPoseEstimator ---
      // Strategy: try multi-tag first (more accurate), fall back to lowest-ambiguity single-tag.
      // estimateCoprocMultiTagPose() uses the coprocessor's multi-tag PnP solve when available.
      // estimateLowestAmbiguityPose() picks the single target with the lowest ambiguity.
      Optional<EstimatedRobotPose> estimatedPose = poseEstimator.estimateCoprocMultiTagPose(result);
      boolean usedMultiTag = estimatedPose.isPresent();
      if (estimatedPose.isEmpty()) {
        estimatedPose = poseEstimator.estimateLowestAmbiguityPose(result);
      }
      Logger.recordOutput("Vision/" + camera.getName() + "/UsedMultiTag", usedMultiTag);

      // If neither strategy produced a result, skip this frame
      if (estimatedPose.isEmpty()) {
        continue;
      }

      EstimatedRobotPose estimate = estimatedPose.get();

      // Calculate average tag distance (used for standard deviation scaling)
      double totalTagDistance = 0.0;
      for (var target : estimate.targetsUsed) {
        totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
      }
      double avgTagDistance =
          estimate.targetsUsed.isEmpty() ? 0.0 : totalTagDistance / estimate.targetsUsed.size();

      // Collect all tag IDs seen
      for (var target : estimate.targetsUsed) {
        tagIds.add((short) target.fiducialId);
      }

      // Determine ambiguity — multi-tag results have near-zero ambiguity,
      // single-tag results use the target's reported ambiguity.
      double ambiguity =
          estimate.targetsUsed.size() > 1
              ? 0.0
              : (estimate.targetsUsed.isEmpty() ? 1.0 : estimate.targetsUsed.get(0).poseAmbiguity);
      Logger.recordOutput("Vision/" + camera.getName() + "/Ambiguity", ambiguity);

      // Add the observation for downstream filtering and pose fusion
      poseObservations.add(
          new PoseObservation(
              estimate.timestampSeconds,
              estimate.estimatedPose,
              ambiguity,
              estimate.targetsUsed.size(),
              avgTagDistance,
              PoseObservationType.PHOTONVISION));
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs object
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
