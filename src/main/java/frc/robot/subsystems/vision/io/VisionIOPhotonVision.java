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
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

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
    Set<Short> multiTagIdsUsed = new HashSet<>();
    int multiTagSolvesDiscarded = 0;
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

      // --- Trench tag exclusion ---
      // Trench tags must not contribute to a pose estimate. This has to happen BEFORE the
      // estimator runs, because both strategies solve from the result we hand them.
      //
      // The two strategies need different treatment:
      //   - Single-tag fallback solves from result.targets, so dropping trench targets from
      //     that list is enough — the estimator then picks the best remaining tag.
      //   - Multi-tag PnP was already solved on the coprocessor, and estimateCoprocMultiTagPose()
      //     just reads that baked transform. Removing targets client-side cannot change it, so a
      //     trench tag inside a multi-tag solve can only be tolerated or the solve rejected
      //     wholesale. VisionConstants.minCleanTagsToKeepMultiTag decides which.
      if (result.getMultiTagResult().isPresent()) {
        List<Short> idsUsed = result.getMultiTagResult().get().fiducialIDsUsed;
        // Recorded before the filter so the log shows what the coprocessor actually solved from
        for (short id : idsUsed) {
          multiTagIdsUsed.add(id);
        }
        long cleanTags = idsUsed.stream().filter(id -> !isTrenchTag(id)).count();
        if (cleanTags < minCleanTagsToKeepMultiTag) {
          result.multitagResult = Optional.empty();
          multiTagSolvesDiscarded++;
        }
      }
      result.targets.removeIf(target -> isTrenchTag(target.fiducialId));

      // Every target in the frame was a trench tag — nothing left to estimate from
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

      // If neither strategy produced a result, skip this frame
      if (estimatedPose.isEmpty()) {
        continue;
      }

      EstimatedRobotPose estimate = estimatedPose.get();

      // Collect all tag IDs seen. estimate.targetsUsed is the filtered target list from
      // above, so trench tags are already gone.
      for (var target : estimate.targetsUsed) {
        if (target.fiducialId >= 0) {
          tagIds.add((short) target.fiducialId);
        }
      }

      // Determine ambiguity, tag count, and average distance for the solve.
      // CAUTION: estimateLowestAmbiguityPose() puts ALL targets in the frame into
      // estimate.targetsUsed even though its pose comes from a single tag, so the
      // single-tag fallback must recompute metadata from the one target actually used.
      double ambiguity;
      int tagCount;
      double avgTagDistance;
      if (usedMultiTag) {
        // Coprocessor multi-tag PnP used every target — near-zero ambiguity
        ambiguity = 0.0;
        tagCount = estimate.targetsUsed.size();
        double totalTagDistance = 0.0;
        for (var target : estimate.targetsUsed) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }
        avgTagDistance = tagCount == 0 ? 0.0 : totalTagDistance / tagCount;
      } else {
        // Find the lowest-ambiguity fiducial target — the one the fallback solve used
        PhotonTrackedTarget usedTarget = null;
        for (var target : estimate.targetsUsed) {
          if (target.poseAmbiguity != -1
              && (usedTarget == null || target.poseAmbiguity < usedTarget.poseAmbiguity)) {
            usedTarget = target;
          }
        }
        if (usedTarget == null) {
          continue; // No valid fiducial target — should not happen if a pose was produced
        }
        ambiguity = usedTarget.poseAmbiguity;
        tagCount = 1;
        avgTagDistance = usedTarget.bestCameraToTarget.getTranslation().getNorm();
      }

      // Add the observation for downstream filtering and pose fusion
      poseObservations.add(
          new PoseObservation(
              estimate.timestampSeconds,
              estimate.estimatedPose,
              ambiguity,
              tagCount,
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

    // Save multi-tag diagnostics. Always assigned, so a cycle with no multi-tag solve reports
    // empty rather than leaving the previous cycle's values in the log.
    inputs.multiTagFiducialIdsUsed = new int[multiTagIdsUsed.size()];
    int j = 0;
    for (int id : multiTagIdsUsed) {
      inputs.multiTagFiducialIdsUsed[j++] = id;
    }
    inputs.multiTagSolvesDiscarded = multiTagSolvesDiscarded;
  }
}
