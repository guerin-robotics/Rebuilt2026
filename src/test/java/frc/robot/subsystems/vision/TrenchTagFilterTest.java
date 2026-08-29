package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;
import static frc.robot.subsystems.vision.VisionConstants.isTrenchTag;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.vision.io.VisionIO.VisionIOInputs;
import frc.robot.subsystems.vision.io.VisionIOPhotonVision;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

/**
 * Verifies in PhotonVision simulation that trench tags never reach the pose estimator.
 *
 * <p>This drives the real production {@link VisionIOPhotonVision#updateInputs} against a simulated
 * camera looking at real tags from the configured field layout, so it exercises the same code path
 * the robot runs.
 *
 * <p>{@code latestTargetObservation} is computed BEFORE the trench filter runs, so a non-zero
 * target observation proves the camera genuinely saw the tag. An empty {@code poseObservations}
 * alongside it proves the filter is what suppressed the solve, rather than the camera having seen
 * nothing.
 */
public class TrenchTagFilterTest {
  private static VisionSystemSim visionSim;
  private static PhotonCamera camera;
  private static VisionIOPhotonVision io;

  // Camera at robot center, level, facing straight forward, at a realistic mounting height.
  // Height matters: at floor level most tags fall outside the vertical FOV at 2 m standoff.
  private static final Transform3d ROBOT_TO_CAMERA =
      new Transform3d(new Translation3d(0.0, 0.0, 0.6), new Rotation3d());

  /** Result of stepping the sim: what the camera saw and what survived the filter. */
  private record Observed(int poseObservations, Set<Integer> tagIds, boolean sawTarget) {}

  @BeforeAll
  static void setup() {
    PhotonCamera.setVersionCheckEnabled(false);

    visionSim = new VisionSystemSim("trenchTagFilterTest");
    camera = new PhotonCamera("TrenchFilterTestCam");
    PhotonCameraSim cameraSim =
        new PhotonCameraSim(camera, new SimCameraProperties(), aprilTagLayout);
    cameraSim.setMaxSightRange(10.0);
    visionSim.addCamera(cameraSim, ROBOT_TO_CAMERA);

    io = new VisionIOPhotonVision("TrenchFilterTestCam", ROBOT_TO_CAMERA);
  }

  @AfterAll
  static void teardown() {
    visionSim.clearCameras();
    camera.close();
  }

  /** Puts exactly the given tags on the simulated field, using their real layout poses. */
  private static void setFieldTags(int... tagIds) {
    visionSim.clearVisionTargets();
    List<VisionTargetSim> targets = new ArrayList<>();
    for (int id : tagIds) {
      targets.add(
          new VisionTargetSim(
              aprilTagLayout.getTagPose(id).orElseThrow(), TargetModel.kAprilTag36h11, id));
    }
    visionSim.addVisionTargets("apriltag", targets.toArray(new VisionTargetSim[0]));
  }

  /** A pose 2 m in front of the given tag, facing it. */
  private static Pose2d poseFacingTag(int tagId) {
    Pose3d tagPose = aprilTagLayout.getTagPose(tagId).orElseThrow();
    double tagYaw = tagPose.getRotation().getZ();
    return new Pose2d(
        tagPose
            .getTranslation()
            .toTranslation2d()
            .plus(new Translation2d(2.0, new Rotation2d(tagYaw))),
        new Rotation2d(tagYaw + Math.PI));
  }

  /**
   * Steps the sim at the given pose and reports what came through. Warm-up cycles are discarded so
   * a stale NetworkTables frame from a previous field configuration cannot leak into the
   * measurement.
   */
  private static Observed observe(Pose2d robotPose) {
    VisionIOInputs inputs = new VisionIOInputs();
    for (int i = 0; i < 6; i++) {
      visionSim.update(robotPose);
      io.updateInputs(inputs);
    }

    int poseObservations = 0;
    Set<Integer> tagIds = new LinkedHashSet<>();
    boolean sawTarget = false;
    for (int i = 0; i < 6; i++) {
      visionSim.update(robotPose);
      io.updateInputs(inputs);
      poseObservations += inputs.poseObservations.length;
      for (int id : inputs.tagIds) {
        tagIds.add(id);
      }
      sawTarget |=
          inputs.latestTargetObservation.tx().getRadians() != 0.0
              || inputs.latestTargetObservation.ty().getRadians() != 0.0;
    }
    return new Observed(poseObservations, tagIds, sawTarget);
  }

  private static List<Integer> layoutTagIds() {
    List<Integer> ids = new ArrayList<>();
    aprilTagLayout.getTags().forEach(tag -> ids.add(tag.ID));
    return ids;
  }

  @Test
  void trenchTagAloneProducesNoPoseObservation() {
    int checked = 0;
    for (int tagId : layoutTagIds()) {
      if (!isTrenchTag(tagId)) {
        continue;
      }
      setFieldTags(tagId);
      Observed o = observe(poseFacingTag(tagId));
      checked++;

      System.out.printf(
          "trench tag %2d alone: sawTarget=%-5s poseObservations=%d tagIds=%s%n",
          tagId, o.sawTarget(), o.poseObservations(), o.tagIds());

      assertTrue(o.sawTarget(), "camera should still SEE trench tag " + tagId);
      assertEquals(
          0, o.poseObservations(), "trench tag " + tagId + " must not produce a pose observation");
      assertEquals(
          Set.of(), o.tagIds(), "trench tag " + tagId + " must not appear in reported tag IDs");
    }
    assertTrue(checked > 0, "no trench tags found in the configured field layout");
    System.out.println("PASS: " + checked + " trench tags produced no pose observations");
  }

  @Test
  void nonTrenchTagAloneStillProducesPoseObservation() {
    int produced = 0;
    int checked = 0;
    for (int tagId : layoutTagIds()) {
      if (isTrenchTag(tagId)) {
        continue;
      }
      setFieldTags(tagId);
      Observed o = observe(poseFacingTag(tagId));
      checked++;
      if (o.poseObservations() > 0) {
        produced++;
        assertEquals(
            Set.of(tagId), o.tagIds(), "tag " + tagId + " should be the only reported tag ID");
      }
    }
    System.out.printf("PASS: non-trench tags produced poses for %d/%d tags%n", produced, checked);
    assertTrue(produced > 0, "no non-trench tag produced a pose — the filter rejects everything");
  }

  @Test
  void mixedFrameSolvesFromNonTrenchTagsOnly() {
    // The realistic case: the full field is present, so a camera pointed at a trench tag also
    // catches its non-trench neighbours. The solve must survive, sourced only from clean tags.
    visionSim.clearVisionTargets();
    visionSim.addAprilTags(aprilTagLayout);

    int mixedFrames = 0;
    for (int tagId : layoutTagIds()) {
      if (!isTrenchTag(tagId)) {
        continue;
      }
      Observed o = observe(poseFacingTag(tagId));

      System.out.printf(
          "full field, facing trench tag %2d: poseObservations=%d tagIds=%s%n",
          tagId, o.poseObservations(), o.tagIds());

      for (int id : o.tagIds()) {
        assertTrue(
            !isTrenchTag(id), "trench tag " + id + " leaked into the solve near tag " + tagId);
      }
      if (o.poseObservations() > 0) {
        mixedFrames++;
      }
    }
    System.out.printf(
        "PASS: %d trench-tag viewpoints still produced clean poses from neighbouring tags%n",
        mixedFrames);
  }
}
