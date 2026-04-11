package frc.robot.commands.autos;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.autos.utils.AutoContext;
import frc.robot.commands.autos.utils.AutoOption;
import java.util.Arrays;
import java.util.List;

/**
 * Contains all autonomous routines built with the Choreo {@link AutoRoutine} API.
 *
 * <p>Each public method returns an {@link AutoOption} that wraps:
 *
 * <ul>
 *   <li>A {@code Supplier<Command>} — lazily builds the routine's command when the match starts
 *   <li>Preview poses — drawn on the dashboard field so the drive team can see the planned path
 *   <li>A starting pose — used to verify the robot is placed correctly before the match
 * </ul>
 *
 * <p><b>How to add a new auto:</b>
 *
 * <ol>
 *   <li>Create trajectory(s) in the Choreo application with event markers (e.g. "deployIntake",
 *       "shootSequence")
 *   <li>Write a new method here that loads those trajectories, binds logic to event markers, and
 *       returns an {@link AutoOption}
 *   <li>Register it in {@code RobotContainer} by adding it to the auto chooser
 * </ol>
 */
public class AutoPaths {

  // ─── private constructor ─ utility class ────────────────────────────────────
  private AutoPaths() {}

  // ═══════════════════════════════════════════════════════════════════════════
  //  Sample Auto 1 — "Score and Pickup"
  //
  //  Trajectory: "ScoreAndPickup" (monolithic or segmented in Choreo)
  //  Event markers expected in the .traj file:
  //    • "deployIntake"   — triggers intake deployment mid-path
  //    • "shootSequence"  — triggers the full shoot sequence at path end
  // ═══════════════════════════════════════════════════════════════════════════

  /**
   * Builds the "Score and Pickup" auto option.
   *
   * <p>The routine:
   *
   * <ol>
   *   <li>Resets odometry to the trajectory start
   *   <li>Follows the "ScoreAndPickup" trajectory
   *   <li>At the "deployIntake" event marker, deploys the intake and runs rollers
   *   <li>When the trajectory finishes, executes a full shoot sequence, then stops all subsystems
   * </ol>
   */
  public static AutoOption scoreAndPickup(AutoContext ctx) {
    // ── Build the routine ────────────────────────────────────────────────────
    AutoRoutine routine = ctx.autoFactory().newRoutine("ScoreAndPickup");
    AutoTrajectory trajectory = routine.trajectory("ScoreAndPickup");

    // When the routine starts, reset odometry then follow the trajectory
    routine.active().onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));

    // At the "deployIntake" event marker, deploy the intake and run rollers + transport
    trajectory.atTime("deployIntake").onTrue(AutoCommands.deployAndRunIntake(ctx));

    // When the trajectory is done, run the shoot sequence then stop everything
    trajectory
        .done()
        .onTrue(
            AutoCommands.shootSequence(ctx).withTimeout(3.0).andThen(AutoCommands.stopAll(ctx)));

    // ── Build preview data from the trajectory poses ─────────────────────────
    List<Pose2d> previewPoses = getTrajectoryPoses(trajectory);
    Pose2d startPose = getStartingPose(trajectory);

    return new AutoOption(routine::cmd, previewPoses, startPose);
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  Sample Auto 2 — "Two Piece"
  //
  //  Trajectories: "TwoPiece.1" and "TwoPiece.2" (split segments in Choreo)
  //  Event markers expected:
  //    • "deployIntake"   — on the first segment, triggers intake deployment
  //    • "shootSequence"  — on the second segment end, triggers shoot sequence
  // ═══════════════════════════════════════════════════════════════════════════

  /**
   * Builds the "Two Piece" auto option.
   *
   * <p>The routine:
   *
   * <ol>
   *   <li>Resets odometry to the first segment's start
   *   <li>Follows "TwoPiece.1" (drive to pickup)
   *   <li>At the "deployIntake" event marker, deploys the intake and runs rollers
   *   <li>When the first segment finishes, starts "TwoPiece.2" (drive back to score)
   *   <li>When the second segment finishes, runs the full shoot sequence then stops all
   * </ol>
   */
  public static AutoOption leftAuto(AutoContext ctx) {
    // ── Build the routine ────────────────────────────────────────────────────
    AutoRoutine routine = ctx.autoFactory().newRoutine("LeftAuto");
    AutoTrajectory path1Traj = routine.trajectory("LeftAuto1", 0);
    AutoTrajectory path2Traj = routine.trajectory("LeftAuto2", 1);

    // When the routine starts, reset odometry then follow the first trajectory
    routine.active().onTrue(Commands.sequence(path1Traj.resetOdometry(), path1Traj.spawnCmd()));

    // At the "deployIntake" event marker, deploy intake and run rollers + transport
    //path1Traj.atTime("deployIntake").onTrue(AutoCommands.deployAndRunIntake(ctx));

    // When the first trajectory finishes, retract intake and start the second trajectory
    path1Traj.done().onTrue(path2Traj.cmd());
    // Commands.sequence(AutoCommands.retractIntake(ctx), path2Traj.cmd());

    // When the second trajectory finishes, run the shoot sequence then stop everything
    path2Traj
        .done()
        .onTrue(
            AutoCommands.shootSequence(ctx).withTimeout(3.0).andThen(AutoCommands.stopAll(ctx)));

    // ── Build preview data from both trajectory segments ─────────────────────
    List<Pose2d> previewPoses = new java.util.ArrayList<>();
    previewPoses.addAll(getTrajectoryPoses(path1Traj));
    previewPoses.addAll(getTrajectoryPoses(path2Traj));
    Pose2d startPose = getStartingPose(path1Traj);

    return new AutoOption(routine::cmd, previewPoses, startPose);
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  Helper Methods
  // ═══════════════════════════════════════════════════════════════════════════

  /**
   * Extracts the list of poses from a Choreo trajectory for dashboard preview.
   *
   * @param trajectory The AutoTrajectory to extract poses from
   * @return A list of Pose2d along the trajectory, or an empty list if unavailable
   */
  private static List<Pose2d> getTrajectoryPoses(AutoTrajectory trajectory) {
    // getRawTrajectory() returns a Trajectory<SampleType> directly (not an Optional).
    // getPoses() returns a Pose2d[] of all sampled poses along the path.
    try {
      Pose2d[] poses = trajectory.<SwerveSample>getRawTrajectory().getPoses();
      return Arrays.asList(poses);
    } catch (Exception e) {
      return List.of();
    }
  }

  /**
   * Extracts the starting pose from a Choreo trajectory.
   *
   * <p>Uses AutoTrajectory.getInitialPose() which automatically handles alliance flipping.
   *
   * @param trajectory The AutoTrajectory to extract the start pose from
   * @return The initial Pose2d of the trajectory, or a default Pose2d if unavailable
   */
  private static Pose2d getStartingPose(AutoTrajectory trajectory) {
    return trajectory.getInitialPose().orElse(new Pose2d());
  }
}
