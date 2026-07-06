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
    AutoTrajectory path1Traj = routine.trajectory("LeftAuto1");
    AutoTrajectory path2Traj = routine.trajectory("LeftAuto2");

    // When the routine starts, reset odometry then follow the first trajectory.
    // Use cmd() (not spawnCmd()) so the trajectory command holds the drive subsystem
    // and blocks until it finishes. This ensures path1's done() trigger fires correctly
    // and that the drive subsystem is available for path2 to claim immediately after.
    routine.active().onTrue(Commands.sequence(path1Traj.resetOdometry(), path1Traj.cmd()));

    // At the "deployIntake" event marker, deploy intake and run rollers + transport
    path1Traj.atTime("deployIntake").onTrue(AutoCommands.deployAndRunIntake(ctx));

    // At the end of the first trajectory, run the shoot sequence and stop everything
    path1Traj
        .done()
        .onTrue(
            AutoCommands.shootSequence(ctx)
                .withTimeout(3)
                .andThen(AutoCommands.stopAll(ctx))
                .andThen(path2Traj.cmd()));

    // When the second trajectory finishes, run the shoot sequence then stop everything
    path2Traj
        .done()
        .onTrue(AutoCommands.shootSequence(ctx).withTimeout(3).andThen(AutoCommands.stopAll(ctx)));

    // ── Build preview data from both trajectory segments ─────────────────────
    List<Pose2d> previewPoses = new java.util.ArrayList<>();
    previewPoses.addAll(getTrajectoryPoses(path1Traj));
    previewPoses.addAll(getTrajectoryPoses(path2Traj));
    Pose2d startPose = getStartingPose(path1Traj);

    return new AutoOption(routine::cmd, previewPoses, startPose);
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  Autos converted from PathPlanner (generated from the .auto files)
  //
  //  Each method replicates its PathPlanner auto's command tree:
  //  drive path -> race(Shoot, wait) -> stopAll -> DeployIntake -> next path.
  //  DeployIntake / RunIntake event markers from the .path files are bound via
  //  AutoCommands.bindMarkers().
  // ═══════════════════════════════════════════════════════════════════════════

  /** Converted from PathPlanner auto "2.5-Left-Comp". */
  public static AutoOption leftComp25(AutoContext ctx) {
    AutoRoutine routine = ctx.autoFactory().newRoutine("2.5-Left-Comp");
    AutoTrajectory t0 = routine.trajectory("Left-Comp-Path-1");
    AutoTrajectory t1 = routine.trajectory("Left-Comp-Path-2");
    AutoTrajectory t2 = routine.trajectory("Left-Comp-Path-3");

    AutoCommands.bindMarkers(t0, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");
    AutoCommands.bindMarkers(t1, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");
    AutoCommands.bindMarkers(t2, ctx, "DeployIntake", "RunIntakeStart");

    routine.active().onTrue(Commands.sequence(t0.resetOdometry(), t0.cmd()));
    t0.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 2.7),
                AutoCommands.stopShooting(ctx),
                AutoCommands.deployIntakeDown(ctx),
                t1.cmd()));
    t1.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 3.0),
                AutoCommands.stopShooting(ctx),
                AutoCommands.deployIntakeDown(ctx),
                t2.cmd()));

    List<Pose2d> previewPoses = new java.util.ArrayList<>();
    previewPoses.addAll(getTrajectoryPoses(t0));
    previewPoses.addAll(getTrajectoryPoses(t1));
    previewPoses.addAll(getTrajectoryPoses(t2));
    return new AutoOption(routine::cmd, previewPoses, getStartingPose(t0));
  }

  /** Converted from PathPlanner auto "2.5-Right-Comp". */
  public static AutoOption rightComp25(AutoContext ctx) {
    AutoRoutine routine = ctx.autoFactory().newRoutine("2.5-Right-Comp");
    AutoTrajectory t0 = routine.trajectory("Right-Comp-Path-1");
    AutoTrajectory t1 = routine.trajectory("Right-Comp-Path-2");
    AutoTrajectory t2 = routine.trajectory("Right-Comp-Path-3");

    AutoCommands.bindMarkers(t0, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");
    AutoCommands.bindMarkers(t1, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");

    routine.active().onTrue(Commands.sequence(t0.resetOdometry(), t0.cmd()));
    t0.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 2.7),
                AutoCommands.stopShooting(ctx),
                AutoCommands.deployIntakeDown(ctx),
                t1.cmd()));
    t1.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 3.0),
                AutoCommands.stopShooting(ctx),
                AutoCommands.deployIntakeDown(ctx),
                t2.cmd()));

    List<Pose2d> previewPoses = new java.util.ArrayList<>();
    previewPoses.addAll(getTrajectoryPoses(t0));
    previewPoses.addAll(getTrajectoryPoses(t1));
    previewPoses.addAll(getTrajectoryPoses(t2));
    return new AutoOption(routine::cmd, previewPoses, getStartingPose(t0));
  }

  /** Converted from PathPlanner auto "Center-Bump". */
  public static AutoOption centerBump(AutoContext ctx) {
    AutoRoutine routine = ctx.autoFactory().newRoutine("Center-Bump");
    AutoTrajectory t0 = routine.trajectory("Center-Path");

    AutoCommands.bindMarkers(t0, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");

    routine.active().onTrue(Commands.sequence(t0.resetOdometry(), t0.cmd()));
    t0.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 4.5), AutoCommands.stopShooting(ctx)));

    List<Pose2d> previewPoses = new java.util.ArrayList<>();
    previewPoses.addAll(getTrajectoryPoses(t0));
    return new AutoOption(routine::cmd, previewPoses, getStartingPose(t0));
  }

  /** Converted from PathPlanner auto "Champs-Follow-Left". */
  public static AutoOption champsFollowLeft(AutoContext ctx) {
    AutoRoutine routine = ctx.autoFactory().newRoutine("Champs-Follow-Left");
    AutoTrajectory t0 = routine.trajectory("Champs-Center-Left-1");

    AutoCommands.bindMarkers(t0, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");

    routine.active().onTrue(Commands.sequence(t0.resetOdometry(), t0.cmd()));
    t0.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 4.5),
                AutoCommands.stopShooting(ctx),
                AutoCommands.deployIntakeDown(ctx)));

    List<Pose2d> previewPoses = new java.util.ArrayList<>();
    previewPoses.addAll(getTrajectoryPoses(t0));
    return new AutoOption(routine::cmd, previewPoses, getStartingPose(t0));
  }

  /** Converted from PathPlanner auto "Champs-Follow-Right". */
  public static AutoOption champsFollowRight(AutoContext ctx) {
    AutoRoutine routine = ctx.autoFactory().newRoutine("Champs-Follow-Right");
    AutoTrajectory t0 = routine.trajectory("Champs-Center-Right-1");

    AutoCommands.bindMarkers(t0, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");

    routine.active().onTrue(Commands.sequence(t0.resetOdometry(), t0.cmd()));
    t0.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 4.5),
                AutoCommands.stopShooting(ctx),
                AutoCommands.deployIntakeDown(ctx)));

    List<Pose2d> previewPoses = new java.util.ArrayList<>();
    previewPoses.addAll(getTrajectoryPoses(t0));
    return new AutoOption(routine::cmd, previewPoses, getStartingPose(t0));
  }

  /** Converted from PathPlanner auto "Champs-Left". */
  public static AutoOption champsLeft(AutoContext ctx) {
    AutoRoutine routine = ctx.autoFactory().newRoutine("Champs-Left");
    AutoTrajectory t0 = routine.trajectory("Champs-Left-1");
    AutoTrajectory t1 = routine.trajectory("Left-Comp-Path-2");

    AutoCommands.bindMarkers(t0, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");
    AutoCommands.bindMarkers(t1, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");

    routine.active().onTrue(Commands.sequence(t0.resetOdometry(), t0.cmd()));
    t0.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 3.25),
                AutoCommands.stopShooting(ctx),
                AutoCommands.deployIntakeDown(ctx),
                t1.cmd()));
    t1.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 3.25),
                AutoCommands.stopShooting(ctx),
                AutoCommands.deployIntakeDown(ctx)));

    List<Pose2d> previewPoses = new java.util.ArrayList<>();
    previewPoses.addAll(getTrajectoryPoses(t0));
    previewPoses.addAll(getTrajectoryPoses(t1));
    return new AutoOption(routine::cmd, previewPoses, getStartingPose(t0));
  }

  /** Converted from PathPlanner auto "Champs-Right". */
  public static AutoOption champsRight(AutoContext ctx) {
    AutoRoutine routine = ctx.autoFactory().newRoutine("Champs-Right");
    AutoTrajectory t0 = routine.trajectory("Champs-Right-1");
    AutoTrajectory t1 = routine.trajectory("Right-Comp-Path-2");

    AutoCommands.bindMarkers(t0, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");
    AutoCommands.bindMarkers(t1, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");

    routine.active().onTrue(Commands.sequence(t0.resetOdometry(), t0.cmd()));
    t0.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 3.5),
                AutoCommands.deployIntakeDown(ctx),
                AutoCommands.stopShooting(ctx),
                t1.cmd()));
    t1.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 3.5),
                AutoCommands.deployIntakeDown(ctx),
                AutoCommands.stopShooting(ctx)));

    List<Pose2d> previewPoses = new java.util.ArrayList<>();
    previewPoses.addAll(getTrajectoryPoses(t0));
    previewPoses.addAll(getTrajectoryPoses(t1));
    return new AutoOption(routine::cmd, previewPoses, getStartingPose(t0));
  }

  /** Converted from PathPlanner auto "Champs-Safe-Left". */
  public static AutoOption champsSafeLeft(AutoContext ctx) {
    AutoRoutine routine = ctx.autoFactory().newRoutine("Champs-Safe-Left");
    AutoTrajectory t0 = routine.trajectory("Champs-Safe-Left-1");
    AutoTrajectory t1 = routine.trajectory("Safe-Left-Comp-Path-2");

    AutoCommands.bindMarkers(t0, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");
    AutoCommands.bindMarkers(t1, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");

    routine.active().onTrue(Commands.sequence(t0.resetOdometry(), t0.cmd()));
    t0.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 3.0),
                AutoCommands.stopShooting(ctx),
                AutoCommands.deployIntakeDown(ctx),
                t1.cmd()));
    t1.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 3.25),
                AutoCommands.stopShooting(ctx),
                AutoCommands.deployIntakeDown(ctx)));

    List<Pose2d> previewPoses = new java.util.ArrayList<>();
    previewPoses.addAll(getTrajectoryPoses(t0));
    previewPoses.addAll(getTrajectoryPoses(t1));
    return new AutoOption(routine::cmd, previewPoses, getStartingPose(t0));
  }

  /** Converted from PathPlanner auto "Champs-Safe-Right". */
  public static AutoOption champsSafeRight(AutoContext ctx) {
    AutoRoutine routine = ctx.autoFactory().newRoutine("Champs-Safe-Right");
    AutoTrajectory t0 = routine.trajectory("Champs-Safe-Right-1");
    AutoTrajectory t1 = routine.trajectory("Safe-Right-Comp-Path-2");

    AutoCommands.bindMarkers(t0, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");
    AutoCommands.bindMarkers(t1, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");

    routine.active().onTrue(Commands.sequence(t0.resetOdometry(), t0.cmd()));
    t0.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 3.25),
                AutoCommands.deployIntakeDown(ctx),
                AutoCommands.stopShooting(ctx),
                t1.cmd()));
    t1.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 3.5),
                AutoCommands.deployIntakeDown(ctx),
                AutoCommands.stopShooting(ctx)));

    List<Pose2d> previewPoses = new java.util.ArrayList<>();
    previewPoses.addAll(getTrajectoryPoses(t0));
    previewPoses.addAll(getTrajectoryPoses(t1));
    return new AutoOption(routine::cmd, previewPoses, getStartingPose(t0));
  }

  /** Converted from PathPlanner auto "Left-Depot". */
  public static AutoOption leftDepot(AutoContext ctx) {
    AutoRoutine routine = ctx.autoFactory().newRoutine("Left-Depot");
    AutoTrajectory t0 = routine.trajectory("Left-Comp-Path-1");
    AutoTrajectory t1 = routine.trajectory("Left-Depot-Path-1");

    AutoCommands.bindMarkers(t0, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");

    routine.active().onTrue(Commands.sequence(t0.resetOdometry(), t0.cmd()));
    t0.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 4.0), AutoCommands.stopShooting(ctx), t1.cmd()));
    t1.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 4.0), AutoCommands.stopShooting(ctx)));

    List<Pose2d> previewPoses = new java.util.ArrayList<>();
    previewPoses.addAll(getTrajectoryPoses(t0));
    previewPoses.addAll(getTrajectoryPoses(t1));
    return new AutoOption(routine::cmd, previewPoses, getStartingPose(t0));
  }

  /** Converted from PathPlanner auto "Left-Disruptor". */
  public static AutoOption leftDisruptor(AutoContext ctx) {
    AutoRoutine routine = ctx.autoFactory().newRoutine("Left-Disruptor");
    AutoTrajectory t0 = routine.trajectory("Left-Disruptor-Path-1");
    AutoTrajectory t1 = routine.trajectory("Left-Disruptor-Path-2");

    AutoCommands.bindMarkers(t1, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");

    routine.active().onTrue(Commands.sequence(t0.resetOdometry(), t0.cmd()));
    t0.done().onTrue(t1.cmd());
    t1.done().onTrue(AutoCommands.shootToHubTimed(ctx, 6.0));

    List<Pose2d> previewPoses = new java.util.ArrayList<>();
    previewPoses.addAll(getTrajectoryPoses(t0));
    previewPoses.addAll(getTrajectoryPoses(t1));
    return new AutoOption(routine::cmd, previewPoses, getStartingPose(t0));
  }

  /** Converted from PathPlanner auto "Left-Double". */
  public static AutoOption leftDouble(AutoContext ctx) {
    AutoRoutine routine = ctx.autoFactory().newRoutine("Left-Double");
    AutoTrajectory t0 = routine.trajectory("Left-Path");
    AutoTrajectory t1 = routine.trajectory("Left-Near-Path");

    AutoCommands.bindMarkers(t0, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");
    AutoCommands.bindMarkers(t1, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");

    routine.active().onTrue(Commands.sequence(t0.resetOdometry(), t0.cmd()));
    t0.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 4.0), AutoCommands.stopShooting(ctx), t1.cmd()));
    t1.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 4.5), AutoCommands.stopShooting(ctx)));

    List<Pose2d> previewPoses = new java.util.ArrayList<>();
    previewPoses.addAll(getTrajectoryPoses(t0));
    previewPoses.addAll(getTrajectoryPoses(t1));
    return new AutoOption(routine::cmd, previewPoses, getStartingPose(t0));
  }

  /** Converted from PathPlanner auto "Qual55". */
  public static AutoOption qual55(AutoContext ctx) {
    AutoRoutine routine = ctx.autoFactory().newRoutine("Qual55");
    AutoTrajectory t0 = routine.trajectory("Copy-of-Left-Comp-Path-2");
    AutoTrajectory t1 = routine.trajectory("Left-Comp-Path-2");

    AutoCommands.bindMarkers(t0, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");
    AutoCommands.bindMarkers(t1, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");

    routine.active().onTrue(Commands.sequence(t0.resetOdometry(), t0.cmd()));
    t0.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 2.6),
                AutoCommands.stopShooting(ctx),
                AutoCommands.deployIntakeDown(ctx),
                t1.cmd()));
    t1.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 3.0),
                AutoCommands.stopShooting(ctx),
                AutoCommands.deployIntakeDown(ctx)));

    List<Pose2d> previewPoses = new java.util.ArrayList<>();
    previewPoses.addAll(getTrajectoryPoses(t0));
    previewPoses.addAll(getTrajectoryPoses(t1));
    return new AutoOption(routine::cmd, previewPoses, getStartingPose(t0));
  }

  /** Converted from PathPlanner auto "Right-Disruptor". */
  public static AutoOption rightDisruptor(AutoContext ctx) {
    AutoRoutine routine = ctx.autoFactory().newRoutine("Right-Disruptor");
    AutoTrajectory t0 = routine.trajectory("Right-Disruptor-Path-1");
    AutoTrajectory t1 = routine.trajectory("Right-Disruptor-Path-2");

    AutoCommands.bindMarkers(t1, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");

    routine.active().onTrue(Commands.sequence(t0.resetOdometry(), t0.cmd()));
    t0.done().onTrue(t1.cmd());
    t1.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 6.0), AutoCommands.stopShooting(ctx)));

    List<Pose2d> previewPoses = new java.util.ArrayList<>();
    previewPoses.addAll(getTrajectoryPoses(t0));
    previewPoses.addAll(getTrajectoryPoses(t1));
    return new AutoOption(routine::cmd, previewPoses, getStartingPose(t0));
  }

  /** Converted from PathPlanner auto "Right-Double". */
  public static AutoOption rightDouble(AutoContext ctx) {
    AutoRoutine routine = ctx.autoFactory().newRoutine("Right-Double");
    AutoTrajectory t0 = routine.trajectory("Right-Path-1");
    AutoTrajectory t1 = routine.trajectory("Right-Path-2");

    routine.active().onTrue(Commands.sequence(t0.resetOdometry(), t0.cmd()));
    t0.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 4.0), AutoCommands.stopShooting(ctx), t1.cmd()));

    List<Pose2d> previewPoses = new java.util.ArrayList<>();
    previewPoses.addAll(getTrajectoryPoses(t0));
    previewPoses.addAll(getTrajectoryPoses(t1));
    return new AutoOption(routine::cmd, previewPoses, getStartingPose(t0));
  }

  /** Converted from PathPlanner auto "Safe-2-Left-Comp". */
  public static AutoOption safe2LeftComp(AutoContext ctx) {
    AutoRoutine routine = ctx.autoFactory().newRoutine("Safe-2-Left-Comp");
    AutoTrajectory t0 = routine.trajectory("Safe-Left-Comp-Path-1");
    AutoTrajectory t1 = routine.trajectory("Safe-Left-Comp-Path-2");

    AutoCommands.bindMarkers(t0, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");
    AutoCommands.bindMarkers(t1, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");

    routine.active().onTrue(Commands.sequence(t0.resetOdometry(), t0.cmd()));
    t0.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 3.0),
                AutoCommands.stopShooting(ctx),
                AutoCommands.deployIntakeDown(ctx),
                t1.cmd()));
    t1.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 3.25),
                AutoCommands.stopShooting(ctx),
                AutoCommands.deployIntakeDown(ctx)));

    List<Pose2d> previewPoses = new java.util.ArrayList<>();
    previewPoses.addAll(getTrajectoryPoses(t0));
    previewPoses.addAll(getTrajectoryPoses(t1));
    return new AutoOption(routine::cmd, previewPoses, getStartingPose(t0));
  }

  /** Converted from PathPlanner auto "Safe-2-Right-Comp". */
  public static AutoOption safe2RightComp(AutoContext ctx) {
    AutoRoutine routine = ctx.autoFactory().newRoutine("Safe-2-Right-Comp");
    AutoTrajectory t0 = routine.trajectory("Safe-Right-Comp-Path-1");
    AutoTrajectory t1 = routine.trajectory("Safe-Right-Comp-Path-2");

    AutoCommands.bindMarkers(t0, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");
    AutoCommands.bindMarkers(t1, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");

    routine.active().onTrue(Commands.sequence(t0.resetOdometry(), t0.cmd()));
    t0.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 3.0),
                AutoCommands.stopShooting(ctx),
                AutoCommands.deployIntakeDown(ctx),
                t1.cmd()));
    t1.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 3.25),
                AutoCommands.stopShooting(ctx),
                AutoCommands.deployIntakeDown(ctx)));

    List<Pose2d> previewPoses = new java.util.ArrayList<>();
    previewPoses.addAll(getTrajectoryPoses(t0));
    previewPoses.addAll(getTrajectoryPoses(t1));
    return new AutoOption(routine::cmd, previewPoses, getStartingPose(t0));
  }

  /** Converted from PathPlanner auto "State-Elims". */
  public static AutoOption stateElims(AutoContext ctx) {
    AutoRoutine routine = ctx.autoFactory().newRoutine("State-Elims");
    AutoTrajectory t0 = routine.trajectory("Safe-Left-Comp-Path-1");
    AutoTrajectory t1 = routine.trajectory("Left-Comp-Path-2");

    AutoCommands.bindMarkers(t0, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");
    AutoCommands.bindMarkers(t1, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");

    routine.active().onTrue(Commands.sequence(t0.resetOdometry(), t0.cmd()));
    t0.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 3.8),
                AutoCommands.stopShooting(ctx),
                AutoCommands.deployIntakeDown(ctx),
                t1.cmd()));
    t1.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 4.0),
                AutoCommands.deployIntakeDown(ctx),
                AutoCommands.stopShooting(ctx)));

    List<Pose2d> previewPoses = new java.util.ArrayList<>();
    previewPoses.addAll(getTrajectoryPoses(t0));
    previewPoses.addAll(getTrajectoryPoses(t1));
    return new AutoOption(routine::cmd, previewPoses, getStartingPose(t0));
  }

  /** Converted from PathPlanner auto "qual2". */
  public static AutoOption qual2(AutoContext ctx) {
    AutoRoutine routine = ctx.autoFactory().newRoutine("qual2");
    AutoTrajectory t0 = routine.trajectory("Champs-Left-1");
    AutoTrajectory t1 = routine.trajectory("qual2-path-2");

    AutoCommands.bindMarkers(t0, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");
    AutoCommands.bindMarkers(t1, ctx, "DeployIntake", "RunIntakeStart", "RunIntakeStop");

    routine.active().onTrue(Commands.sequence(t0.resetOdometry(), t0.cmd()));
    t0.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 3.25),
                AutoCommands.stopShooting(ctx),
                AutoCommands.deployIntakeDown(ctx),
                t1.cmd()));
    t1.done()
        .onTrue(
            Commands.sequence(
                AutoCommands.shootToHubTimed(ctx, 3.25),
                AutoCommands.stopShooting(ctx),
                AutoCommands.deployIntakeDown(ctx)));

    List<Pose2d> previewPoses = new java.util.ArrayList<>();
    previewPoses.addAll(getTrajectoryPoses(t0));
    previewPoses.addAll(getTrajectoryPoses(t1));
    return new AutoOption(routine::cmd, previewPoses, getStartingPose(t0));
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
