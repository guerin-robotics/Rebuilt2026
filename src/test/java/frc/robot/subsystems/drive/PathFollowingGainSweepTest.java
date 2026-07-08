package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Headless simulation harness for tuning the PathPlanner path-following PID gains in {@code
 * AutoBuilder.configure()} (Drive.java).
 *
 * <p>Runs real .path files against the ModuleIOSim swerve physics with candidate translation and
 * rotation kP values, and reports tracking error plus a choppiness metric (RMS / max of the
 * commanded chassis acceleration). A mid-path pose jump simulates a vision correction so the
 * recovery response is part of the comparison.
 *
 * <p>This is a tuning tool, not a regression gate — the single @Test prints a comparison table.
 */
public class PathFollowingGainSweepTest {

  private static final double DT = 0.02;
  private static final int MAX_STEPS = 1500; // 30 s safety cap
  private static final int POSE_JUMP_STEP = 100; // inject vision-style pose jump at t = 2.0 s
  private static final double POSE_JUMP_METERS = 0.06;

  private static Drive drive;
  private static RobotConfig robotConfig;
  private static java.util.function.Supplier<ChassisSpeeds> chassisSpeedsSupplier;

  @BeforeAll
  static void setup() throws Exception {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    DriverStation.silenceJoystickConnectionWarning(true);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    // Logger is deliberately not started (requires LoggedRobot); Logger calls no-op when
    // not running, which is fine for this harness

    // Stub the event-marker commands referenced by the .path files
    NamedCommands.registerCommand("DeployIntake", Commands.none());
    NamedCommands.registerCommand("RunIntake", Commands.none());

    drive =
        new Drive(
            new GyroIO() {},
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));

    robotConfig = RobotConfig.fromGUISettings();

    // getChassisSpeeds() is intentionally private in Drive; access it reflectively so the
    // harness measures the same speeds AutoBuilder does without widening production visibility
    var method = Drive.class.getDeclaredMethod("getChassisSpeeds");
    method.setAccessible(true);
    chassisSpeedsSupplier =
        () -> {
          try {
            return (ChassisSpeeds) method.invoke(drive);
          } catch (ReflectiveOperationException e) {
            throw new RuntimeException(e);
          }
        };
  }

  private record Result(
      String path,
      double kpXY,
      double kpTheta,
      double rmsErr,
      double maxErr,
      double endErr,
      double rmsHeadingErrDeg,
      double rmsAccel,
      double maxAccel,
      int steps) {}

  private Result runPath(String pathName, double kpXY, double kpTheta) throws Exception {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    Pose2d start = path.getStartingHolonomicPose().orElseThrow();
    drive.setPose(start);

    // Let the sim modules settle at rest
    for (int i = 0; i < 10; i++) {
      drive.periodic();
      drive.stop();
      SimHooks.stepTiming(DT);
    }
    drive.setPose(start);

    AtomicReference<Pose2d> target = new AtomicReference<>(start);
    PathPlannerLogging.setLogTargetPoseCallback(target::set);

    AtomicReference<ChassisSpeeds> lastCmd = new AtomicReference<>(new ChassisSpeeds());
    List<Double> errs = new ArrayList<>();
    List<Double> headingErrs = new ArrayList<>();
    List<Double> accels = new ArrayList<>();

    FollowPathCommand cmd =
        new FollowPathCommand(
            path,
            drive::getPose,
            chassisSpeedsSupplier::get,
            (speeds, ff) -> {
              ChassisSpeeds prev = lastCmd.get();
              double ax = (speeds.vxMetersPerSecond - prev.vxMetersPerSecond) / DT;
              double ay = (speeds.vyMetersPerSecond - prev.vyMetersPerSecond) / DT;
              accels.add(Math.hypot(ax, ay));
              lastCmd.set(speeds);
              drive.runVelocity(speeds);
            },
            new PPHolonomicDriveController(
                new PIDConstants(kpXY, 0.0, 0.0), new PIDConstants(kpTheta, 0.0, 0.0)),
            robotConfig,
            () -> false,
            drive);

    cmd.initialize();
    int steps = 0;
    while (!cmd.isFinished() && steps < MAX_STEPS) {
      drive.periodic();
      cmd.execute();
      SimHooks.stepTiming(DT);
      steps++;

      if (steps == POSE_JUMP_STEP) {
        // Simulate a vision pose correction: shift the estimated pose sideways
        drive.setPose(
            drive.getPose().plus(new Transform2d(0.0, POSE_JUMP_METERS, Rotation2d.kZero)));
      }

      Pose2d pose = drive.getPose();
      Pose2d tgt = target.get();
      errs.add(pose.getTranslation().getDistance(tgt.getTranslation()));
      headingErrs.add(Math.abs(pose.getRotation().minus(tgt.getRotation()).getDegrees()));
    }
    cmd.end(steps >= MAX_STEPS);
    drive.stop();
    drive.periodic();
    SimHooks.stepTiming(DT);

    double endErr = drive.getPose().getTranslation().getDistance(target.get().getTranslation());
    return new Result(
        pathName,
        kpXY,
        kpTheta,
        rms(errs),
        errs.stream().mapToDouble(Double::doubleValue).max().orElse(0),
        endErr,
        rms(headingErrs),
        rms(accels),
        accels.stream().mapToDouble(Double::doubleValue).max().orElse(0),
        steps);
  }

  private static double rms(List<Double> vals) {
    return Math.sqrt(vals.stream().mapToDouble(v -> v * v).average().orElse(0));
  }

  @Test
  void gainSweep() throws Exception {
    String[] paths = {"Left-Comp-Path-1", "Right-Path-2", "Champs-Left-1"};
    double[][] gains = {
      {50.0, 50.0}, // current values in Drive.java
      {10.0, 7.5}, // parity with the validated Choreo follower gains
      {5.0, 5.0}, // PathPlanner / AdvantageKit template default
      {7.0, 6.0}, // midpoint candidate
    };

    System.out.printf(
        "%-18s %6s %6s | %8s %8s %8s | %10s | %9s %9s | %5s%n",
        "path",
        "kpXY",
        "kpTh",
        "rmsErr",
        "maxErr",
        "endErr",
        "rmsHdgDeg",
        "rmsAccel",
        "maxAccel",
        "steps");
    for (double[] g : gains) {
      for (String p : paths) {
        Result r = runPath(p, g[0], g[1]);
        System.out.printf(
            "%-18s %6.1f %6.1f | %8.4f %8.4f %8.4f | %10.3f | %9.2f %9.2f | %5d%n",
            r.path(),
            r.kpXY(),
            r.kpTheta(),
            r.rmsErr(),
            r.maxErr(),
            r.endErr(),
            r.rmsHeadingErrDeg(),
            r.rmsAccel(),
            r.maxAccel(),
            r.steps());
      }
    }
  }
}
