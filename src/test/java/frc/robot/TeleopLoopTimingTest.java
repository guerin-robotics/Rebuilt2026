package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Regression gate on the cost of one teleop drive-loop iteration (CommandScheduler.run() with the
 * drive subsystem and {@code joystickDrive} active), measured headlessly against ModuleIOSim and
 * expressed in <b>estimated roboRIO milliseconds</b>.
 *
 * <p>Context: PR #89 put NetworkTables reads and per-call logging in the 50 Hz drive path, which
 * saturated the RIO CPU and stretched loop times; it had to be reverted. Wall-clock timing on a
 * desktop cannot catch that exact class of bug (local NT reads are microseconds here but expensive
 * on the RIO) — {@code DriverPresetsSimTest.ntEditsApplyOnlyOnRefresh} guards that pattern
 * structurally. This test instead catches gross per-iteration regressions: device config applies,
 * file I/O, busy-waits, allocation storms, or anything else that adds real milliseconds per loop.
 *
 * <p><b>RIO speed simulation:</b> the test first times a fixed calibration workload
 * (drive-loop-shaped floating-point math) on the machine running the test, then scales every
 * measured iteration by {@code RIO_REFERENCE_WORKLOAD_MS / hostWorkloadMs}. Assertions are made
 * against the real 20 ms RIO loop budget in that scaled time base, so the test is
 * machine-independent — a slow CI runner and a fast Mac normalize to the same estimate.
 *
 * <p>Baseline on an M-series Mac (2026-07): raw avg ~0.06 ms → estimated RIO avg ~2.5 ms. If this
 * test starts failing, profile what was added to Drive.periodic() or the teleop drive command
 * before touching the budgets.
 */
public class TeleopLoopTimingTest {

  private static final double DT = 0.02;
  private static final int WARMUP_ITERATIONS = 250; // let JIT + sim settle
  private static final int MEASURED_ITERATIONS = 1000; // 20 s of simulated teleop

  /**
   * Time for {@link #calibrationWorkloadMs()} on the roboRIO. ESTIMATE: ~40x an M-series Mac (~29
   * ms), based on typical roboRIO 2.0 vs desktop single-thread ratios — not yet measured on the
   * robot. To calibrate exactly: temporarily paste the calibration loop into robotInit(), deploy,
   * read the console, and replace this constant with the printed value.
   */
  private static final double RIO_REFERENCE_WORKLOAD_MS = 1200.0;

  // Budgets in estimated RIO time (real RIO loop budget: 20 ms)
  private static final double MAX_RIO_AVG_MS = 15.0;
  private static final double MAX_RIO_P99_MS = 20.0; // p99 instead of max: tolerate GC/OS spikes

  private static Drive drive;

  private static final AtomicReference<Double> stickX = new AtomicReference<>(0.0);
  private static final AtomicReference<Double> stickY = new AtomicReference<>(0.0);
  private static final AtomicReference<Double> stickOmega = new AtomicReference<>(0.0);

  @BeforeAll
  static void setup() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    DriverStation.silenceJoystickConnectionWarning(true);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    // Isolate from subsystems registered by other test classes in the same JVM
    CommandScheduler.getInstance().unregisterAllSubsystems();
    CommandScheduler.getInstance().cancelAll();

    drive =
        new Drive(
            new GyroIO() {},
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));
    drive.setPose(new Pose2d(4.0, 4.0, Rotation2d.kZero));

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> stickX.get(), () -> stickY.get(), () -> stickOmega.get()));
  }

  /**
   * Fixed workload of drive-loop-shaped math (trig, hypot, sqrt) used to measure how fast the
   * current machine is relative to the roboRIO. Must stay byte-identical to the version used to
   * measure {@link #RIO_REFERENCE_WORKLOAD_MS}.
   */
  private static double calibrationWorkloadMs() {
    double sink = 0;
    long start = System.nanoTime();
    for (int i = 0; i < 400_000; i++) {
      double a = i * 1e-4;
      sink += Math.hypot(Math.sin(a), Math.cos(a));
      sink += Math.atan2(a, (sink % 10) + 1);
      sink += Math.sqrt(a + 1);
    }
    if (sink == 42) System.out.println(sink); // defeat dead-code elimination
    return (System.nanoTime() - start) / 1e6;
  }

  /** Median of several calibration runs (first runs discarded as JIT warmup). */
  private static double hostCalibrationMs() {
    double[] runs = new double[7];
    for (int i = 0; i < runs.length; i++) {
      runs[i] = calibrationWorkloadMs();
    }
    double[] settled = Arrays.copyOfRange(runs, 2, runs.length);
    Arrays.sort(settled);
    return settled[settled.length / 2];
  }

  @AfterAll
  static void teardown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  @Test
  void teleopDriveIterationStaysWithinBudget() {
    CommandScheduler scheduler = CommandScheduler.getInstance();

    for (int i = 0; i < WARMUP_ITERATIONS; i++) {
      scheduler.run();
      SimHooks.stepTiming(DT);
    }

    double[] iterationMs = new double[MEASURED_ITERATIONS];
    for (int i = 0; i < MEASURED_ITERATIONS; i++) {
      // Vary the sticks so the command exercises deadband, shaping, and kinematics paths
      double t = i * DT;
      stickX.set(0.7 * Math.sin(t * 0.5));
      stickY.set(0.4 * Math.cos(t * 0.3));
      stickOmega.set(0.5 * Math.sin(t * 0.7));

      long start = System.nanoTime();
      scheduler.run(); // drive.periodic() + joystickDrive execute, like the real robot loop
      iterationMs[i] = (System.nanoTime() - start) / 1e6;
      SimHooks.stepTiming(DT);
    }

    double avg = Arrays.stream(iterationMs).average().orElseThrow();
    double[] sorted = iterationMs.clone();
    Arrays.sort(sorted);
    double p50 = sorted[(int) (sorted.length * 0.50)];
    double p99 = sorted[(int) (sorted.length * 0.99)];
    double max = sorted[sorted.length - 1];

    // Scale host time to estimated RIO time using the calibration workload
    double hostCalMs = hostCalibrationMs();
    double rioScale = RIO_REFERENCE_WORKLOAD_MS / hostCalMs;
    double rioAvg = avg * rioScale;
    double rioP99 = p99 * rioScale;

    System.out.printf(
        "Teleop drive iteration over %d loops: avg %.3f ms, p50 %.3f ms, p99 %.3f ms, max %.3f ms"
            + " (host)%n",
        MEASURED_ITERATIONS, avg, p50, p99, max);
    System.out.printf(
        "Host calibration %.1f ms → this machine is ~%.0fx the RIO. Estimated RIO: avg %.2f ms,"
            + " p99 %.2f ms (budgets: avg < %.0f ms, p99 < %.0f ms; RIO loop budget 20 ms)%n",
        hostCalMs, rioScale, rioAvg, rioP99, MAX_RIO_AVG_MS, MAX_RIO_P99_MS);

    // Sanity: the robot is actually driving during the measurement, not idling
    double distance =
        drive
            .getPose()
            .getTranslation()
            .getDistance(new Pose2d(4.0, 4.0, Rotation2d.kZero).getTranslation());
    assertTrue(distance > 0.5, "robot should have moved during the timing run, got " + distance);

    assertTrue(
        rioAvg < MAX_RIO_AVG_MS,
        String.format(
            "estimated RIO average iteration %.2f ms exceeds %.0f ms — something expensive was"
                + " added to the drive loop (see class javadoc before raising this budget)",
            rioAvg, MAX_RIO_AVG_MS));
    assertTrue(
        rioP99 < MAX_RIO_P99_MS,
        String.format(
            "estimated RIO p99 iteration %.2f ms exceeds %.0f ms — check for intermittent blocking"
                + " work (config applies, file I/O, NT flushes) in the drive loop",
            rioP99, MAX_RIO_P99_MS));
  }
}
