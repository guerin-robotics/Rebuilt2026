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
 * on the RIO) — the companion {@code DriverPresetsSimTest} used to guard that pattern structurally,
 * but it was deleted in the IRI merge (ad069ef) and has not been restored, so that hole is
 * currently unguarded. This test catches the other class: gross per-iteration regressions — device
 * config applies, file I/O, busy-waits, allocation storms, or anything else that adds real
 * milliseconds per loop.
 *
 * <p><b>RIO speed simulation:</b> the test first times a fixed calibration workload
 * (drive-loop-shaped floating-point math) on the machine running the test, then scales every
 * measured iteration by {@code RIO_REFERENCE_WORKLOAD_MS / hostWorkloadMs}. Assertions are made
 * against the real 20 ms RIO loop budget in that scaled time base, so the test is
 * machine-independent — a slow CI runner and a fast Mac normalize to the same estimate.
 *
 * <p><b>Baselines.</b> M-series Mac (2026-07): raw avg ~0.06 ms → estimated RIO avg ~2.5 ms.
 * Windows dev box (2026-07-26, on restore): raw avg 0.09–0.11 ms, host ~25x the RIO → estimated RIO
 * avg 2.4–2.9 ms, p99 6.8–9.7 ms, across five runs. If this test starts failing, profile what was
 * added to Drive.periodic() or the teleop drive command before touching the budgets.
 *
 * <p><b>Why the tail is reported but not asserted.</b> The original version of this test gated on
 * p99 &lt; 20 estimated ms, and that gate is not sound. The host→RIO scale (~17-27x depending on
 * machine load) multiplies host-side stalls that have no RIO meaning: one desktop GC or OS
 * scheduling pause becomes ~25 estimated ms and fails a 20 ms tail budget by itself. Measured over
 * 13 runs on one machine while varying background load, the tail ranged 5.9 → 49.6 estimated ms
 * while the average never left 2.2 → 4.3 and the median was steadier still. A statistic that swings
 * 8x on unchanged code is measuring the host, not the robot.
 *
 * <p>So the gate is <b>average and median</b>; p99 and max are printed for humans to read when
 * investigating. A regression that adds real per-iteration cost moves the median. A regression that
 * adds intermittent blocking work (config applies, file I/O, NT flushes) shows up in the printed
 * tail — look at it, but do not let CI fail on it.
 *
 * <p>This is almost certainly why the test was deleted in 9fadc35 rather than any real regression:
 * it and PathFollowingGainSweepTest both pass unmodified on this tree. A contributing factor was
 * JVM sharing — constructing a Drive starts the PhoenixOdometryThread singleton, which is never
 * stopped, so a second test class building a Drive left a 250 Hz thread competing with the
 * measurement. build.gradle now sets {@code forkEvery = 1} to isolate that.
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

  // Median budget. Together with the average this is the actual gate — both are robust to host
  // OS/GC stalls, and both move if per-iteration cost genuinely regresses.
  private static final double MAX_RIO_P50_MS = 10.0;

  // NOT ASSERTED — printed as a diagnostic only. See the class javadoc: the host->RIO scale
  // multiplies desktop stalls that have no RIO meaning, so a single GC pause turns into ~25
  // estimated ms and fails a 20 ms tail budget on its own. Measured across 13 runs on one machine
  // the tail ranged 5.9 -> 49.6 estimated ms while the average never left 2.2 -> 4.3, which is
  // what a noise-dominated statistic looks like. Read it when investigating; do not gate on it.
  private static final double TAIL_DIAGNOSTIC_REFERENCE_MS = 20.0;

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
    double rioP50 = p50 * rioScale;
    double rioP99 = p99 * rioScale;
    double rioMax = max * rioScale;

    System.out.printf(
        "Teleop drive iteration over %d loops: avg %.3f ms, p50 %.3f ms, p99 %.3f ms, max %.3f ms"
            + " (host)%n",
        MEASURED_ITERATIONS, avg, p50, p99, max);
    System.out.printf(
        "Host calibration %.1f ms → this machine is ~%.0fx the RIO. Estimated RIO: avg %.2f ms,"
            + " p50 %.2f ms (gated: avg < %.0f, p50 < %.0f). Tail diagnostics (NOT gated,"
            + " host-noise dominated): p99 %.2f ms, max %.2f ms vs %.0f ms reference;"
            + " RIO loop budget 20 ms%n",
        hostCalMs,
        rioScale,
        rioAvg,
        rioP50,
        MAX_RIO_AVG_MS,
        MAX_RIO_P50_MS,
        rioP99,
        rioMax,
        TAIL_DIAGNOSTIC_REFERENCE_MS);

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
        rioP50 < MAX_RIO_P50_MS,
        String.format(
            "estimated RIO median iteration %.2f ms exceeds %.0f ms — per-iteration cost regressed"
                + " (see class javadoc before raising this budget)",
            rioP50, MAX_RIO_P50_MS));
  }
}
