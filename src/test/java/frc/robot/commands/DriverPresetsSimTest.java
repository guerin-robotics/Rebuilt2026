package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.util.DriverPresets;
import java.lang.reflect.Field;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Headless simulation check for the cached driver-preset flow that replaced the reverted PR #89.
 *
 * <p>Verifies three things: (1) the robot drives under {@code DriveCommands.joystickDrive} with the
 * cached exponents; (2) NT preset/exponent edits do NOT take effect mid-teleop and DO take effect
 * after a simulated disable→enable ({@code DriverPresets.refresh()}, as wired in {@code
 * Robot.teleopInit()}); (3) the drive hot path performs no NetworkTables access — the cached getter
 * is a plain field read, timed here to prove it.
 *
 * <p>Logger is deliberately not started (same as PathFollowingGainSweepTest); dashboard inputs are
 * ticked manually via their public {@code periodic()} methods, which is what Logger.periodic() does
 * on the real robot.
 */
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class DriverPresetsSimTest {

  private static final double DT = 0.02;

  private static Drive drive;
  private static DriverPresets presets;

  // Mutable "joystick" state read by the command's suppliers
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

    drive =
        new Drive(
            new GyroIO() {},
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));

    presets = DriverPresets.getInstance();
    tickDashboardInputs();
    presets.refresh(); // what Robot.teleopInit() does
  }

  // Keeps test-side NT publishers alive so their values don't vanish on GC
  private static final java.util.List<Object> livePublishers = new java.util.ArrayList<>();

  /**
   * Simulates an Elastic edit of a LoggedNetworkNumber. AdvantageKit registers these topics under
   * the literal key WITHOUT a leading slash, so a normalized "/Tune/..." entry write would land on
   * a different topic — publish to the exact key instead.
   */
  private static void setTuneNumber(String key, double value) {
    var pub = NetworkTableInstance.getDefault().getDoubleTopic(key).publish();
    pub.set(value);
    livePublishers.add(pub);
  }

  /** Mimics Logger.periodic()'s dashboard-input update, which the harness doesn't run. */
  @SuppressWarnings("unchecked")
  private static void tickDashboardInputs() {
    // Deliver queued NT writes to the SendableChooser (normally done by LoggedRobot each loop)
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.updateValues();
    try {
      Field chooserField = DriverPresets.class.getDeclaredField("chooser");
      chooserField.setAccessible(true);
      ((LoggedDashboardChooser<?>) chooserField.get(presets)).periodic();
      for (String mapName : new String[] {"driveExponents", "steerExponents"}) {
        Field mapField = DriverPresets.class.getDeclaredField(mapName);
        mapField.setAccessible(true);
        ((Map<?, LoggedNetworkNumber>) mapField.get(presets))
            .values()
            .forEach(LoggedNetworkNumber::periodic);
      }
    } catch (ReflectiveOperationException e) {
      throw new RuntimeException(e);
    }
  }

  private static Command newJoystickDrive() {
    return DriveCommands.joystickDrive(
        drive, () -> stickX.get(), () -> stickY.get(), () -> stickOmega.get());
  }

  /** Runs the teleop drive loop for {@code steps} iterations, returning avg/max user-code time. */
  private static double[] runLoop(Command cmd, int steps) {
    double totalNanos = 0;
    double maxNanos = 0;
    for (int i = 0; i < steps; i++) {
      long start = System.nanoTime();
      drive.periodic();
      cmd.execute();
      long elapsed = System.nanoTime() - start;
      totalNanos += elapsed;
      maxNanos = Math.max(maxNanos, elapsed);
      SimHooks.stepTiming(DT);
    }
    return new double[] {totalNanos / steps / 1e6, maxNanos / 1e6};
  }

  /** Steady-state linear speed for a given fixed forward stick, using the current cached preset. */
  private static double steadyStateSpeed(double stick) {
    stickX.set(stick);
    stickY.set(0.0);
    stickOmega.set(0.0);
    Command cmd = newJoystickDrive();
    cmd.initialize();
    runLoop(cmd, 100); // 2 s — plenty to reach steady state
    Pose2d before = drive.getPose();
    runLoop(cmd, 50); // measure over 1 s
    Pose2d after = drive.getPose();
    cmd.end(true);
    drive.stop();
    stickX.set(0.0);
    return before.getTranslation().getDistance(after.getTranslation()) / (50 * DT);
  }

  @Test
  @Order(1)
  void robotDrivesAndTurnsWithBootPreset() {
    assertEquals(2.0, presets.getDriveExponent(), 1e-9, "Parker drive exponent at boot");
    assertEquals(1.35, presets.getSteerExponent(), 1e-9, "Parker steer exponent at boot");

    drive.setPose(new Pose2d(4.0, 4.0, Rotation2d.kZero));
    Command cmd = newJoystickDrive();
    cmd.initialize();

    stickX.set(0.8); // forward
    runLoop(cmd, 100);
    stickX.set(0.0);
    stickOmega.set(0.6); // rotate in place
    runLoop(cmd, 100);
    stickOmega.set(0.0);
    cmd.end(true);
    drive.stop();

    Pose2d pose = drive.getPose();
    double distance =
        pose.getTranslation().getDistance(new Pose2d(4.0, 4.0, Rotation2d.kZero).getTranslation());
    double rotationDeg = Math.abs(pose.getRotation().getDegrees());
    System.out.printf(
        "Drove %.2f m and rotated %.1f deg under joystickDrive with cached preset%n",
        distance, rotationDeg);
    assertTrue(distance > 1.0, "robot should have translated >1 m, got " + distance);
    assertTrue(rotationDeg > 20.0, "robot should have rotated >20 deg, got " + rotationDeg);
  }

  @Test
  @Order(2)
  void ntEditsApplyOnlyOnRefresh() {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    // Edit Parker's drive exponent in "Elastic" while teleop is running
    setTuneNumber("Tune/DriverPreset/Parker/DriveExponent", 3.0);
    tickDashboardInputs(); // Logger.periodic() would do this every loop on the real robot

    Command cmd = newJoystickDrive();
    cmd.initialize();
    stickX.set(0.5);
    runLoop(cmd, 25);
    cmd.end(true);
    drive.stop();
    stickX.set(0.0);
    assertEquals(
        2.0,
        presets.getDriveExponent(),
        1e-9,
        "NT edit must NOT apply mid-teleop (no per-loop NT reads)");

    // Disable → enable: Robot.teleopInit() calls refresh()
    presets.refresh();
    assertEquals(3.0, presets.getDriveExponent(), 1e-9, "NT edit applies after teleop re-enable");

    // Switch driver preset via the Elastic chooser, then re-enable
    nt.getEntry("/SmartDashboard/Driver Preset/selected").setString("Cristian");
    tickDashboardInputs();
    assertEquals(3.0, presets.getDriveExponent(), 1e-9, "chooser switch must NOT apply mid-teleop");
    presets.refresh();
    assertEquals(2.0, presets.getDriveExponent(), 1e-9, "Cristian drive exponent after re-enable");
    assertEquals(2.0, presets.getSteerExponent(), 1e-9, "Cristian steer exponent after re-enable");
  }

  @Test
  @Order(3)
  void exponentChangeAffectsRobotSpeed() {
    // Cristian preset (drive ^2.0) is active from the previous test
    drive.setPose(new Pose2d(4.0, 4.0, Rotation2d.kZero));
    double speedExp2 = steadyStateSpeed(0.5);

    // Bump Cristian's drive exponent to 3.0 and re-enable
    setTuneNumber("Tune/DriverPreset/Cristian/DriveExponent", 3.0);
    tickDashboardInputs();
    presets.refresh();
    assertEquals(3.0, presets.getDriveExponent(), 1e-9);

    drive.setPose(new Pose2d(4.0, 4.0, Rotation2d.kZero));
    double speedExp3 = steadyStateSpeed(0.5);

    System.out.printf(
        "Half-stick steady-state speed: %.3f m/s (exp 2.0) vs %.3f m/s (exp 3.0)%n",
        speedExp2, speedExp3);
    // Deadbanded half stick = 0.444; 0.444^3 / 0.444^2 = 0.444, so expect a large drop
    assertTrue(speedExp2 > 0.3, "exp-2 speed should be measurable, got " + speedExp2);
    assertTrue(
        speedExp3 < speedExp2 * 0.7,
        "exp-3 speed should be well below exp-2 speed: " + speedExp3 + " vs " + speedExp2);
  }

  @Test
  @Order(4)
  void hotPathStaysCheap() {
    // The cached getter must be a plain field read — nanoseconds, no NT, no logging
    long start = System.nanoTime();
    double sink = 0;
    int calls = 1_000_000;
    for (int i = 0; i < calls; i++) {
      sink += presets.getDriveExponent() + presets.getSteerExponent();
    }
    double nsPerCall = (System.nanoTime() - start) / (double) calls;
    System.out.printf(
        "getDriveExponent+getSteerExponent: %.1f ns per pair (sink=%.0f)%n", nsPerCall, sink);
    assertTrue(nsPerCall < 1000, "cached getters should be ns-scale, got " + nsPerCall + " ns");

    // Full teleop drive iteration (drive.periodic + joystickDrive execute) on this machine —
    // informational; the RIO budget is 20 ms and the point is there is no NT work in here
    Command cmd = newJoystickDrive();
    cmd.initialize();
    stickX.set(0.6);
    stickOmega.set(0.3);
    runLoop(cmd, 50); // warmup / JIT
    double[] stats = runLoop(cmd, 500);
    cmd.end(true);
    drive.stop();
    System.out.printf(
        "Teleop drive iteration: avg %.3f ms, max %.3f ms over 500 loops (20 ms budget)%n",
        stats[0], stats[1]);
    assertTrue(stats[0] < 20.0, "average iteration should be far under the 20 ms loop budget");
  }
}
