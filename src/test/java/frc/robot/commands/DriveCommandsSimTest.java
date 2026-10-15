package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.AllianceFlipUtil;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Functional regression tests for the two drive commands changed alongside the loop-cost work:
 * {@code joystickDrive} (now reads the cached {@code AllianceFlipUtil.shouldFlip()} instead of
 * calling {@code DriverStation.getAlliance()} inline) and {@code joystickDriveAtAngle} (now samples
 * {@code rotationSupplier} and {@code drive.getRotation()} once per loop instead of twice and four
 * times).
 *
 * <p>Both changes are meant to be behaviour-preserving. These tests assert the observable
 * behaviour, not the implementation: field-relative translation still flips on red, and heading
 * control still converges.
 *
 * <p><b>Ordering note.</b> {@code driveLoop()} calls {@code AllianceFlipUtil.refresh()} before
 * {@code CommandScheduler.run()}, mirroring {@code Robot.robotPeriodic()}. That ordering is load
 * bearing: {@code shouldFlip()} is a per-loop snapshot, so a red-alliance flip only takes effect
 * once something has refreshed it. {@link #redAllianceFlipsFieldRelativeForward()} is what fails if
 * that refresh is ever dropped or moved back after the scheduler.
 */
public class DriveCommandsSimTest {

  private static final double DT = 0.02;
  private static final Pose2d START = new Pose2d(4.0, 4.0, Rotation2d.kZero);

  private static Drive drive;

  @BeforeAll
  static void setup() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    DriverStation.silenceJoystickConnectionWarning(true);

    CommandScheduler.getInstance().unregisterAllSubsystems();
    CommandScheduler.getInstance().cancelAll();

    drive =
        new Drive(
            new GyroIO() {},
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));

    // joystickDrive raises the rotation input to this exponent. Robot.teleopInit() normally sets
    // it from the driver-preset chooser; pin it here so the test does not depend on the default.
    DriveConstants.rotationExponent = 2.0;
  }

  @BeforeEach
  void enabledTeleop() {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setFmsAttached(false);
    DriverStationSim.notifyNewData();
    drive.setPose(START);
  }

  @AfterEach
  void clearCommands() {
    CommandScheduler.getInstance().cancelAll();
  }

  @AfterAll
  static void teardown() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    AllianceFlipUtil.refresh();
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  private static void setAlliance(AllianceStationID station) {
    DriverStationSim.setAllianceStationId(station);
    DriverStationSim.notifyNewData();
  }

  /** One robot loop, in the same order Robot.robotPeriodic() runs them. */
  private static void driveLoop(int iterations) {
    for (int i = 0; i < iterations; i++) {
      AllianceFlipUtil.refresh();
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(DT);
    }
  }

  private static void run(Command command, int iterations) {
    CommandScheduler.getInstance().schedule(command);
    driveLoop(iterations);
  }

  /**
   * On blue, a positive X joystick input is field +X, so the robot's X coordinate increases. This
   * is the unflipped reference for the red case below.
   */
  @Test
  void blueAllianceDrivesFieldForwardAsCommanded() {
    setAlliance(AllianceStationID.Blue1);

    run(DriveCommands.joystickDrive(drive, () -> 1.0, () -> 0.0, () -> 0.0), 100);

    double deltaX = drive.getPose().getX() - START.getX();
    assertTrue(
        deltaX > 0.5,
        "blue alliance: +X joystick should move the robot toward +X, moved " + deltaX + " m");
  }

  /**
   * On red the field frame is rotated 180 degrees, so the same joystick input must move the robot
   * the other way in blue-origin field coordinates.
   *
   * <p>This is the test that covers swapping the inline {@code DriverStation.getAlliance()} for the
   * cached {@code AllianceFlipUtil.shouldFlip()}. It fails if the per-loop refresh stops happening
   * before the scheduler, because the snapshot would still read blue.
   */
  @Test
  void redAllianceFlipsFieldRelativeForward() {
    setAlliance(AllianceStationID.Red1);
    AllianceFlipUtil.refresh();
    assertTrue(AllianceFlipUtil.shouldFlip(), "precondition: cache should report red");

    run(DriveCommands.joystickDrive(drive, () -> 1.0, () -> 0.0, () -> 0.0), 100);

    double deltaX = drive.getPose().getX() - START.getX();
    assertTrue(
        deltaX < -0.5,
        "red alliance: +X joystick should move the robot toward -X, moved " + deltaX + " m");
  }

  /**
   * Heading control still converges after hoisting {@code rotationSupplier.get()} and {@code
   * drive.getRotation()} out of the lambda body. Commands 90 degrees with no translation.
   */
  @Test
  void joystickDriveAtAngleConvergesOnTargetHeading() {
    setAlliance(AllianceStationID.Blue1);

    Rotation2d target = Rotation2d.fromDegrees(90.0);
    run(
        DriveCommands.joystickDriveAtAngle(drive, () -> 0.0, () -> 0.0, () -> target),
        300); // 6 s of simulated time

    double errorDegrees = Math.abs(drive.getPose().getRotation().minus(target).getDegrees());
    assertTrue(
        errorDegrees < 2.0,
        "should settle within 2 deg of the 90 deg target, off by " + errorDegrees + " deg");
  }

  /**
   * The heading target is re-read every loop, so a supplier that moves mid-command must still be
   * tracked. Guards against the hoist accidentally caching the target across loops rather than
   * within one.
   */
  @Test
  void joystickDriveAtAngleTracksAMovingTarget() {
    setAlliance(AllianceStationID.Blue1);

    Rotation2d[] target = {Rotation2d.fromDegrees(45.0)};
    CommandScheduler.getInstance()
        .schedule(DriveCommands.joystickDriveAtAngle(drive, () -> 0.0, () -> 0.0, () -> target[0]));

    driveLoop(200);
    assertEquals(
        45.0,
        drive.getPose().getRotation().getDegrees(),
        2.0,
        "should reach the first target before it moves");

    target[0] = Rotation2d.fromDegrees(-45.0);
    driveLoop(300);
    assertEquals(
        -45.0,
        drive.getPose().getRotation().getDegrees(),
        2.0,
        "should follow the target after it changes mid-command");
  }
}
