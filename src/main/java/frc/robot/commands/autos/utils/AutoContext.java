package frc.robot.commands.autos.utils;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakeRoller.intakeRoller;
import frc.robot.subsystems.lowerFeeder.LowerFeeder;
import frc.robot.subsystems.prestage.Prestage;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.upperFeeder.UpperFeeder;

/**
 * Shared dependencies for auto routines.
 *
 * <p>Every autonomous routine needs access to the same set of subsystems and the common Choreo
 * factory configuration. Bundling them into a single record keeps each auto method's parameter list
 * short and makes it easy to add new subsystems later without editing every auto signature.
 */
public record AutoContext(
    Drive drive,
    Flywheel flywheel,
    Prestage prestage,
    Hood hood,
    UpperFeeder upperFeeder,
    LowerFeeder lowerFeeder,
    Transport transport,
    IntakePivot intakePivot,
    intakeRoller intakeRoller,
    RobotState robotState,
    AutoFactory autoFactory) {

  /**
   * Creates the shared autonomous context and configures the Choreo factory used by all autos.
   *
   * <p>Also schedules a warm-up command so that trajectory loading doesn't cause a delay when the
   * match starts.
   */
  public static AutoContext create(
      Drive drive,
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      UpperFeeder upperFeeder,
      LowerFeeder lowerFeeder,
      Transport transport,
      IntakePivot intakePivot,
      intakeRoller intakeRoller,
      AutoFactory autoFactory) {

    RobotState robotState = RobotState.getInstance();

    // Warm up Choreo so trajectory loading happens before the match starts
    CommandScheduler.getInstance().schedule(autoFactory.warmupCmd());

    return new AutoContext(
        drive,
        flywheel,
        prestage,
        hood,
        upperFeeder,
        lowerFeeder,
        transport,
        intakePivot,
        intakeRoller,
        robotState,
        autoFactory);
  }
}
