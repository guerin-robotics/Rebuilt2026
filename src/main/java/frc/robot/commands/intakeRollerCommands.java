package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.intakeRoller.intakeRoller;
import java.util.function.BooleanSupplier;

public class intakeRollerCommands {

  public static Command setRollerVoltage(intakeRoller intakeRoller, Voltage voltage) {
    return Commands.startEnd(
            () -> intakeRoller.setRollerVoltage(voltage),
            () -> intakeRoller.setRollerVoltage(Volts.of(0)),
            intakeRoller)
        .withName("IntakeRollerVoltage_" + voltage.in(Volts) + "V");
  }

  public static Command setRollerVelocity(intakeRoller intakeRoller, AngularVelocity rollerVelo) {
    return Commands.runOnce(() -> intakeRoller.setRollerVelocity(rollerVelo), intakeRoller)
        .withName("IntakeRollerVelocity");
  }

  public static Command intakeRollerIdle(intakeRoller intakeRoller) {
    return Commands.run(() -> intakeRoller.setIntakeRollerIdle(), intakeRoller)
        .withName("intakeRollerIdle");
  }

  public static Command stopIntakeRoller(intakeRoller intakeRoller) {
    return Commands.runOnce(
            () -> intakeRoller.setRollerVelocity(RotationsPerSecond.of(0)), intakeRoller)
        .withName("IntakeRollerStop");
  }

  /**
   * Runs the intake roller at the given agitate voltage, but only once the shot is actually being
   * fed. The caller passes the same condition its feeder/transport binding gates on, so the roller
   * starts agitating on the same loop the fuel starts moving — agitating earlier just stirs the
   * hopper while nothing is being fed, and agitating later wastes part of the shot.
   *
   * <p>Unlike the feeder/transport wait sequences, this does not fall through to agitating after
   * the timeout — if the shot never becomes ready, the roller is held at zero (claiming the
   * subsystem so the default command cannot re-engage) until the caller's whileTrue binding is
   * released.
   *
   * <p>See {@link FeederCommands#setLowerVelocityAfterWait} for full details on the wait logic.
   *
   * @param intakeRoller The intake roller subsystem
   * @param voltage The agitate voltage to apply once ready
   * @param readyToFeed Supplier that returns true when the feeders and transport are allowed to run
   */
  public static Command setVoltageAfterWait(
      intakeRoller intakeRoller, Voltage voltage, BooleanSupplier readyToFeed) {
    return Commands.sequence(
            Commands.waitUntil(readyToFeed)
                .withTimeout(HardwareConstants.CompConstants.Waits.alignmentTimeoutSeconds),
            Commands.either(
                setRollerVoltage(intakeRoller, voltage),
                Commands.run(() -> {}, intakeRoller),
                readyToFeed))
        .withName("IntakeRollerVoltageAfterWait");
  }
}
