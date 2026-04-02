package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeRoller.intakeRoller;

/**
 * Command factory methods for the intake roller subsystem.
 *
 * <p><b>DEPRECATED:</b> Prefer calling commands directly on the subsystem instance (e.g., {@code
 * intakeRoller.setRollerVoltageCommand(...)}).
 */
public class intakeRollerCommands {

  /**
   * @deprecated Use {@link intakeRoller#setRollerVoltageCommand(Voltage)} instead.
   */
  public static Command setRollerVoltage(intakeRoller intakeRoller, Voltage voltage) {
    return intakeRoller.setRollerVoltageCommand(voltage);
  }

  /**
   * @deprecated Use {@link intakeRoller#setRollerVelocityCommand(AngularVelocity)} instead.
   */
  public static Command setRollerVelocity(intakeRoller intakeRoller, AngularVelocity rollerVelo) {
    return intakeRoller.setRollerVelocityCommand(rollerVelo);
  }

  /**
   * @deprecated Use {@link intakeRoller#stopIntakeRollerCommand()} instead.
   */
  public static Command stopIntakeRoller(intakeRoller intakeRoller) {
    return intakeRoller.stopIntakeRollerCommand();
  }

  /**
   * @deprecated Use {@link intakeRoller#setVoltageAfterWaitCommand(Voltage)} instead.
   */
  public static Command setVoltageAfterWait(intakeRoller intakeRoller, Voltage voltage) {
    return intakeRoller.setVoltageAfterWaitCommand(voltage);
  }
}
