package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;

/**
 * Command factory methods for the Feeder subsystem.
 *
 * <p><b>DEPRECATED:</b> Prefer calling commands directly on the subsystem instance (e.g., {@code
 * feeder.setFeederVoltageCommand(...)}).
 */
public class FeederCommands {

  /**
   * @deprecated Use {@link Feeder#setFeederVoltageCommand(Voltage)} instead.
   */
  public static Command setFeederVoltage(Feeder feeder, Voltage voltage) {
    return feeder.setFeederVoltageCommand(voltage);
  }

  /**
   * @deprecated Use {@link Feeder#stopCommand()} instead.
   */
  public static Command stop(Feeder feeder) {
    return feeder.stopCommand();
  }

  /**
   * @deprecated Use {@link Feeder#setFeederVelocityCommand(AngularVelocity)} instead.
   */
  public static Command setFeederVelocity(Feeder feeder, AngularVelocity feederVelo) {
    return feeder.setFeederVelocityCommand(feederVelo);
  }

  /**
   * @deprecated Use {@link Feeder#setVelocityAfterWaitCommand(AngularVelocity)} instead.
   */
  public static Command setVelocityAfterWait(Feeder feeder, AngularVelocity feederVelo) {
    return feeder.setVelocityAfterWaitCommand(feederVelo);
  }
}
