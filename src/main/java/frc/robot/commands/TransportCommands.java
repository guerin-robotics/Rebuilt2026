package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.transport.Transport;

/**
 * Command factory methods for the Transport subsystem.
 *
 * <p><b>DEPRECATED:</b> Prefer calling commands directly on the subsystem instance (e.g., {@code
 * transport.setTransportVoltageCommand(...)}).
 */
public class TransportCommands {

  /**
   * @deprecated Use {@link Transport#setTransportVoltageCommand(Voltage)} instead.
   */
  public static Command setTransportVoltage(Transport transport, Voltage voltage) {
    return transport.setTransportVoltageCommand(voltage);
  }

  /**
   * @deprecated Use {@link Transport#stopCommand()} instead.
   */
  public static Command stop(Transport transport) {
    return transport.stopCommand();
  }

  /**
   * @deprecated Use {@link Transport#setTransportVelocityCommand(AngularVelocity)} instead.
   */
  public static Command setTransportVelocity(Transport transport, AngularVelocity transportVelo) {
    return transport.setTransportVelocityCommand(transportVelo);
  }

  /**
   * @deprecated Use {@link Transport#setVoltageAfterWaitCommand(Voltage)} instead.
   */
  public static Command setVoltageAfterWait(Transport transport, Voltage voltage) {
    return transport.setVoltageAfterWaitCommand(voltage);
  }
}
