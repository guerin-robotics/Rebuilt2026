package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.transport.Transport;

public class TransportCommands {

  public static Command runTransportVoltage(Transport transport, Voltage voltage) {
    return Commands.startEnd(
            () -> transport.setTransportVoltage(voltage), // Apply voltage
            () -> transport.setTransportVoltage(Volts.of(0)), // Stop on end
            transport)
        .withName("TransportVoltage_" + voltage.in(Volts) + "V");
  }

  public static Command stop(Transport transport) {
    return Commands.runOnce(() -> transport.setTransportVoltage(Volts.of(0)), transport)
        .withName("TransportStop");
  }

  public static Command runTorque(Transport transport, AngularVelocity transportVelo) {
    return Commands.startEnd(
        () -> transport.setTransportTorque(transportVelo),
        () -> transport.setTransportTorque(RotationsPerSecond.of(0)),
        transport);
  }
}
