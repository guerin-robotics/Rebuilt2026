package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.transport.Transport;
import java.util.function.BooleanSupplier;

public class TransportCommands {

  public static Command setTransportVoltage(Transport transport, Voltage voltage) {
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

  public static Command setTransportVelocity(Transport transport, AngularVelocity transportVelo) {
    return Commands.runOnce(() -> transport.setTransportVelocity(transportVelo), transport)
        .withName("TransportVelocity");
  }

  public static Command setVoltageAfterWait(Transport transport, Voltage voltage) {
    return Commands.sequence(
            new WaitCommand(HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
            setTransportVoltage(transport, voltage))
        .withName("TransportVoltageAfterWait");
  }

  /**
   * Runs the transport at the given velocity, but only after the robot is aligned (or timeout).
   *
   * <p>See {@link FeederCommands#setLowerVelocityAfterWait} for full details on the wait logic.
   *
   * @param transport The transport subsystem
   * @param transportVelo The velocity to run at once ready
   * @param isAligned Supplier that returns true when the robot is facing its target
   */
  public static Command setVelocityAfterWait(
      Transport transport, AngularVelocity transportVelo, BooleanSupplier isAligned) {
    return Commands.sequence(
            new WaitCommand(HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
            Commands.waitUntil(isAligned)
                .withTimeout(
                    HardwareConstants.CompConstants.Waits.alignmentTimeoutSeconds
                        - HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
            setTransportVelocity(transport, transportVelo))
        .withName("TransportVelocityAfterWait");
  }

    public static Command setVelocityAfterWait(Transport transport, AngularVelocity transportVelo) {
      return setVelocityAfterWait(transport, transportVelo, () -> true)
          .withName("TransportVelocityAfterWaitNoAlign");
    }
}
