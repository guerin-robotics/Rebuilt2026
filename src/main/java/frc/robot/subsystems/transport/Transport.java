package frc.robot.subsystems.transport;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.transport.io.TransportIO;
import frc.robot.subsystems.transport.io.TransportIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Transport extends SubsystemBase {
  private final TransportIO io;

  private final TransportIOInputsAutoLogged inputs;

  public Transport(TransportIO io) {
    this.io = io;
    this.inputs = new TransportIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Transport", inputs);
  }

  public void setTransportVoltage(Voltage volts) {
    io.setTransportVoltage(volts);
  }

  public void setTransportVelocity(AngularVelocity transportVelo) {
    io.setTransportVelocity(transportVelo);
  }

  public void setTransportVelocityAtRPM(AngularVelocity transportVelo, boolean isAtRPM) {
    if (isAtRPM) {
      io.setTransportVelocity(transportVelo);
    }
  }

  // ==================== COMMAND FACTORIES ====================

  /** Runs the transport at a specific voltage; stops (0V) when the command ends. */
  public Command setTransportVoltageCommand(Voltage voltage) {
    return Commands.startEnd(
            () -> setTransportVoltage(voltage), () -> setTransportVoltage(Volts.of(0)), this)
        .withName("TransportVoltage_" + voltage.in(Volts) + "V");
  }

  /** Stops the transport immediately. */
  public Command stopCommand() {
    return Commands.runOnce(() -> setTransportVoltage(Volts.of(0)), this).withName("TransportStop");
  }

  /** Sets the transport to a given velocity (instant). */
  public Command setTransportVelocityCommand(AngularVelocity transportVelo) {
    return Commands.runOnce(() -> setTransportVelocity(transportVelo), this);
  }

  /** Waits for flywheel spinup, then sets transport voltage. */
  public Command setVoltageAfterWaitCommand(Voltage voltage) {
    return Commands.sequence(
        new WaitCommand(HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
        setTransportVoltageCommand(voltage));
  }
}
