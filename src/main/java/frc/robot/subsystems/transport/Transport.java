package frc.robot.subsystems.transport;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  public void setTransportSpeed(AngularVelocity speed) {
    io.setTransportSpeed(speed);
  }

  public void setTransportTorque(AngularVelocity transportVelo) {
    io.setTransportTorque(transportVelo);
  }
}
