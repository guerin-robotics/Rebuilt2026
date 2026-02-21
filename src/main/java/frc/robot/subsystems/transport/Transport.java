package frc.robot.subsystems.transport;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.transport.io.TransportIO;

public class Transport extends SubsystemBase {
  private final TransportIO io;

  private final TransportIO.TransportIOInputs inputs = new TransportIO.TransportIOInputs();

  public Transport(TransportIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
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
