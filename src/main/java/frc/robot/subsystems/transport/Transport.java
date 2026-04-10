package frc.robot.subsystems.transport;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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

    // Report transport current usage to the battery logger
    Robot.batteryLogger.reportCurrentUsage(
        "Transport",
        false,
        inputs.TransportSupplyAmps != null ? inputs.TransportSupplyAmps.in(Units.Amps) : 0.0);
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
}
