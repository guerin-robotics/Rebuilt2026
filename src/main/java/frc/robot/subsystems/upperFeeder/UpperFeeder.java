package frc.robot.subsystems.upperFeeder;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.upperFeeder.io.UpperFeederIO;
import frc.robot.subsystems.upperFeeder.io.UpperFeederIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class UpperFeeder extends SubsystemBase {

  private final UpperFeederIO io;
  private final UpperFeederIOInputsAutoLogged inputs;

  public UpperFeeder(UpperFeederIO io) {
    this.io = io;
    this.inputs = new UpperFeederIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }

  public void setUpperFeederVoltage(Voltage volts) {
    io.setUpperFeederVoltage(volts);
  }

  public void setUpperFeederVelocity(AngularVelocity feederVelo) {
    io.setUpperFeederVelocity(feederVelo);
  }

  public void setUpperFeederVelocityAtRPM(AngularVelocity feederVelo, boolean isAtRPM) {
    if (isAtRPM) {
      io.setUpperFeederVelocity(feederVelo);
    }
  }
}
