package frc.robot.subsystems.feeder;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.feeder.io.FeederIO;
import frc.robot.subsystems.feeder.io.FeederIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {

  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs;

  public Feeder(FeederIO io) {
    this.io = io;
    this.inputs = new FeederIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }

  public void setFeederVoltage(Voltage volts) {
    io.setFeederVoltage(volts);
  }

  public void setFeederSpeed(AngularVelocity speed) {
    io.setFeederSpeed(speed);
  }

  public void setFeederTorqueControl(AngularVelocity feederVelo) {
    io.setFeederTorqueControl(feederVelo);
  }
}
