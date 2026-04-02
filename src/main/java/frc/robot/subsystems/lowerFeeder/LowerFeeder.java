package frc.robot.subsystems.lowerFeeder;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.lowerFeeder.io.LowerFeederIO;
import frc.robot.subsystems.lowerFeeder.io.LowerFeederIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class LowerFeeder extends SubsystemBase {

  private final LowerFeederIO io;
  private final LowerFeederIOInputsAutoLogged inputs;

  public LowerFeeder(LowerFeederIO io) {
    this.io = io;
    this.inputs = new LowerFeederIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }

  public void setLowerFeederVoltage(Voltage volts) {
    io.setLowerFeederVoltage(volts);
  }

  public void setLowerFeederVelocity(AngularVelocity feederVelo) {
    io.setLowerFeederVelocity(feederVelo);
  }

  public void setLowerFeederVelocityAtRPM(AngularVelocity feederVelo, boolean isAtRPM) {
    if (isAtRPM) {
      io.setLowerFeederVelocity(feederVelo);
    }
  }
}
