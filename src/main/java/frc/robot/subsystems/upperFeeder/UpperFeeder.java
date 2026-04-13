package frc.robot.subsystems.upperFeeder;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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
    Logger.processInputs("Feeder/Upper", inputs);

    // Report upper feeder current usage to the battery logger
    Robot.batteryLogger.reportCurrentUsage(
        "Feeder/Upper",
        false,
        inputs.upperFeederSupplyAmps != null ? inputs.upperFeederSupplyAmps.in(Units.Amps) : 0.0);

    // Log the currently running command for this subsystem
    Logger.recordOutput(
        "Feeder/Upper/CurrentCommand",
        getCurrentCommand() != null ? getCurrentCommand().getName() : "none");
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
