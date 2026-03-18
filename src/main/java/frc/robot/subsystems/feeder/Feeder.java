package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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

    // Report energy usage
    Robot.batteryLogger.reportCurrentUsage(
        "Feeder", inputs.feederSupplyAmps != null ? inputs.feederSupplyAmps.in(Amps) : 0.0);
  }

  public void setFeederVoltage(Voltage volts) {
    io.setFeederVoltage(volts);
  }

  public void setFeederVelocity(AngularVelocity feederVelo) {
    io.setFeederVelocity(feederVelo);
  }

  public void setFeederVelocityAtRPM(AngularVelocity feederVelo, boolean isAtRPM) {
    if (isAtRPM) {
      io.setFeederVelocity(feederVelo);
    }
  }
}
