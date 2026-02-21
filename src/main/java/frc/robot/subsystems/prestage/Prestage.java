package frc.robot.subsystems.prestage;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.prestage.io.PrestageIO;
import frc.robot.subsystems.prestage.io.PrestageIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Prestage extends SubsystemBase {
  private final PrestageIO io;

  private final PrestageIOInputsAutoLogged inputs;

  public Prestage(PrestageIO io) {
    this.io = io;
    this.inputs = new PrestageIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Prestage", inputs);
  }

  public void setPrestageVoltage(Voltage volts) {
    io.setPrestageVoltage(volts);
  }

  public void setPrestageSpeed(AngularVelocity speed) {
    io.setPrestageSpeed(speed);
  }

  public void setPrestageTorque(AngularVelocity prestageVelo) {
    io.setPrestageTorque(prestageVelo);
  }
}
