package frc.robot.subsystems.prestage;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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

    // Report prestage current usage to the battery logger (left + right motors)
    Robot.batteryLogger.reportCurrentUsage(
        "Prestage/Left",
        false,
        inputs.prestageLeftSupplyAmps != null ? inputs.prestageLeftSupplyAmps.in(Units.Amps) : 0.0);
    Robot.batteryLogger.reportCurrentUsage(
        "Prestage/Right",
        false,
        inputs.prestageRightSupplyAmps != null
            ? inputs.prestageRightSupplyAmps.in(Units.Amps)
            : 0.0);
  }

  public void setPrestageVoltage(Voltage volts) {
    io.setPrestageVoltage(volts);
  }

  public void setPrestageVelocity(AngularVelocity prestageVelo) {
    io.setPrestageVelocity(prestageVelo);
  }

  public void setOneVelo(AngularVelocity prestageVelo) {
    io.setOneVelo(prestageVelo);
  }
}
