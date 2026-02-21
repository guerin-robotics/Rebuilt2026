package frc.robot.subsystems.intakeRoller;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intakeRoller.io.intakeRollerIO;
import frc.robot.subsystems.intakeRoller.io.intakeRollerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class intakeRoller extends SubsystemBase {
  private final intakeRollerIO io;

  private final intakeRollerIOInputsAutoLogged inputs;

  public intakeRoller(intakeRollerIO io) {
    this.io = io;
    this.inputs = new intakeRollerIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake Roller", inputs);
  }

  public void setIntakeRollerVoltage(Voltage volts) {
    io.setIntakeRollerVoltage(volts);
  }

  public void setRollerTorqueControl(AngularVelocity rollerVelo) {
    io.setRollerTorqueControl(rollerVelo);
  }
}
