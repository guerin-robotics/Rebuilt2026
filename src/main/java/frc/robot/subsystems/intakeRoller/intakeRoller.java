package frc.robot.subsystems.intakeRoller;

import frc.robot.subsystems.intakeRoller.io.intakeRollerIO;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Voltage;

public class intakeRoller extends SubsystemBase {
  private final intakeRollerIO io;

  private final intakeRollerIO.intakeRollerIOInputs inputs = new intakeRollerIO.intakeRollerIOInputs();

  public intakeRoller(intakeRollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void setIntakeRollerVoltage(Voltage volts) {
    io.setIntakeRollerVoltage(volts);
  }

}
