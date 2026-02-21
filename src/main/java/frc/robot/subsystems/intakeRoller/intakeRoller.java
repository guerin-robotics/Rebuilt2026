package frc.robot.subsystems.intakeRoller;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intakeRoller.io.intakeRollerIO;

public class intakeRoller extends SubsystemBase {
  private final intakeRollerIO io;

  private final intakeRollerIO.intakeRollerIOInputs inputs =
      new intakeRollerIO.intakeRollerIOInputs();

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

  public void setRollerTorqueControl(AngularVelocity rollerVelo) {
    io.setRollerTorqueControl(rollerVelo);
  }
}
