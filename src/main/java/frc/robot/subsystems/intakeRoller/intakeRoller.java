package frc.robot.subsystems.intakeRoller;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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

    // Report energy usage
    Robot.batteryLogger.reportCurrentUsage(
        "IntakeRoller",
        inputs.intakeRollerSupplyCurrent != null ? inputs.intakeRollerSupplyCurrent.in(Amps) : 0.0);
  }

  public void setRollerVoltage(Voltage volts) {
    io.setRollerVoltage(volts);
  }

  public void setRollerVelocity(AngularVelocity rollerVelo) {
    io.setRollerVelocity(rollerVelo);
  }
}
