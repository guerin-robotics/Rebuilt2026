package frc.robot.subsystems.intakeRoller;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HardwareConstants;
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

  public void setRollerVoltage(Voltage volts) {
    io.setRollerVoltage(volts);
  }

  public void setRollerVelocity(AngularVelocity rollerVelo) {
    io.setRollerVelocity(rollerVelo);
  }

  // ==================== COMMAND FACTORIES ====================

  /** Runs the roller at a specific voltage; stops (0V) when the command ends. */
  public Command setRollerVoltageCommand(Voltage voltage) {
    return Commands.startEnd(
        () -> setRollerVoltage(voltage), () -> setRollerVoltage(Volts.of(0)), this);
  }

  /** Sets the roller to a given velocity (instant). */
  public Command setRollerVelocityCommand(AngularVelocity rollerVelo) {
    return Commands.runOnce(() -> setRollerVelocity(rollerVelo), this);
  }

  /** Stops the intake roller immediately. */
  public Command stopIntakeRollerCommand() {
    return Commands.runOnce(() -> setRollerVelocity(RotationsPerSecond.of(0)), this);
  }

  /** Waits for flywheel spinup, then sets roller voltage. */
  public Command setVoltageAfterWaitCommand(Voltage voltage) {
    return Commands.sequence(
        new WaitCommand(HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
        setRollerVoltageCommand(voltage));
  }
}
