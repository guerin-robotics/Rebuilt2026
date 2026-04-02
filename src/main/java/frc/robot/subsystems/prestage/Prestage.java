package frc.robot.subsystems.prestage;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareConstants;
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

  public void setPrestageVelocity(AngularVelocity prestageVelo) {
    io.setPrestageVelocity(prestageVelo);
  }

  public void setOneVelo(AngularVelocity prestageVelo) {
    io.setOneVelo(prestageVelo);
  }

  // ==================== COMMAND FACTORIES ====================

  /** Runs the prestage at a specific voltage; stops (0V) when the command ends. */
  public Command setPrestageVoltageCommand(Voltage voltage) {
    return Commands.startEnd(
            () -> setPrestageVoltage(voltage), () -> setPrestageVoltage(Volts.of(0)), this)
        .withName("PrestageVoltage_" + voltage.in(Volts) + "V");
  }

  /** Stops the prestage immediately. */
  public Command stopCommand() {
    return Commands.runOnce(() -> setPrestageVelocity(RotationsPerSecond.of(0)), this)
        .withName("PrestageStop");
  }

  /** Sets the prestage to a given velocity (instant). */
  public Command setPrestageVelocityCommand(AngularVelocity prestageVelo) {
    return Commands.runOnce(() -> setPrestageVelocity(prestageVelo), this);
  }

  /** Sets one motor to a given velocity (instant). */
  public Command setOneVeloCommand(AngularVelocity velo) {
    return Commands.runOnce(() -> setOneVelo(velo), this);
  }

  /** Runs the prestage at idle speed continuously. */
  public Command prestageIdleCommand() {
    return Commands.run(
        () -> setPrestageVelocity(HardwareConstants.CompConstants.Velocities.prestageIdleVelocity));
  }
}
