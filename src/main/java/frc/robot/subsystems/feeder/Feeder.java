package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HardwareConstants;
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

  // ==================== COMMAND FACTORIES ====================

  /** Runs the feeder at a specific voltage; stops (0V) when the command ends. */
  public Command setFeederVoltageCommand(Voltage voltage) {
    return Commands.startEnd(
            () -> setFeederVoltage(voltage), () -> setFeederVoltage(Volts.of(0)), this)
        .withName("FeederVoltage_" + voltage.in(Volts) + "V");
  }

  /** Stops the feeder immediately. */
  public Command stopCommand() {
    return Commands.runOnce(() -> setFeederVelocity(RotationsPerSecond.of(0)), this)
        .withName("FeederStop");
  }

  /** Sets the feeder to a given velocity (instant). */
  public Command setFeederVelocityCommand(AngularVelocity feederVelo) {
    return Commands.runOnce(() -> setFeederVelocity(feederVelo), this);
  }

  /** Waits for flywheel spinup, then sets feeder velocity. */
  public Command setVelocityAfterWaitCommand(AngularVelocity feederVelo) {
    return Commands.sequence(
        new WaitCommand(HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
        setFeederVelocityCommand(feederVelo));
  }
}
