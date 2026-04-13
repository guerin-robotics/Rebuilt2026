package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.lowerFeeder.LowerFeeder;
import frc.robot.subsystems.upperFeeder.UpperFeeder;

public class FeederCommands {

  public static Command setLowerFeederVoltage(LowerFeeder lowerFeeder, Voltage voltage) {
    return Commands.startEnd(
            () -> lowerFeeder.setLowerFeederVoltage(voltage), // Apply voltage
            () -> lowerFeeder.setLowerFeederVoltage(Volts.of(0)), // Stop on end
            lowerFeeder)
        .withName("FeederVoltage_" + voltage.in(Volts) + "V");
  }

  public static Command setUpperFeederVoltage(UpperFeeder feeder, Voltage voltage) {
    return Commands.startEnd(
            () -> feeder.setUpperFeederVoltage(voltage), // Apply voltage
            () -> feeder.setUpperFeederVoltage(Volts.of(0)), // Stop on end
            feeder)
        .withName("FeederVoltage_" + voltage.in(Volts) + "V");
  }

  public static Command stopLower(LowerFeeder feeder) {
    return Commands.runOnce(() -> feeder.setLowerFeederVelocity(RotationsPerSecond.of(0)), feeder)
        .withName("FeederStop");
  }

  public static Command stopUpper(UpperFeeder feeder) {
    return Commands.runOnce(() -> feeder.setUpperFeederVelocity(RotationsPerSecond.of(0)), feeder)
        .withName("FeederStop");
  }

  public static Command setLowerFeederVelocity(LowerFeeder feeder, AngularVelocity feederVelo) {
    return Commands.runOnce(() -> feeder.setLowerFeederVelocity(feederVelo), feeder)
        .withName("LowerFeederVelocity");
  }

  public static Command setUpperFeederVelocity(UpperFeeder feeder, AngularVelocity feederVelo) {
    return Commands.runOnce(() -> feeder.setUpperFeederVelocity(feederVelo), feeder)
        .withName("UpperFeederVelocity");
  }

  public static Command setLowerVelocityAfterWait(LowerFeeder feeder, AngularVelocity feederVelo) {
    return Commands.sequence(
            new WaitCommand(HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
            setLowerFeederVelocity(feeder, feederVelo))
        .withName("LowerFeederVelocityAfterWait");
  }

  public static Command setUpperVelocityAfterWait(UpperFeeder feeder, AngularVelocity feederVelo) {
    return Commands.sequence(
            new WaitCommand(HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
            setUpperFeederVelocity(feeder, feederVelo))
        .withName("UpperFeederVelocityAfterWait");
  }
}
