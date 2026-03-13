package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.feeder.Feeder;

public class FeederCommands {

  public static Command setFeederVoltage(Feeder feeder, Voltage voltage) {
    return Commands.startEnd(
            () -> feeder.setFeederVoltage(voltage), // Apply voltage
            () -> feeder.setFeederVoltage(Volts.of(0)), // Stop on end
            feeder)
        .withName("FeederVoltage_" + voltage.in(Volts) + "V");
  }

  public static Command stop(Feeder feeder) {
    return Commands.runOnce(() -> feeder.setFeederVoltage(Volts.of(0)), feeder)
        .withName("FeederStop");
  }

  public static Command setFeederVelocity(Feeder feeder, AngularVelocity feederVelo) {
    return Commands.runOnce(() -> feeder.setFeederVelocity(feederVelo), feeder);
  }

  public static Command setVelocityAfterWait(Feeder feeder, AngularVelocity feederVelo) {
    return Commands.sequence(new WaitCommand(0.5), setFeederVelocity(feeder, feederVelo))
        .finallyDo(() -> feeder.setFeederVelocity(RotationsPerSecond.of(0)));
  }
}
