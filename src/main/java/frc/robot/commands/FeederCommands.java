package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.feeder.Feeder;

public class FeederCommands {

  public static Command runFeederVoltage(Feeder feeder, Voltage voltage) {
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

  public static Command runTorque(Feeder feeder, AngularVelocity feederVelo) {
    return Commands.startEnd(
        () -> feeder.setFeederTorqueControl(feederVelo),
        () -> feeder.setFeederTorqueControl(RotationsPerSecond.of(0)),
        feeder);
  }
}
