package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.prestage.Prestage;

public class PrestageCommands {

  public static Command runPrestageVoltage(Prestage prestage, Voltage voltage) {
    return Commands.startEnd(
            () -> prestage.setPrestageVoltage(voltage), // Apply voltage
            () -> prestage.setPrestageVoltage(Volts.of(0)), // Stop on end
            prestage)
        .withName("PrestageVoltage_" + voltage.in(Volts) + "V");
  }

  public static Command stop(Prestage prestage) {
    return Commands.runOnce(() -> prestage.setPrestageVoltage(Volts.of(0)), prestage)
        .withName("PrestageStop");
  }

  public static Command runTorque(Prestage prestage, AngularVelocity prestageVelo) {
    return Commands.startEnd(
      () -> prestage.setPrestageTorque(prestageVelo),
      () -> prestage.setPrestageTorque(RotationsPerSecond.of(0)),
      prestage
    );
  }
}
