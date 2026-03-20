package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intakeRoller.intakeRoller;

public class intakeRollerCommands {

  public static Command setRollerVoltage(intakeRoller intakeRoller, Voltage voltage) {
    return Commands.startEnd(
        () -> intakeRoller.setRollerVoltage(voltage),
        () -> intakeRoller.setRollerVoltage(Volts.of(0)),
        intakeRoller);
  }

  public static Command setRollerVelocity(intakeRoller intakeRoller, AngularVelocity rollerVelo) {
    return Commands.runOnce(() -> intakeRoller.setRollerVelocity(rollerVelo), intakeRoller);
  }

  public static Command stopIntakeRoller(intakeRoller intakeRoller) {
    return Commands.runOnce(
        () -> intakeRoller.setRollerVelocity(RotationsPerSecond.of(0)), intakeRoller);
  }
}
