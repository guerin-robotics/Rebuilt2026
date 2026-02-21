package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intakeRoller.intakeRoller;

public class intakeRollerCommands {

  public static Command runIntakeRoller(intakeRoller intakeRoller, Voltage voltage) {
    return Commands.startEnd(
        () -> intakeRoller.setIntakeRollerVoltage(voltage),
        () -> intakeRoller.setIntakeRollerVoltage(Volts.of(0)),
        intakeRoller);
  }

  public static Command stopIntakeRoller(intakeRoller intakeRoller) {
    return Commands.runOnce(() -> intakeRoller.setIntakeRollerVoltage(Volts.of(0)), intakeRoller);
  }

  public static Command runTorque(intakeRoller intakeRoller, AngularVelocity rollerVelo) {
    return Commands.startEnd(
        () -> intakeRoller.setRollerTorqueControl(rollerVelo),
        () -> intakeRoller.setRollerTorqueControl(RotationsPerSecond.of(0)),
        intakeRoller);
  }
}
