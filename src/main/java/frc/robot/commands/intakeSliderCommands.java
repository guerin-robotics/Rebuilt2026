package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intakeSlider.intakeSlider;

public class intakeSliderCommands {

  public static Command runIntakeForward(intakeSlider intakeSlider, Voltage voltage) {
    return Commands.startEnd(
        () -> intakeSlider.setIntakeSliderVoltage(voltage),
        () -> intakeSlider.setIntakeSliderVoltage(Volts.of(0)),
        intakeSlider);
  }

  public static Command stopIntakeSlider(intakeSlider intakeSlider) {
    return Commands.runOnce(() -> intakeSlider.setIntakeSliderVoltage(Volts.of(0)), intakeSlider);
  }

  public static Command intakeRetractUntilCurrent(
      intakeSlider intakeSlider, AngularVelocity retractVelo, double extensionInches, double seconds) {
    return Commands.startEnd(
        () -> intakeSlider.intakeRetractUntilCurrent(retractVelo, extensionInches, seconds),
        () -> intakeSlider.setIntakeSliderVoltage(Volts.of(0)),
        intakeSlider);
  }

  public static Command zeroIntake(intakeSlider intakeSlider) {
    return Commands.runOnce(() -> intakeSlider.zeroMotor());
  }

  public static Command runTorque(intakeSlider intakeSlider, AngularVelocity sliderVelo) {
    return Commands.startEnd(
        () -> intakeSlider.setIntakeSliderVelocityTorque(sliderVelo),
        () -> intakeSlider.setIntakeSliderVoltage(Volts.of(0)),
        intakeSlider);
  }
}
