package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intakeSlider.intakeSlider;
import frc.robot.subsystems.intakeSlider.intakeSliderConstants;

public class intakeSliderCommands {

  public static Command setSliderVoltage(intakeSlider intakeSlider, Voltage voltage) {
    return Commands.startEnd(
        () -> intakeSlider.setSliderVoltage(voltage),
        () -> intakeSlider.setSliderVoltage(Volts.of(0)),
        intakeSlider);
  }

  public static Command stopIntakeSlider(intakeSlider intakeSlider) {
    return Commands.runOnce(() -> intakeSlider.setSliderVoltage(Volts.of(0)), intakeSlider);
  }

  public static Command setIntakeInch(intakeSlider intakeSlider, double inches) {
    return Commands.startEnd(
        () -> intakeSlider.setSliderInch(inches*intakeSliderConstants.Mechanical.rotationsPerInch),
        () -> intakeSlider.setSliderVoltage(Volts.of(0)),
        intakeSlider);
  }

  public static Command intakeJostleByCurrent(
      intakeSlider intakeSlider,
      AngularVelocity retractVelo,
      double extensionInches,
      double seconds) {
    return Commands.startEnd(
        () -> intakeSlider.intakeJostleByCurrent(retractVelo, extensionInches, seconds),
        () -> intakeSlider.setSliderVoltage(Volts.of(0)),
        intakeSlider);
  }

  public static Command zeroSlider(intakeSlider intakeSlider) {
    return Commands.runOnce(() -> intakeSlider.zeroSliderMotor());
  }

  public static Command setSliderVelocity(intakeSlider intakeSlider, AngularVelocity sliderVelo) {
    return Commands.startEnd(
        () -> intakeSlider.setSliderVelocity(sliderVelo),
        () -> intakeSlider.setSliderVelocity(RotationsPerSecond.of(0)),
        intakeSlider);
  }
}
