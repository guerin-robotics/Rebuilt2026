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

  public static Command setIntakePos(intakeSlider intakeSlider, double inches) {
    return Commands.startEnd(
        () -> intakeSlider.setIntakeInch(inches),
        () -> intakeSlider.setIntakeSliderVoltage(Volts.of(0)),
        intakeSlider);
  }

  public static Command intakeRetract(
      intakeSlider intakeSlider, AngularVelocity retractVelo, double extension) {
    return Commands.startEnd(
        () -> intakeSlider.intakeRetract(retractVelo, extension),
        () -> intakeSlider.setIntakeSliderVoltage(Volts.of(0)),
        intakeSlider);
  }

  public static Command intakeWait(intakeSlider intakeSlider, double seconds) {
    return Commands.runOnce(() -> intakeSlider.intakeWait(seconds), intakeSlider);
  }

  public static Command pulseIntakeSlider(intakeSlider intakeSlider, double rotationChange) {
    return Commands.repeatingSequence(
        setIntakePos(intakeSlider, rotationChange), intakeWait(intakeSlider, 0.5));
  }

  public static Command pulseIntakeByCurrent(
      intakeSlider intakeSlider, AngularVelocity retractVelo, double extension) {
    return Commands.repeatingSequence(
        intakeRetract(intakeSlider, retractVelo, extension), intakeWait(intakeSlider, 0.5));
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
