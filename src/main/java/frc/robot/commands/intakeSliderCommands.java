package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intakeSlider.intakeSlider;

public class intakeSliderCommands {

  public static Command setSliderVoltage(intakeSlider intakeSlider, Voltage voltage) {
    return Commands.startEnd(
        () -> intakeSlider.setSliderVoltage(voltage),
        () -> intakeSlider.setSliderVoltage(Volts.of(0)),
        intakeSlider);
  }

  public static Command setSliderVelocity(intakeSlider intakeSlider, AngularVelocity sliderVelo) {
    return Commands.runOnce(
        () -> intakeSlider.setSliderVelocity(sliderVelo),
        intakeSlider);
  }

  public static Command stopSlider(intakeSlider intakeSlider) {
    return Commands.runOnce(() -> intakeSlider.setSliderVoltage(Volts.of(0)), intakeSlider);
  }

  public static Command setSliderRotations(intakeSlider intakeSlider, double angleRotations) {
    return Commands.runOnce(() -> intakeSlider.setSliderPosition(angleRotations), intakeSlider);
  }

  public static Command jostleSliderByCurrent(
      intakeSlider intakeSlider,
      AngularVelocity upVelocity,
      AngularVelocity downVelocity,
      double degreesDown,
      double seconds) {
    return Commands.startEnd(
        () -> intakeSlider.intakeJostleByCurrent(upVelocity, downVelocity, degreesDown, seconds),
        () -> intakeSlider.setSliderVoltage(Volts.of(0)),
        intakeSlider);
  }

  public static Command zeroSlider(intakeSlider intakeSlider) {
    return Commands.runOnce(() -> intakeSlider.zeroSliderEncoder());
  }
}
