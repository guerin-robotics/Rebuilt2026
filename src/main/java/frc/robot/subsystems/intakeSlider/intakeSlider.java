package frc.robot.subsystems.intakeSlider;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intakeSlider.io.IntakeSliderIOInputsAutoLogged;
import frc.robot.subsystems.intakeSlider.io.intakeSliderIO;
import org.littletonrobotics.junction.Logger;

public class intakeSlider extends SubsystemBase {

  private final intakeSliderIO io;
  private final IntakeSliderIOInputsAutoLogged inputs;

  public intakeSlider(intakeSliderIO io) {
    this.io = io;
    this.inputs = new IntakeSliderIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake Slider", inputs);
  }

  public void setSliderVoltage(Voltage volts) {
    io.setSliderVoltage(volts);
  }

  public void setSliderVelocity(AngularVelocity sliderVelo) {
    io.setSliderVelocity(sliderVelo);
  }

  public void setSliderPosition(double positionRotations) {
    io.setSliderPosition(positionRotations);
  }

  public void zeroSliderEncoder() {
    io.zeroSliderEncoder();
  }

  // Retract for pulse sequence
  public void intakeJostleByCurrent(
      AngularVelocity upVelocity,
      AngularVelocity downVelocity,
      double degreesDown,
      double seconds) {
    double currentPos = inputs.intakeSliderPosition;
    if (inputs.intakeSliderStatorCurrent
        < intakeSliderConstants.Mechanical.sliderJostleCurrentLimit) {
      io.setSliderVelocity(downVelocity);
    } else {
      setSliderPosition(currentPos + degreesDown);
      new WaitCommand(seconds);
    }
  }

  public void intakeJostleByPos(double degreesUp, double degreesDown) {
    io.setSliderPosition(degreesUp);
    new WaitCommand(0.25);
    io.setSliderPosition(degreesDown);
    new WaitCommand(0.25);
  }

  public void intakeHome(AngularVelocity homeVelo) {
    if (inputs.intakeSliderStatorCurrent > 0.5) {
      io.setSliderVelocity(homeVelo);
    } else {
      io.zeroSliderEncoder();
    }
  }
}
