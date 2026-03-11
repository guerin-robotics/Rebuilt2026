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

  // Checks position and defines velocity request accordingly
  // Encoder provides position in -1.0 to 1.0, so the degree value must be divided by 360 to put it
  // in this range
  public void setSliderDegree(
      double angleDegrees, AngularVelocity velocityUp, AngularVelocity velocityDown) {
    if (inputs.intakeSliderPosition < (angleDegrees / 360)) {
      io.setSliderVelocity(velocityUp);
    } else if (inputs.intakeSliderPosition > (angleDegrees / 360)) {
      io.setSliderVelocity(velocityDown);
    }
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
      setSliderDegree(((360 * currentPos) + degreesDown), downVelocity, upVelocity);
      new WaitCommand(seconds);
    }
  }

  public void intakeHome(AngularVelocity homeVelo) {
    if (inputs.intakeSliderStatorCurrent > 0.5) {
      io.setSliderVelocity(homeVelo);
    } else {
      io.zeroSliderEncoder();
    }
  }
}
