package frc.robot.subsystems.intakeSlider;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  public void setIntakeSliderVoltage(Voltage volts) {
    io.setIntakeSliderVoltage(volts);
  }

  public void setIntakeInch(double inches) {
    io.setIntakeInch(inches);
  }

  public void setIntakeSliderVelocityTorque(AngularVelocity sliderVelo) {
    io.setIntakeSliderVelocityTorque(sliderVelo);
  }

  // Retract for pulse sequence
  public void intakeRetract(AngularVelocity retractVelo, double extension) {
    double currentPos = inputs.intakeSliderPosition;
    if (inputs.intakeSliderStatorCurrent < 2.0) {
      io.setIntakeSliderVelocityTorque(retractVelo);
    } else {
      io.setIntakePositionTorque(currentPos + extension);
    }
  }

  public void intakeHome(AngularVelocity homeVelo) {
    if (inputs.intakeSliderStatorCurrent > 0.5) {
      io.setIntakeSliderVelocityTorque(homeVelo);
    } else {
      io.zeroMotor();
    }
  }

  public void zeroMotor() {
    io.zeroMotor();
  }

  public void intakeWait(double seconds) {
    io.intakeWait(seconds);
  }
}
