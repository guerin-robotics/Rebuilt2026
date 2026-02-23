package frc.robot.subsystems.intakeSlider;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HardwareConstants;
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

  public void setIntakeRotations(double rotations) {
    io.setIntakeRotations(rotations);
  }

  public void setIntakeSliderVelocityTorque(AngularVelocity sliderVelo) {
    io.setIntakeSliderVelocityTorque(sliderVelo);
  }

  public void zeroMotor() {
    io.zeroMotor();
  }

  // Retract for pulse sequence
  public void intakeJostleByCurrent(
      AngularVelocity retractVelo, double extensionInches, double seconds) {
    double currentPos =
        inputs.intakeSliderPosition * intakeSliderConstants.Mechanical.rotationsPerInch;
    if (inputs.intakeSliderStatorCurrent < 50) {
      io.setIntakeSliderVoltage(HardwareConstants.TestVoltages.intakeSliderTestVoltageIn);
    } else {
      io.setIntakeInch(currentPos + extensionInches);
      new WaitCommand(seconds);
    }
  }

  public void intakeHome(AngularVelocity homeVelo) {
    if (inputs.intakeSliderStatorCurrent > 0.5) {
      io.setIntakeSliderVelocityTorque(homeVelo);
    } else {
      io.zeroMotor();
    }
  }
}
