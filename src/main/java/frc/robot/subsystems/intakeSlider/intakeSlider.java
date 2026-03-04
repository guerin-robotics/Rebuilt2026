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

  public void setSliderVoltage(Voltage volts) {
    io.setSliderVoltage(volts);
  }

  public void setSliderInch(double inches) {
    io.setSliderInch(inches);
  }

  public void setSliderVelocity(AngularVelocity sliderVelo) {
    io.setSliderVelocity(sliderVelo);
  }

  public void zeroSliderMotor() {
    io.zeroSliderMotor();
  }

  // Retract for pulse sequence
  public void intakeJostleByCurrent(
      AngularVelocity retractVelo, double extensionInches, double seconds) {
    double currentPos =
        inputs.intakeSliderPosition * intakeSliderConstants.Mechanical.rotationsPerInch;
    if (inputs.intakeSliderStatorCurrent < 50) {
      io.setSliderVoltage(HardwareConstants.TestVoltages.intakeSliderTestVoltageIn);
    } else {
      io.setSliderInch(currentPos + extensionInches);
      new WaitCommand(seconds);
    }
  }

  public void intakeHome(AngularVelocity homeVelo) {
    if (inputs.intakeSliderStatorCurrent > 0.5) {
      io.setSliderVelocity(homeVelo);
    } else {
      io.zeroSliderMotor();
    }
  }
}
