package frc.robot.subsystems.intakeSlider;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intakeSlider.io.intakeSliderIO;

public class intakeSlider extends SubsystemBase {

  private final intakeSliderIO io;

  private final intakeSliderIO.IntakeSliderIOInputs inputs =
      new intakeSliderIO.IntakeSliderIOInputs();

  public intakeSlider(intakeSliderIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void setIntakeSliderVoltage(Voltage volts) {
    io.setIntakeSliderVoltage(volts);
  }

  public void setIntakeInch(double inches) {
    io.setIntakeInch(inches);
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

    }
  }

  public void zeroMotor() {
    io.zeroMotor();
  }

  public void intakeWait(double seconds) {
    io.intakeWait(seconds);
  }
}
