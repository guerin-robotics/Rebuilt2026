package frc.robot.subsystems.intakeSlider;

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

  public void setIntakePos(double setpoint) {
    io.setIntakePos(setpoint);
  }

  public void intakeRetract(double retractVelo, double extension) {
    io.intakeRetract(retractVelo, extension);
  }

  public void setIntakePosForPulse(double rotations) {
    io.setIntakePosForPulse(rotations);
  }

  public void intakeWait(double seconds) {
    io.intakeWait(seconds);
  }
}
