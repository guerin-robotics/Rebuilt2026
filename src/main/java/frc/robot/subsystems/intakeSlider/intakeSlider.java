package frc.robot.subsystems.intakeSlider;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
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

  public void setIntakePos(double rotationChange) {
    io.setIntakePos(rotationChange);
  }

  public void intakeWait(double seconds) {
    Timer intakeTimer = new Timer();
    intakeTimer.start();
    if (intakeTimer.get() > 0.5) {
      intakeTimer.stop();
    }
  }

}
