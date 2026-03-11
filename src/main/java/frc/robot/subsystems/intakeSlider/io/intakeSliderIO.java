package frc.robot.subsystems.intakeSlider.io;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface intakeSliderIO {

  @AutoLog
  public static class IntakeSliderIOInputs {
    public Voltage intakeSliderVoltage;
    public Current intakeSliderSupplyCurrent;
    public double intakeSliderStatorCurrent;
    public Temperature intakeSliderTemperature;
    public AngularVelocity intakeSliderVelocity;
    public double intakeSliderPosition;
    public AngularVelocity intakeSliderClosedLoopReference;
    public AngularVelocity intakeSliderClosedLoopError;
  }

  public default void updateInputs(IntakeSliderIOInputs inputs) {}

  public default void setSliderVoltage(Voltage volts) {}

  public default void setSliderVelocity(AngularVelocity velocity) {}

  public default void setSliderPosition(double position) {}

  public default void zeroSliderEncoder() {}
}
