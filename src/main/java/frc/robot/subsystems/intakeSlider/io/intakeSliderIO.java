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

  public default void setIntakeSliderVoltage(Voltage volts) {}

  public default void setIntakeInch(double inches) {}

  public default void setIntakeSliderVelocityTorque(AngularVelocity velocity) {}

  public default void zeroMotor() {}

  public default void intakeRetractUntilCurrent(double retractVolts, double extension) {}

  public default void intakeHome() {}
}
