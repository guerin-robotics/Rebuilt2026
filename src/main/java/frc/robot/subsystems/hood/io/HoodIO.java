package frc.robot.subsystems.hood.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface HoodIO {

  @AutoLog
  public static class HoodIOInputs {
    // public double servoSpeed;
    // public double servoPos;
    public Voltage hoodVoltage;
    public Current hoodSupplyCurrent;
    public Current hoodStatorCurrent;
    public Temperature hoodTemperature;
    public AngularVelocity hoodVelocity;
    public double hoodPosition;
    public double hoodClosedLoopReference;
    public double hoodClosedLoopError;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  /**
   * Sets the hood position.
   *
   * @param position in rotations
   */
  public default void setHoodPos(double position) {}

  public default void stopHood() {}
}
