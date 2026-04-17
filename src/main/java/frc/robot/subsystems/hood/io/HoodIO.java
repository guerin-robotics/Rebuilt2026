package frc.robot.subsystems.hood.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  @AutoLog
  public static class HoodIOInputs {
    public Voltage hoodVoltage = Volts.of(0);
    public Current hoodSupplyCurrent = Amps.of(0);
    public Current hoodStatorCurrent = Amps.of(0);
    public Temperature hoodTemperature = Celsius.of(0);
    public AngularVelocity hoodVelocity = DegreesPerSecond.of(0);
    public Angle hoodPosition = Degrees.of(0);
    public Angle hoodClosedLoopReference = Degrees.of(0);
    public Angle hoodClosedLoopError = Degrees.of(0);
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  /**
   * Sets the hood position.
   *
   * @param position in rotations
   */
  public default void setHoodPos(Angle position) {}
}
