package frc.robot.subsystems.hood.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  @AutoLog
  public static class HoodIOInputs {
    public Voltage hoodVoltage = Volts.of(0);
    public Current hoodSupplyCurrent = Amps.of(0);
    public Current hoodStatorCurrent = Amps.of(0);
    /** Degrees Celsius. */
    public double hoodTemperature = 0.0;

    /** Revolutions per minute. */
    public double hoodVelocity = 0.0;
    /** Degrees. */
    public double hoodPosition = 0.0;
    /** Degrees. */
    public double hoodClosedLoopReference = 0.0;
    /** Degrees. */
    public double hoodClosedLoopError = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  /**
   * Sets the hood position.
   *
   * @param position in rotations
   */
  public default void setHoodPos(Angle position) {}
}
