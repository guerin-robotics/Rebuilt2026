package frc.robot.subsystems.upperFeeder.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface UpperFeederIO {

  @AutoLog
  public static class UpperFeederIOInputs {
    public Voltage upperFeederVoltage = Volts.of(0);
    public Current upperFeederStatorAmps = Amps.of(0);
    public Current upperFeederSupplyAmps = Amps.of(0);
    /** Revolutions per minute. */
    public double upperFeederMotorVelocity = 0.0;
    /** Degrees Celsius. */
    public double upperFeederMotorTemperature = 0.0;

    /** Revolutions per minute. */
    public double upperFeederClosedLoopReference = 0.0;
    /** Revolutions per minute. */
    public double upperFeederClosedLoopError = 0.0;
    /** Degrees. */
    public double upperFeederPos = 0.0;
  }

  public default void updateInputs(UpperFeederIOInputs inputs) {}

  public default void setUpperFeederVoltage(Voltage volts) {}

  public default void setUpperFeederVelocity(AngularVelocity feederVelo) {}
}
