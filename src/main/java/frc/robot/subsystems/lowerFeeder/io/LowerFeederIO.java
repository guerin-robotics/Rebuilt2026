package frc.robot.subsystems.lowerFeeder.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface LowerFeederIO {

  @AutoLog
  public static class LowerFeederIOInputs {
    public Voltage lowerFeederVoltage = Volts.of(0);
    public Current lowerFeederStatorAmps = Amps.of(0);
    public Current lowerFeederSupplyAmps = Amps.of(0);
    /** Revolutions per minute. */
    public double lowerFeederMotorVelocity = 0.0;
    /** Degrees Celsius. */
    public double lowerFeederMotorTemperature = 0.0;

    /** Revolutions per minute. */
    public double lowerFeederClosedLoopReference = 0.0;
    /** Revolutions per minute. */
    public double lowerFeederClosedLoopError = 0.0;
    /** Degrees. */
    public double lowerFeederPos = 0.0;
  }

  public default void updateInputs(LowerFeederIOInputs inputs) {}

  public default void setLowerFeederVoltage(Voltage volts) {}

  public default void setLowerFeederVelocity(AngularVelocity feederVelo) {}
}
