package frc.robot.subsystems.transport.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface TransportIO {

  @AutoLog
  public class TransportIOInputs {
    public Voltage TransportVoltage = Volts.of(0);
    public Current TransportStatorAmps = Amps.of(0);
    public Current TransportSupplyAmps = Amps.of(0);
    /** Revolutions per minute. */
    public double TransportMotorVelocity = 0.0;
    /** Degrees Celsius. */
    public double TransportMotorTemperature = 0.0;

    /** Revolutions per minute. */
    public double transportClosedLoopReference = 0.0;
    /** Revolutions per minute. */
    public double transportClosedLoopError = 0.0;
    /** Degrees. */
    public double transportPos = 0.0;
  }

  public default void updateInputs(TransportIOInputs inputs) {}

  public default void setTransportVoltage(Voltage volts) {}

  public default void setTransportVelocity(AngularVelocity transportVelo) {}
}
