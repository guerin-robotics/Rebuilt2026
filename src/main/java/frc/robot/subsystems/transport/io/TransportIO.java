package frc.robot.subsystems.transport.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface TransportIO {

  @AutoLog
  public class TransportIOInputs {
    public Voltage TransportVoltage = Volts.of(0);
    public Current TransportStatorAmps = Amps.of(0);
    public Current TransportSupplyAmps = Amps.of(0);
    public AngularVelocity TransportMotorVelocity = RotationsPerSecond.of(0);
    public Temperature TransportMotorTemperature = Celsius.of(0);
    public AngularVelocity transportClosedLoopReference = RotationsPerSecond.of(0);
    public AngularVelocity transportClosedLoopError = RotationsPerSecond.of(0);
    public Angle transportPos = Rotations.of(0);
  }

  public default void updateInputs(TransportIOInputs inputs) {}

  public default void setTransportVoltage(Voltage volts) {}

  public default void setTransportVelocity(AngularVelocity transportVelo) {}
}
