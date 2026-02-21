package frc.robot.subsystems.transport.io;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface TransportIO {

  @AutoLog
  public class TransportIOInputs {
    public Voltage TransportVoltage;
    public Current TransportStatorAmps;
    public Current TransportSupplyAmps;
    public AngularVelocity TransportMotorVelocity;
    public Temperature TransportMotorTemperature;
  }

  public default void updateInputs(TransportIOInputs inputs) {}

  public default void setTransportVoltage(Voltage volts) {}

  public default void setTransportSpeed(AngularVelocity speed) {}

  public default void setTransportTorque(AngularVelocity transportVelo) {}
}
