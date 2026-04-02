package frc.robot.subsystems.lowerFeeder.io;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface LowerFeederIO {

  @AutoLog
  public static class LowerFeederIOInputs {
    public Voltage lowerFeederVoltage;
    public Current lowerFeederStatorAmps;
    public Current lowerFeederSupplyAmps;
    public AngularVelocity lowerFeederMotorVelocity;
    public Temperature lowerFeederMotorTemperature;
    public AngularVelocity lowerFeederClosedLoopReference;
    public AngularVelocity lowerFeederClosedLoopError;
    public Angle lowerFeederPos;
  }

  public default void updateInputs(LowerFeederIOInputs inputs) {}

  public default void setLowerFeederVoltage(Voltage volts) {}

  public default void setLowerFeederVelocity(AngularVelocity feederVelo) {}
}
