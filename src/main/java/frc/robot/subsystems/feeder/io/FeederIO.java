package frc.robot.subsystems.feeder.io;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {

  @AutoLog
  public static class FeederIOInputs {
    public Voltage feederVoltage;
    public Current feederStatorAmps;
    public Current feederSupplyAmps;
    public AngularVelocity feederMotorVelocity;
    public Temperature feederMotorTemperature;
    public AngularVelocity feederClosedLoopReference;
    public AngularVelocity feederClosedLoopError;
    public Angle feederPos;
  }

  public default void updateInputs(FeederIOInputs inputs) {}

  public default void setFeederVoltage(Voltage volts) {}

  public default void setFeederVelocity(AngularVelocity feederVelo) {}
}
