package frc.robot.subsystems.upperFeeder.io;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface UpperFeederIO {

  @AutoLog
  public static class UpperFeederIOInputs {
    public Voltage upperFeederVoltage;
    public Current upperFeederStatorAmps;
    public Current upperFeederSupplyAmps;
    public AngularVelocity upperFeederMotorVelocity;
    public Temperature upperFeederMotorTemperature;
    public AngularVelocity upperFeederClosedLoopReference;
    public AngularVelocity upperFeederClosedLoopError;
    public Angle upperFeederPos;
  }

  public default void updateInputs(UpperFeederIOInputs inputs) {}

  public default void setUpperFeederVoltage(Voltage volts) {}

  public default void setUpperFeederVelocity(AngularVelocity feederVelo) {}
}
