package frc.robot.subsystems.prestage.io;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface PrestageIO {

  @AutoLog
  public static class PrestageIOInputs {
    public Voltage prestageVoltage;
    public Current prestageStatorAmps;
    public Current prestageSupplyAmps;

    public AngularVelocity prestageVelocity;

    public Temperature prestageTemperature;

    public AngularVelocity prestageClosedLoopReference;

    public AngularVelocity prestageClosedLoopError;

    public Angle prestagePos;
  }

  public default void updateInputs(PrestageIOInputs inputs) {}

  public default void setPrestageVoltage(Voltage volts) {}

  public default void setPrestageVelocity(AngularVelocity prestageVelo) {}

  public default void setOneVelo(AngularVelocity prestageVelo) {}
}
