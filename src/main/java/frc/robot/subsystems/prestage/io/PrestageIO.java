package frc.robot.subsystems.prestage.io;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface PrestageIO {

  @AutoLog
  public static class PrestageIOInputs {
    public Voltage prestageLeftVoltage;
    public Current prestageLeftStatorAmps;
    public Current prestageLeftSupplyAmps;
    public Voltage prestageRightVoltage;
    public Current prestageRightStatorAmps;
    public Current prestageRightSupplyAmps;

    public AngularVelocity prestageLeftVelocity;
    public AngularVelocity prestageRightVelocity;

    public Temperature prestageLeftTemperature;
    public Temperature prestageRightTemperature;

    public AngularVelocity prestageLeftClosedLoopReference;
    public AngularVelocity prestageRightClosedLoopReference;

    public AngularVelocity prestageLeftClosedLoopError;
    public AngularVelocity prestageRightClosedLoopError;

    public Angle prestageLeftPos;
    public Angle prestageRightPos;
  }

  public default void updateInputs(PrestageIOInputs inputs) {}

  public default void setPrestageVoltage(Voltage volts) {}

  public default void setPrestageVelocity(AngularVelocity prestageVelo) {}

  public default void setOneVelo(AngularVelocity prestageVelo) {}
}
