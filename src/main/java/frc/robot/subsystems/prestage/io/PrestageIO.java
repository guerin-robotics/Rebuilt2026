package frc.robot.subsystems.prestage.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface PrestageIO {

  @AutoLog
  public static class PrestageIOInputs {
    public Voltage prestageLeftVoltage = Volts.of(0);
    public Current prestageLeftStatorAmps = Amps.of(0);
    public Current prestageLeftSupplyAmps = Amps.of(0);
    public Voltage prestageRightVoltage = Volts.of(0);
    public Current prestageRightStatorAmps = Amps.of(0);
    public Current prestageRightSupplyAmps = Amps.of(0);

    /** Revolutions per minute. */
    public double prestageLeftVelocity = 0.0;
    /** Revolutions per minute. */
    public double prestageRightVelocity = 0.0;

    /** Degrees Celsius. */
    public double prestageLeftTemperature = 0.0;
    /** Degrees Celsius. */
    public double prestageRightTemperature = 0.0;

    /** Revolutions per minute. */
    public double prestageLeftClosedLoopReference = 0.0;

    /** Revolutions per minute. */
    public double prestageLeftClosedLoopError = 0.0;

    /** Degrees. */
    public double prestageLeftPos = 0.0;
    /** Degrees. */
    public double prestageRightPos = 0.0;
  }

  public default void updateInputs(PrestageIOInputs inputs) {}

  public default void setPrestageVoltage(Voltage volts) {}

  public default void setPrestageVelocity(AngularVelocity prestageVelo) {}
}
