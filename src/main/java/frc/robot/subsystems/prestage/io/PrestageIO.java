package frc.robot.subsystems.prestage.io;

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

public interface PrestageIO {

  @AutoLog
  public static class PrestageIOInputs {
    public Voltage prestageLeftVoltage = Volts.of(0);
    public Current prestageLeftStatorAmps = Amps.of(0);
    public Current prestageLeftSupplyAmps = Amps.of(0);
    public Voltage prestageRightVoltage = Volts.of(0);
    public Current prestageRightStatorAmps = Amps.of(0);
    public Current prestageRightSupplyAmps = Amps.of(0);

    public AngularVelocity prestageLeftVelocity = RotationsPerSecond.of(0);
    public AngularVelocity prestageRightVelocity = RotationsPerSecond.of(0);

    public Temperature prestageLeftTemperature = Celsius.of(0);
    public Temperature prestageRightTemperature = Celsius.of(0);

    public AngularVelocity prestageLeftClosedLoopReference = RotationsPerSecond.of(0);
    public AngularVelocity prestageRightClosedLoopReference = RotationsPerSecond.of(0);

    public AngularVelocity prestageLeftClosedLoopError = RotationsPerSecond.of(0);
    public AngularVelocity prestageRightClosedLoopError = RotationsPerSecond.of(0);

    public Angle prestageLeftPos = Rotations.of(0);
    public Angle prestageRightPos = Rotations.of(0);
  }

  public default void updateInputs(PrestageIOInputs inputs) {}

  public default void setPrestageVoltage(Voltage volts) {}

  public default void setPrestageVelocity(AngularVelocity prestageVelo) {}
}
