package frc.robot.subsystems.lowerFeeder.io;

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

public interface LowerFeederIO {

  @AutoLog
  public static class LowerFeederIOInputs {
    public Voltage lowerFeederVoltage = Volts.of(0);
    public Current lowerFeederStatorAmps = Amps.of(0);
    public Current lowerFeederSupplyAmps = Amps.of(0);
    public AngularVelocity lowerFeederMotorVelocity = RotationsPerSecond.of(0);
    public Temperature lowerFeederMotorTemperature = Celsius.of(0);
    public AngularVelocity lowerFeederClosedLoopReference = RotationsPerSecond.of(0);
    public AngularVelocity lowerFeederClosedLoopError = RotationsPerSecond.of(0);
    public Angle lowerFeederPos = Rotations.of(0);
  }

  public default void updateInputs(LowerFeederIOInputs inputs) {}

  public default void setLowerFeederVoltage(Voltage volts) {}

  public default void setLowerFeederVelocity(AngularVelocity feederVelo) {}
}
