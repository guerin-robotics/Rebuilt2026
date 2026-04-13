package frc.robot.subsystems.upperFeeder.io;

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

public interface UpperFeederIO {

  @AutoLog
  public static class UpperFeederIOInputs {
    public Voltage upperFeederVoltage = Volts.of(0);
    public Current upperFeederStatorAmps = Amps.of(0);
    public Current upperFeederSupplyAmps = Amps.of(0);
    public AngularVelocity upperFeederMotorVelocity = RotationsPerSecond.of(0);
    public Temperature upperFeederMotorTemperature = Celsius.of(0);
    public AngularVelocity upperFeederClosedLoopReference = RotationsPerSecond.of(0);
    public AngularVelocity upperFeederClosedLoopError = RotationsPerSecond.of(0);
    public Angle upperFeederPos = Rotations.of(0);
  }

  public default void updateInputs(UpperFeederIOInputs inputs) {}

  public default void setUpperFeederVoltage(Voltage volts) {}

  public default void setUpperFeederVelocity(AngularVelocity feederVelo) {}
}
