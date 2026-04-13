package frc.robot.subsystems.intakeRoller.io;

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

public interface intakeRollerIO {

  @AutoLog
  public static class intakeRollerIOInputs {
    public Voltage intakeRollerVoltage = Volts.of(0);
    public Current intakeRollerSupplyCurrent = Amps.of(0);
    public Current intakeRollerStatorCurrent = Amps.of(0);
    public Temperature intakeRollerTemperature = Celsius.of(0);
    public AngularVelocity intakeRollerVelocity = RotationsPerSecond.of(0);
    public AngularVelocity rollerClosedLoopReference = RotationsPerSecond.of(0);
    public AngularVelocity rollerClosedLoopError = RotationsPerSecond.of(0);
    public Angle rollerPos = Rotations.of(0);
    public Voltage intakeRollerFollowerVoltage = Volts.of(0);
    public Current intakeRollerFollowerSupplyCurrent = Amps.of(0);
    public Current intakeRollerFollowerStatorCurrent = Amps.of(0);
    public Temperature intakeRollerFollowerTemperature = Celsius.of(0);
    public AngularVelocity intakeRollerFollowerVelocity = RotationsPerSecond.of(0);
    public AngularVelocity rollerFollowerClosedLoopReference = RotationsPerSecond.of(0);
    public AngularVelocity rollerFollowerClosedLoopError = RotationsPerSecond.of(0);
    public Angle rollerFollowerPos = Rotations.of(0);
  }

  public default void updateInputs(intakeRollerIOInputs inputs) {}

  public default void setRollerVoltage(Voltage volts) {}

  public default void setRollerVelocity(AngularVelocity rollerVelo) {}
}
