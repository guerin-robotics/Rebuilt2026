package frc.robot.subsystems.intakePivot.io;

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

/**
 * Hardware abstraction interface for the intake pivot mechanism.
 *
 * <p>Defines the contract for reading sensors and controlling the pivot motor. Implementations
 * include {@link IntakePivotIOReal} for real hardware and {@link IntakePivotIOSim} for simulation.
 */
public interface IntakePivotIO {

  @AutoLog
  public static class IntakePivotIOInputs {
    public Voltage intakePivotVoltage = Volts.of(0);
    public Current intakePivotSupplyCurrent = Amps.of(0);
    public Current intakePivotStatorCurrent = Amps.of(0);
    public Temperature intakePivotTemperature = Celsius.of(0);
    public AngularVelocity intakePivotVelocity = RotationsPerSecond.of(0);
    public Angle intakePivotPosition = Rotations.of(0);
    public double intakePivotClosedLoopReference;
    public double intakePivotClosedLoopError;
  }

  public default void updateInputs(IntakePivotIOInputs inputs) {}

  public default void setPivotVoltage(Voltage volts) {}

  public default void setPivotVelocity(AngularVelocity velocity) {}

  public default void setPivotPosition(Angle position) {}

  public default void zeroPivotEncoder() {}
}
