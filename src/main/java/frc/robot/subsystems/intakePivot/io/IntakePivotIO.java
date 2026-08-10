package frc.robot.subsystems.intakePivot.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
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
    /** Degrees Celsius. */
    public double intakePivotTemperature = 0.0;

    /** Revolutions per minute. */
    public double intakePivotVelocity = 0.0;
    /** Degrees. */
    public double intakePivotPosition = 0.0;

    /** Degrees. The pivot is position-controlled; setPivotVelocity is never bound. */
    public double intakePivotClosedLoopReference = 0.0;

    /** Degrees. */
    public double intakePivotClosedLoopError = 0.0;
  }

  public default void updateInputs(IntakePivotIOInputs inputs) {}

  public default void setPivotVoltage(Voltage volts) {}

  public default void setPivotVelocity(AngularVelocity velocity) {}

  public default void setPivotPosition(Angle position) {}

  public default void zeroPivotEncoder() {}
}
