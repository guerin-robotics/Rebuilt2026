package frc.robot.subsystems.intakePivot.io;

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
    public Voltage intakePivotVoltage;
    public Current intakePivotSupplyCurrent;
    public Current intakePivotStatorCurrent;
    public Temperature intakePivotTemperature;
    public AngularVelocity intakePivotVelocity;
    public double intakePivotPosition;
    public double intakePivotClosedLoopReference;
    public double intakePivotClosedLoopError;
  }

  public default void updateInputs(IntakePivotIOInputs inputs) {}

  public default void setPivotVoltage(Voltage volts) {}

  public default void setPivotVelocity(AngularVelocity velocity) {}

  public default void setPivotPosition(double position) {}

  public default void zeroPivotEncoder() {}
}
