package frc.robot.subsystems.intakeRoller.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface intakeRollerIO {

  @AutoLog
  public static class intakeRollerIOInputs {
    public Voltage intakeRollerVoltage = Volts.of(0);
    public Current intakeRollerSupplyCurrent = Amps.of(0);
    public Current intakeRollerStatorCurrent = Amps.of(0);
    /** Degrees Celsius. */
    public double intakeRollerTemperature = 0.0;

    /** Revolutions per minute. */
    public double intakeRollerVelocity = 0.0;
    /** Revolutions per minute. */
    public double rollerClosedLoopReference = 0.0;
    /** Revolutions per minute. */
    public double rollerClosedLoopError = 0.0;
    /** Degrees. */
    public double rollerPos = 0.0;

    public Voltage intakeRollerFollowerVoltage = Volts.of(0);
    public Current intakeRollerFollowerSupplyCurrent = Amps.of(0);
    public Current intakeRollerFollowerStatorCurrent = Amps.of(0);
    /** Degrees Celsius. */
    public double intakeRollerFollowerTemperature = 0.0;

    /** Revolutions per minute. */
    public double intakeRollerFollowerVelocity = 0.0;
    /** Degrees. */
    public double rollerFollowerPos = 0.0;
  }

  public default void updateInputs(intakeRollerIOInputs inputs) {}

  public default void setRollerVoltage(Voltage volts) {}

  public default void setRollerVelocity(AngularVelocity rollerVelo) {}
}
