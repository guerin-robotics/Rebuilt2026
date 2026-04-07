package frc.robot.subsystems.intakeRoller.io;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface intakeRollerIO {

  @AutoLog
  public static class intakeRollerIOInputs {
    public Voltage intakeRollerVoltage;
    public Current intakeRollerSupplyCurrent;
    public Current intakeRollerStatorCurrent;
    public Temperature intakeRollerTemperature;
    public AngularVelocity intakeRollerVelocity;
    public AngularVelocity rollerClosedLoopReference;
    public AngularVelocity rollerClosedLoopError;
    public Angle rollerPos;
    public Voltage intakeRollerFollowerVoltage;
    public Current intakeRollerFollowerSupplyCurrent;
    public Current intakeRollerFollowerStatorCurrent;
    public Temperature intakeRollerFollowerTemperature;
    public AngularVelocity intakeRollerFollowerVelocity;
    public AngularVelocity rollerFollowerClosedLoopReference;
    public AngularVelocity rollerFollowerClosedLoopError;
    public Angle rollerFollowerPos;
  }

  public default void updateInputs(intakeRollerIOInputs inputs) {}

  public default void setRollerVoltage(Voltage volts) {}

  public default void setRollerVelocity(AngularVelocity rollerVelo) {}
}
