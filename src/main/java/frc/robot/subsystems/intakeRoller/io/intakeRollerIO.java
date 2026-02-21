package frc.robot.subsystems.intakeRoller.io;

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
  }

  public default void updateInputs(intakeRollerIOInputs inputs) {}

  public default void setIntakeRollerVoltage(Voltage volts) {}

  public default void setRollerTorqueControl(AngularVelocity rollerVelo) {}
}
