package frc.robot.subsystems.intakeRoller.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

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
    
}
