package frc.robot.subsystems.intakeSlider.io;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public interface intakeSliderIO {

    @AutoLog
    public static class IntakeSliderIOInputs {
        public Voltage intakeSliderVoltage;
        public Current intakeSliderSupplyCurrent;
        public Current intakeSliderStatorCurrent;
        public Temperature intakeSliderTemperature;
        public AngularVelocity intakeSliderVelocity;
    }

    public default void updateInputs(IntakeSliderIOInputs inputs) {}

    public default void setIntakeSliderVoltage(Voltage volts) {}
    
}
