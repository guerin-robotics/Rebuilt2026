package frc.robot.subsystems.feeder.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface FeederIO {

    @AutoLog
    public static class FeederIOInputs {
        public Voltage feederVoltage;
        public Current feederStatorAmps;
        public Current feederSupplyAmps;
        public AngularVelocity feederMotorVelocity;
        public Temperature feederMotorTemperature;
    }

    public default void updateInputs(FeederIOInputs inputs) {}
    
    public default void setFeederVoltage(Voltage volts) {}

    public default void setFeederSpeed(AngularVelocity speed) {}

}
