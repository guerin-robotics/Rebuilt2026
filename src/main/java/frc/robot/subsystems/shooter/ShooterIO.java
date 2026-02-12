package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        // Define sensor inputs here
        // Example: public double positionRad = 0.0;
        // Example: public double velocityRadPerSec = 0.0;
        // Example: public double appliedVolts = 0.0;
        // Example: public double currentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ShooterIOInputs inputs) {}

    /** Run motors at voltage */
    public default void setVoltage(double volts) {}

    // Add other control methods as needed
    // Example: public default void setPosition(double positionRad) {}
    // Example: public default void stop() {}
}