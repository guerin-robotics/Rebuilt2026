package frc.robot.subsystems.flywheel.io;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Temperature;
import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the shooter flywheel.
 *
 * <p>Defines the hardware abstraction contract for the shooter. Implementations include {@link
 * FlywheelIOPhoenix6} for real CTRE TalonFX hardware.
 *
 * <p>This interface uses WPILib units (AngularVelocity, Voltage) for type safety.
 */
public interface FlywheelIO {

  /** Sensor data read from the flywheel motors each loop. */
  @AutoLog
  public static class ShooterIOInputs {
    // Combined flywheel velocity (typically matches leader velocity)
    /** Combined flywheel velocity (average or leader). */
    public AngularVelocity flywheelVelocity;

    public AngularVelocity closedLoopError;
    public AngularVelocity closedLoopReference;

    // Leader motor
    public AngularVelocity leaderVelocity;
    public Voltage leaderAppliedVolts;
    public Current leaderSupplyCurrentAmps;
    public Current leaderStatorCurrentAmps;
    public Temperature leaderTemp;

    // Follower 1 motor
    public AngularVelocity follower1Velocity;
    public Voltage follower1AppliedVolts;
    public Current follower1SupplyCurrentAmps;
    public Current follower1StatorCurrentAmps;
    public Temperature follower1Temp;

    // Follower 2 motor
    public AngularVelocity follower2Velocity;
    public Voltage follower2AppliedVolts;
    public Current follower2SupplyCurrentAmps;
    public Current follower2StatorCurrentAmps;
    public Temperature follower2Temp;

    // Follower 3 motor
    public AngularVelocity follower3Velocity;
    public Voltage follower3AppliedVolts;
    public Current follower3SupplyCurrentAmps;
    public Current follower3StatorCurrentAmps;
    public Temperature follower3Temp;
  }

  /** Read sensor data from the flywheel motor. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Set the flywheel to a raw duty cycle output (0.0 to 1.0). For testing only. */
  public default void setFlywheelDutyCycle(double output) {}

  /** Set the flywheel to a specific voltage output. For SysId characterization. */
  public default void setFlywheelVoltage(Voltage volts) {}

  // Set flywheel speed
  public default void setFlywheelSpeed(AngularVelocity targetSpeed) {}

  // VelocityTorqueCurrentFOC control
  public default void setFlywheelTorque(AngularVelocity velocity) {}
}
