package frc.robot.subsystems.flywheel.io;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
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

    // Leader motor
    /** Leader motor velocity. */
    public AngularVelocity leaderVelocity;
    /** Voltage applied to the leader motor. */
    public double leaderAppliedVolts;
    /** Supply current drawn by the leader motor (amps). */
    public double leaderSupplyCurrentAmps;
    /** Stator current of the leader motor (amps). */
    public double leaderStatorCurrentAmps;

    // Follower 1 motor
    /** Follower 1 motor velocity. */
    public AngularVelocity follower1Velocity;
    /** Voltage applied to follower 1 motor. */
    public double follower1AppliedVolts;
    /** Supply current drawn by follower 1 motor (amps). */
    public double follower1SupplyCurrentAmps;
    /** Stator current of follower 1 motor (amps). */
    public double follower1StatorCurrentAmps;

    // Follower 2 motor
    /** Follower 2 motor velocity. */
    public AngularVelocity follower2Velocity;
    /** Voltage applied to follower 2 motor. */
    public double follower2AppliedVolts;
    /** Supply current drawn by follower 2 motor (amps). */
    public double follower2SupplyCurrentAmps;
    /** Stator current of follower 2 motor (amps). */
    public double follower2StatorCurrentAmps;

    // Follower 3 motor
    /** Follower 3 motor velocity. */
    public AngularVelocity follower3Velocity;
    /** Voltage applied to follower 3 motor. */
    public double follower3AppliedVolts;
    /** Supply current drawn by follower 3 motor (amps). */
    public double follower3SupplyCurrentAmps;
    /** Stator current of follower 3 motor (amps). */
    public double follower3StatorCurrentAmps;
  }

  /** Read sensor data from the flywheel motor. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Set the flywheel to a target velocity using closed-loop control. */
  public default void setFlywheelSpeed(AngularVelocity speed) {}

  /** Set the flywheel to a raw duty cycle output (0.0 to 1.0). For testing only. */
  public default void setFlywheelDutyCycle(double output) {}

  /** Stop the flywheel motor. */
  public default void stopFlywheel() {}

  /** Set the flywheel to a specific voltage output. For SysId characterization. */
  public default void setFlywheelVoltage(Voltage volts) {}
}
