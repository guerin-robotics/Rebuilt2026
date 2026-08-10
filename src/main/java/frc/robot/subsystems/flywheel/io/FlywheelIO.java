package frc.robot.subsystems.flywheel.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
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
    /** Revolutions per minute. */
    public double flywheelVelocity = 0.0;

    /** Revolutions per minute. */
    public double closedLoopError = 0.0;
    /** Revolutions per minute. */
    public double closedLoopReference = 0.0;

    // Leader motor
    /** Revolutions per minute. */
    public double leaderVelocity = 0.0;

    public Voltage leaderAppliedVolts = Volts.of(0);
    public Current leaderSupplyCurrentAmps = Amps.of(0);
    public Current leaderStatorCurrentAmps = Amps.of(0);
    /** Degrees Celsius. */
    public double leaderTemp = 0.0;

    /** Degrees, accumulated. */
    public double leaderAngle = 0.0;

    // Follower 1 motor
    /** Revolutions per minute. */
    public double follower1Velocity = 0.0;

    public Voltage follower1AppliedVolts = Volts.of(0);
    public Current follower1SupplyCurrentAmps = Amps.of(0);
    public Current follower1StatorCurrentAmps = Amps.of(0);
    /** Degrees Celsius. */
    public double follower1Temp = 0.0;

    // Follower 2 motor
    /** Revolutions per minute. */
    public double follower2Velocity = 0.0;

    public Voltage follower2AppliedVolts = Volts.of(0);
    public Current follower2SupplyCurrentAmps = Amps.of(0);
    public Current follower2StatorCurrentAmps = Amps.of(0);
    /** Degrees Celsius. */
    public double follower2Temp = 0.0;

    // Follower 3 motor
    /** Revolutions per minute. */
    public double follower3Velocity = 0.0;

    public Voltage follower3AppliedVolts = Volts.of(0);
    public Current follower3SupplyCurrentAmps = Amps.of(0);
    public Current follower3StatorCurrentAmps = Amps.of(0);
    /** Degrees Celsius. */
    public double follower3Temp = 0.0;

    // Follower 4 motor
    /** Revolutions per minute. */
    public double follower4Velocity = 0.0;

    public Voltage follower4AppliedVolts = Volts.of(0);
    public Current follower4SupplyCurrentAmps = Amps.of(0);
    public Current follower4StatorCurrentAmps = Amps.of(0);
    /** Degrees Celsius. */
    public double follower4Temp = 0.0;
  }

  /** Read sensor data from the flywheel motor. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Set the flywheel to a specific voltage output. For SysId characterization. */
  public default void setFlywheelVoltage(Voltage volts) {}

  // VelocityTorqueCurrentFOC control
  public default void setFlywheelVelocity(AngularVelocity velocity) {}
}
