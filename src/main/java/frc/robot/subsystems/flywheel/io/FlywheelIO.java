package frc.robot.subsystems.flywheel.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
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
    public AngularVelocity flywheelVelocity = RotationsPerSecond.of(0);

    public AngularVelocity closedLoopError = RotationsPerSecond.of(0);
    public AngularVelocity closedLoopReference = RotationsPerSecond.of(0);

    // Leader motor
    public AngularVelocity leaderVelocity = RotationsPerSecond.of(0);
    public Voltage leaderAppliedVolts = Volts.of(0);
    public Current leaderSupplyCurrentAmps = Amps.of(0);
    public Current leaderStatorCurrentAmps = Amps.of(0);
    public Temperature leaderTemp = Celsius.of(0);

    public Angle leaderAngle = Rotations.of(0);

    // Follower 1 motor
    public AngularVelocity follower1Velocity = RotationsPerSecond.of(0);
    public Voltage follower1AppliedVolts = Volts.of(0);
    public Current follower1SupplyCurrentAmps = Amps.of(0);
    public Current follower1StatorCurrentAmps = Amps.of(0);
    public Temperature follower1Temp = Celsius.of(0);

    // Follower 2 motor
    public AngularVelocity follower2Velocity = RotationsPerSecond.of(0);
    public Voltage follower2AppliedVolts = Volts.of(0);
    public Current follower2SupplyCurrentAmps = Amps.of(0);
    public Current follower2StatorCurrentAmps = Amps.of(0);
    public Temperature follower2Temp = Celsius.of(0);

    // Follower 3 motor
    public AngularVelocity follower3Velocity = RotationsPerSecond.of(0);
    public Voltage follower3AppliedVolts = Volts.of(0);
    public Current follower3SupplyCurrentAmps = Amps.of(0);
    public Current follower3StatorCurrentAmps = Amps.of(0);
    public Temperature follower3Temp = Celsius.of(0);

    // Follower 4 motor
    public AngularVelocity follower4Velocity = RotationsPerSecond.of(0);
    public Voltage follower4AppliedVolts = Volts.of(0);
    public Current follower4SupplyCurrentAmps = Amps.of(0);
    public Current follower4StatorCurrentAmps = Amps.of(0);
    public Temperature follower4Temp = Celsius.of(0);
  }

  /** Read sensor data from the flywheel motor. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Set the flywheel to a specific voltage output. For SysId characterization. */
  public default void setFlywheelVoltage(Voltage volts) {}

  // VelocityTorqueCurrentFOC control
  public default void setFlywheelVelocity(AngularVelocity velocity) {}
}
