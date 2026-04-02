package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;

/**
 * Command factory methods for controlling the shooter subsystem.
 *
 * <p><b>DEPRECATED:</b> These static methods delegate to the command factory methods on the {@link
 * Flywheel} subsystem. Prefer calling commands directly on the subsystem instance (e.g., {@code
 * flywheel.setFlywheelVoltageCommand(...)}).
 *
 * <p>Provides various control modes:
 *
 * <ul>
 *   <li><b>Voltage control</b> - Direct voltage output (for manual testing)
 *   <li><b>Velocity control</b> - PID-controlled speed setpoints
 *   <li><b>Duty cycle control</b> - Percent output (for testing motor direction)
 * </ul>
 */
public class FlywheelCommands {

  /**
   * @deprecated Use {@link Flywheel#setFlywheelVoltageCommand(Voltage)} instead.
   */
  public static Command setFlywheelVoltage(Flywheel flywheel, Voltage voltage) {
    return flywheel.setFlywheelVoltageCommand(voltage);
  }

  /**
   * @deprecated Use {@link Flywheel#setFlywheelVelocityCommand(AngularVelocity)} instead.
   */
  public static Command setFlywheelVelocity(Flywheel flywheel, AngularVelocity velocity) {
    return flywheel.setFlywheelVelocityCommand(velocity);
  }

  /**
   * @deprecated Use {@link Flywheel#flywheelIdleCommand()} instead.
   */
  public static Command flywheelIdle(Flywheel flywheel) {
    return flywheel.flywheelIdleCommand();
  }

  /**
   * @deprecated Use {@link Flywheel#setPassVelocityCommand()} instead.
   */
  public static Command setPassVelocity(Flywheel flywheel) {
    return flywheel.setPassVelocityCommand();
  }

  /**
   * @deprecated Use {@link Flywheel#setVelocityForHubCommand()} instead.
   */
  public static Command setVelocityForHub(Flywheel flywheel) {
    return flywheel.setVelocityForHubCommand();
  }

  /**
   * @deprecated Use {@link Flywheel#setVelocityForTargetCommand(Translation3d)} instead.
   */
  public static Command setVelocityForTarget(Flywheel flywheel, Translation3d target) {
    return flywheel.setVelocityForTargetCommand(target);
  }

  /**
   * @deprecated Use {@link Flywheel#setVelocityForDistanceCommand(Distance)} instead.
   */
  public static Command setVelocityForDistance(Flywheel flywheel, Distance distance) {
    return flywheel.setVelocityForDistanceCommand(distance);
  }

  /**
   * @deprecated Use {@link Flywheel#stopCommand()} instead.
   */
  public static Command stop(Flywheel shooter) {
    return shooter.stopCommand();
  }
}
