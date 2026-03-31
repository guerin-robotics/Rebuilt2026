package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.flywheel.Flywheel;

/**
 * Command factory methods for controlling the shooter subsystem.
 *
 * <p>Provides various control modes:
 *
 * <ul>
 *   <li><b>Voltage control</b> - Direct voltage output (for manual testing)
 *   <li><b>Velocity control</b> - PID-controlled speed setpoints
 *   <li><b>Duty cycle control</b> - Percent output (for testing motor direction)
 * </ul>
 *
 * <p>Example usage in RobotContainer:
 *
 * <pre>
 * // Run shooter at 6V while button held, stop when released
 * button.whileTrue(ShooterCommands.runVoltage(shooter, Volts.of(6.0)));
 *
 * // Spin up to 3000 RPM
 * button.onTrue(ShooterCommands.runVelocity(shooter, RPM.of(3000)));
 * </pre>
 */
public class FlywheelCommands {

  /**
   * Runs the shooter at a specific voltage while the command is active. Stops when the command
   * ends.
   *
   * <p><b>Use case:</b> Manual testing, simple shooting without PID control.
   *
   * @param shooter The shooter subsystem
   * @param voltage The voltage to apply (e.g., Volts.of(6.0))
   * @return Command that runs shooter at voltage, stops on end
   */
  public static Command setFlywheelVoltage(Flywheel flywheel, Voltage voltage) {
    return Commands.startEnd(
            () -> flywheel.setFlywheelVoltage(voltage), // Apply voltage
            () -> flywheel.setFlywheelVoltage(Volts.of(0)), // Stop on end
            flywheel)
        .withName("FlywheelVoltage_" + voltage.in(Volts) + "V");
  }

  /**
   * Runs the shooter at a given velocity.
   *
   * @param flywheel
   * @param velocity
   * @return Command to run shooter at velocity, stops on end
   */
  public static Command setFlywheelVelocity(Flywheel flywheel, AngularVelocity velocity) {
    return Commands.runOnce(() -> flywheel.setFlywheelVelocity(velocity), flywheel);
  }

  /**
   * Runs the shooter at a low RPM
   *
   * @param flywheel
   * @return Command to run shooter at low velocity
   */
  public static Command flywheelIdle(Flywheel flywheel) {
    return Commands.run(() -> flywheel.setFlywheelIdle(), flywheel);
  }

  /**
   * Sets flywheel velocity based on position relative to specified passing target.
   *
   * @param flywheel
   * @return Command that sets flywheel velocity, stops on end
   */
  public static Command setPassVelocity(Flywheel flywheel) {
    return setVelocityForTarget(flywheel, flywheel.getPassTarget());
  }

  /**
   * Sets flywheel velocity based on position relative to hub (calculated from odometry).
   *
   * @param flywheel
   * @return Command that sets flywheel velocity using VelocityTorqueCurrent control, stops
   */
  public static Command setVelocityForHub(Flywheel flywheel) {
    return Commands.run(() -> flywheel.setSpeedForHub(), flywheel);
  }

  /**
   * Sets flywheel velocity based on position relative to target (calculated from odometry).
   *
   * @param flywheel
   * @param target
   * @return Command that sets flywheel velocity using VelocityTorqueCurrent control, stops
   */
  public static Command setVelocityForTarget(Flywheel flywheel, Translation3d target) {
    return Commands.startEnd(
        () -> flywheel.setSpeedForTarget(target),
        () -> flywheel.setFlywheelVoltage(Volts.of(0)),
        flywheel);
  }

  /**
   * Sets flywheel velocity based on distance given.
   *
   * @param flywheel
   * @param distance
   * @return Command that sets flywheel velocity using VelocityTorqueCurrent control, stops
   */
  public static Command setVelocityForDistance(Flywheel flywheel, Distance distance) {
    return Commands.startEnd(
        () -> flywheel.setSpeedForDistance(distance),
        () -> flywheel.setFlywheelVoltage(Volts.of(0)),
        flywheel);
  }

  /**
   * Stops the shooter immediately.
   *
   * <p><b>Use case:</b> Emergency stop, or ending shooter operation without waiting for command
   * interruption.
   *
   * @param shooter The shooter subsystem
   * @return Instant command that stops the shooter
   */
  public static Command stop(Flywheel shooter) {
    return Commands.runOnce(() -> shooter.setFlywheelVelocity(RotationsPerSecond.of(0)), shooter)
        .withName("FlywheelStop");
  }
}
