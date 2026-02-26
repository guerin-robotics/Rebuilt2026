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
   * Runs the shooter at a specific duty cycle (percent output). Stops when the command ends.
   *
   * <p><b>Use case:</b> Testing motor direction, wiring, or basic functionality without sensors.
   *
   * <p><b>Warning:</b> This is open-loop control with no velocity feedback. Use {@link
   * #runVelocity} for actual shooting.
   *
   * @param shooter The shooter subsystem
   * @param dutyCycle The motor output (-1.0 to 1.0, where 1.0 = 100% forward)
   * @return Command that runs shooter at duty cycle, stops on end
   */
  public static Command runDutyCycle(Flywheel flywheel, double dutyCycle) {
    return Commands.startEnd(
            () -> flywheel.setFlywheelDutyCycle(dutyCycle), // Set duty cycle
            () -> flywheel.setFlywheelVoltage(Volts.of(0)), // Stop on end
            flywheel)
        .withName("FlywheelDutyCycle_" + Math.round(dutyCycle * 100) + "%");
  }

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
  public static Command runVoltage(Flywheel flywheel, Voltage voltage) {
    return Commands.startEnd(
            () -> flywheel.setFlywheelVoltage(voltage), // Apply voltage
            () -> flywheel.setFlywheelVoltage(Volts.of(0)), // Stop on end
            flywheel)
        .withName("FlywheelVoltage_" + voltage.in(Volts) + "V");
  }

  /**
   * Runs the shooter at a target speed using feedforward-only control. Stops when the command ends.
   *
   * <p><b>Use case:</b> Shooting at a known RPM. Uses kS and kV from constants (no PID).
   *
   * @param shooter The shooter subsystem
   * @param targetSpeed Desired angular velocity (e.g., RPM.of(3000))
   * @return Command that runs shooter at target speed via feedforward, stops on end
   */
  public static Command runVelocity(Flywheel flywheel, AngularVelocity targetSpeed) {
    return Commands.startEnd(
            () -> flywheel.setFlywheelSpeed(targetSpeed),
            () -> flywheel.setFlywheelVoltage(Volts.of(0)),
            flywheel)
        .withName("FlywheelVelocity_" + targetSpeed.in(RPM) + "RPM");
  }

  public static Command runTorque(Flywheel flywheel, AngularVelocity velocity) {
    return Commands.startEnd(
        () -> flywheel.setFlywheelTorque(velocity),
        () -> flywheel.setFlywheelTorque(RotationsPerSecond.of(0)),
        flywheel);
  }

  /**
   * Sets flywheel velocity based on position relative to hub (calculated from odometry).
   *
   * @param flywheel
   * @return Command that sets flywheel velocity using VelocityTorqueCurrent control, stops
   */
  public static Command setVelocityForHub(Flywheel flywheel) {
    return Commands.startEnd(
        () -> flywheel.setSpeedForHub(), () -> flywheel.setFlywheelVoltage(Volts.of(0)), flywheel);
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
    return Commands.runOnce(() -> shooter.setFlywheelVoltage(Volts.of(0)), shooter)
        .withName("FlywheelStop");
  }
}
