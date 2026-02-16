package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

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
public class FlwheelCommands {

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
  public static Command runVoltage(Flywheel shooter, Voltage voltage) {
    return Commands.startEnd(
            () -> shooter.setFlywheelVoltage(voltage), // Apply voltage
            () -> shooter.stopFlywheels(), // Stop on end
            shooter)
        .withName("FlwheelVoltage_" + voltage.in(Volts) + "V");
  }

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
  public static Command runDutyCycle(Flywheel shooter, double dutyCycle) {
    return Commands.startEnd(
            () -> shooter.setFlywheelDutyCycle(dutyCycle), // Set duty cycle
            () -> shooter.stopFlywheels(), // Stop on end
            shooter)
        .withName("FlwheelDutyCycle_" + Math.round(dutyCycle * 100) + "%");
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
    return Commands.runOnce(() -> shooter.stopFlywheels(), shooter).withName("FlwheelStop");
  }

  // Prevent instantiation - this is a utility class
  private FlwheelCommands() {}
}
