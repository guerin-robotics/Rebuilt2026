package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;

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
public class ShooterCommands {

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
  public static Command runVoltage(Shooter shooter, Voltage voltage) {
    return Commands.startEnd(
            () -> shooter.runCharacterization(voltage), // Apply voltage
            () -> shooter.stopFlywheels(), // Stop on end
            shooter)
        .withName("ShooterVoltage_" + voltage.in(Volts) + "V");
  }

  /**
   * Runs the shooter at a specific angular velocity using closed-loop PID control. Stops when the
   * command ends.
   *
   * <p><b>Use case:</b> Precise shooting at a known speed (e.g., shooting from a fixed distance).
   *
   * @param shooter The shooter subsystem
   * @param speed The target angular velocity (e.g., RPM.of(3000) or RadiansPerSecond.of(314))
   * @return Command that runs shooter at target velocity, stops on end
   */
  public static Command runVelocity(Shooter shooter, AngularVelocity speed) {
    return Commands.startEnd(
            () -> shooter.setFlywheelSpeed(speed), // Set target velocity
            () -> shooter.stopFlywheels(), // Stop on end
            shooter)
        .withName(
            "ShooterVelocity_" + Math.round(speed.in(RevolutionsPerSecond) * 60.0) + "RPM");
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
  public static Command runDutyCycle(Shooter shooter, double dutyCycle) {
    return Commands.startEnd(
            () -> shooter.setFlywheelDutyCycle(dutyCycle), // Set duty cycle
            () -> shooter.stopFlywheels(), // Stop on end
            shooter)
        .withName("ShooterDutyCycle_" + Math.round(dutyCycle * 100) + "%");
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
  public static Command stop(Shooter shooter) {
    return Commands.runOnce(() -> shooter.stopFlywheels(), shooter).withName("ShooterStop");
  }

  /**
   * Runs the shooter at a target velocity and waits until it reaches the setpoint within
   * tolerance.
   *
   * <p><b>Use case:</b> Auto sequences where you need to wait for the shooter to spin up before
   * feeding.
   *
   * @param shooter The shooter subsystem
   * @param speed The target angular velocity
   * @return Command that spins up shooter and finishes when at speed
   */
  public static Command spinUpAndWait(Shooter shooter, AngularVelocity speed) {
    return Commands.runOnce(() -> shooter.setFlywheelSpeed(speed), shooter)
        .andThen(Commands.waitUntil(() -> shooter.areFlywheelsAtTargetSpeed()))
        .withName(
            "ShooterSpinUp_" + Math.round(speed.in(RevolutionsPerSecond) * 60.0) + "RPM");
  }

  /**
   * Runs the shooter at a target velocity for a specific duration, then stops.
   *
   * <p><b>Use case:</b> Testing, or timed shooting sequences in auto.
   *
   * @param shooter The shooter subsystem
   * @param speed The target angular velocity
   * @param seconds How long to run the shooter (in seconds)
   * @return Command that runs shooter for duration then stops
   */
  public static Command runForDuration(Shooter shooter, AngularVelocity speed, double seconds) {
    return runVelocity(shooter, speed)
        .withTimeout(seconds)
        .withName(
            "ShooterTimed_"
                + Math.round(speed.in(RevolutionsPerSecond) * 60.0)
                + "RPM_"
                + seconds
                + "s");
  }

  // Prevent instantiation - this is a utility class
  private ShooterCommands() {}
}
