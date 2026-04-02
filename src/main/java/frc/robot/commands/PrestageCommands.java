package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.prestage.Prestage;

/**
 * Command factory methods for the Prestage subsystem.
 *
 * <p><b>DEPRECATED:</b> Prefer calling commands directly on the subsystem instance (e.g., {@code
 * prestage.setPrestageVelocityCommand(...)}).
 */
public class PrestageCommands {

  /**
   * @deprecated Use {@link Prestage#setPrestageVoltageCommand(Voltage)} instead.
   */
  public static Command setPrestageVoltage(Prestage prestage, Voltage voltage) {
    return prestage.setPrestageVoltageCommand(voltage);
  }

  /**
   * @deprecated Use {@link Prestage#stopCommand()} instead.
   */
  public static Command stop(Prestage prestage) {
    return prestage.stopCommand();
  }

  /**
   * @deprecated Use {@link Prestage#setPrestageVelocityCommand(AngularVelocity)} instead.
   */
  public static Command setPrestageVelocity(Prestage prestage, AngularVelocity prestageVelo) {
    return prestage.setPrestageVelocityCommand(prestageVelo);
  }

  /**
   * @deprecated Use {@link Prestage#setOneVeloCommand(AngularVelocity)} instead.
   */
  public static Command setOneVelo(Prestage prestage, AngularVelocity velo) {
    return prestage.setOneVeloCommand(velo);
  }

  /**
   * @deprecated Use {@link Prestage#prestageIdleCommand()} instead.
   */
  public static Command prestageIdle(Prestage prestage) {
    return prestage.prestageIdleCommand();
  }
}
