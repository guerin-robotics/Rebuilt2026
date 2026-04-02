package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;

/**
 * Command factory methods for the Hood subsystem.
 *
 * <p><b>DEPRECATED:</b> Prefer calling commands directly on the subsystem instance (e.g., {@code
 * hood.setHoodPosCommand(...)}).
 */
public class HoodCommands {

  /**
   * @deprecated Use {@link Hood#setHoodPosCommand(double)} instead.
   */
  public static Command setHoodPos(Hood hood, double position) {
    return hood.setHoodPosCommand(position);
  }

  /**
   * @deprecated Use {@link Hood#setHoodPosForHubCommand()} instead.
   */
  public static Command setHoodPosForHub(Hood hood) {
    return hood.setHoodPosForHubCommand();
  }

  /**
   * @deprecated Use {@link Hood#incrementHoodPosCommand()} instead.
   */
  public static Command incrementHoodPos(Hood hood) {
    return hood.incrementHoodPosCommand();
  }
}
