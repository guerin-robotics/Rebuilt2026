package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hood.Hood;

public class HoodCommands {

  /**
   * Sets the hood to a specific position. When the command ends (e.g., button released), the hood
   * motor is stopped so it doesn't keep holding a closed-loop reference.
   */
  public static Command setHoodPos(Hood hood, Angle position) {
    return Commands.startEnd(() -> hood.setHoodPos(position), () -> hood.stopHood(), hood);
  }

  /**
   * Continuously tracks the calculated hub position. When the command ends, the hood motor is
   * stopped.
   */
  public static Command setHoodPosForHub(Hood hood) {
    return Commands.runEnd(() -> hood.setHoodPosForHub(), () -> hood.stopHood(), hood);
  }

  /** Stops the hood motor. */
  public static Command stopHood(Hood hood) {
    return Commands.runOnce(() -> hood.stopHood(), hood);
  }

  public static Command incrementHoodPos(Hood hood) {
    return Commands.runOnce(() -> hood.incrementHoodPos(), hood);
  }
}
