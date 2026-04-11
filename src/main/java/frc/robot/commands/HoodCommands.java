package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

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
    return Commands.startEnd(() -> hood.setHoodPos(position), () -> hood.stowHood(), hood);
  }

  /**
   * Continuously tracks the calculated hub position. When the command ends, the hood motor is
   * stopped.
   */
  public static Command setHoodPosForHub(Hood hood) {
    return Commands.runEnd(() -> hood.setHoodPosForHub(), () -> hood.stowHood(), hood);
  }

  /** Stops the hood motor. */
  public static Command stowHood(Hood hood) {
    return Commands.runOnce(() -> hood.stowHood(), hood);
  }

  /**
   * Idle command for the hood — continuously sends the hood to the stow position (0°). The tuning
   * offset is applied inside {@code Hood.setHoodPos}, so in tuning mode the "stow" position shifts
   * by whatever offset the operator has dialed in. Use this as the default command so the hood
   * always has a target when no other command is active.
   */
  public static Command hoodIdle(Hood hood) {
    return Commands.run(() -> hood.setHoodPos(Degrees.of(0)), hood).withName("HoodIdle");
  }

  /**
   * Increments the persistent tuning offset by +5°. The offset is added to every hood position
   * command, so the next time the hood moves it will be 5° higher than before. Only useful in
   * tuning mode.
   */
  public static Command incrementHoodOffset(Hood hood) {
    return Commands.runOnce(() -> hood.incrementHoodOffset(), hood);
  }

  /** Resets the persistent tuning offset back to 0°. */
  public static Command resetHoodOffset(Hood hood) {
    return Commands.runOnce(() -> hood.resetHoodOffset(), hood);
  }
}
