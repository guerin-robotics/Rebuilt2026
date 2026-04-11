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

  /**
   * Idle command for the hood — continuously sends zero voltage to the motor. Use this as the
   * default command so the hood motor doesn't hold a stale closed-loop reference when no other
   * command is active.
   */
  public static Command hoodIdle(Hood hood) {
    return Commands.run(() -> hood.setHoodPos(Degrees.of(1)), hood).withName("HoodIdle");
  }

  public static Command incrementHoodPos(Hood hood) {
    return Commands.runOnce(() -> hood.incrementHoodPos(), hood);
  }
}
