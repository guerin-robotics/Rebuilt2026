package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hood.Hood;

public class HoodCommands {

  public static Command setHoodPos(Hood hood, double position) {
    return Commands.runOnce(() -> hood.setHoodPos(position), hood);
  }
}
