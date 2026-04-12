package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.intakeRoller.intakeRoller;
import frc.robot.subsystems.lowerFeeder.LowerFeeder;
import frc.robot.subsystems.prestage.Prestage;
import frc.robot.subsystems.upperFeeder.UpperFeeder;

public class SpitSequences {

  public static Command clearAfterShoot(
      Prestage prestage,
      UpperFeeder upperFeeder,
      LowerFeeder lowerFeeder,
      intakeRoller intakeRoller) {
    // Use deadline group: WaitCommand(0.5) is the deadline that controls duration.
    // The motor commands use Commands.run() so they continuously apply setpoints
    // and don't finish immediately (which would prematurely end a race group).
    return Commands.sequence(
        new WaitCommand(0.5)
            .deadlineFor(
                Commands.run(
                    () ->
                        prestage.setPrestageVelocity(
                            HardwareConstants.CompConstants.SpitVelocities.prestageSpitVelocity),
                    prestage),
                Commands.run(
                    () ->
                        upperFeeder.setUpperFeederVelocity(
                            HardwareConstants.CompConstants.SpitVelocities.feederSpitVelocity),
                    upperFeeder),
                Commands.run(
                    () ->
                        lowerFeeder.setLowerFeederVelocity(
                            HardwareConstants.CompConstants.SpitVelocities.feederSpitVelocity),
                    lowerFeeder)),
        Commands.parallel(

            PrestageCommands.setPrestageVelocity(prestage, RotationsPerSecond.of(0)),
            FeederCommands.setUpperFeederVelocity(upperFeeder, RotationsPerSecond.of(0)),
            FeederCommands.setLowerFeederVelocity(lowerFeeder, RotationsPerSecond.of(0))));
  }
}
