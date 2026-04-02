package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intakeRoller.intakeRoller;
import frc.robot.subsystems.lowerFeeder.LowerFeeder;
import frc.robot.subsystems.prestage.Prestage;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.upperFeeder.UpperFeeder;

public class SpitSequences {

  public static Command spitAll(
      Flywheel flywheel,
      Prestage prestage,
      UpperFeeder upperFeeder,
      LowerFeeder lowerFeeder,
      Transport transport,
      intakeRoller intakeRoller) {
    return Commands.parallel(
            FlywheelCommands.setFlywheelVelocity(
                flywheel, HardwareConstants.CompConstants.SpitVelocities.flywheelSpitVelocity),
            PrestageCommands.setPrestageVelocity(
                prestage, HardwareConstants.CompConstants.SpitVelocities.prestageSpitVelocity),
            FeederCommands.setUpperFeederVelocity(
                upperFeeder, HardwareConstants.CompConstants.SpitVelocities.feederSpitVelocity),
            FeederCommands.setLowerFeederVelocity(
                lowerFeeder, HardwareConstants.CompConstants.SpitVelocities.feederSpitVelocity),
            TransportCommands.setTransportVoltage(
                transport, HardwareConstants.CompConstants.SpitVoltages.transportSpitVoltage),
            intakeRollerCommands.setRollerVoltage(
                intakeRoller, HardwareConstants.CompConstants.SpitVoltages.intakeRollerSpitVoltage))
        .finallyDo(
            () -> {
              flywheel.setFlywheelVelocity(RotationsPerSecond.of(0));
              prestage.setPrestageVelocity(RotationsPerSecond.of(0));
              lowerFeeder.setLowerFeederVelocity(RotationsPerSecond.of(0));
              upperFeeder.setUpperFeederVelocity(RotationsPerSecond.of(0));
              transport.setTransportVoltage(Volts.of(0));
              intakeRoller.setRollerVoltage(Volts.of(0));
            });
  }

  public static Command spitHopper(
      UpperFeeder upperFeeder,
      LowerFeeder lowerFeeder,
      Transport transport,
      intakeRoller intakeRoller) {
    return Commands.parallel(
            FeederCommands.setUpperFeederVelocity(
                upperFeeder, HardwareConstants.CompConstants.SpitVelocities.feederSpitVelocity),
            FeederCommands.setLowerFeederVelocity(
                lowerFeeder, HardwareConstants.CompConstants.SpitVelocities.feederSpitVelocity),
            TransportCommands.setTransportVoltage(
                transport, HardwareConstants.CompConstants.SpitVoltages.transportSpitVoltage),
            intakeRollerCommands.setRollerVoltage(
                intakeRoller, HardwareConstants.CompConstants.SpitVoltages.intakeRollerSpitVoltage))
        .finallyDo(
            () -> {
              lowerFeeder.setLowerFeederVelocity(RotationsPerSecond.of(0));
              upperFeeder.setUpperFeederVelocity(RotationsPerSecond.of(0));
              transport.setTransportVoltage(Volts.of(0));
              intakeRoller.setRollerVoltage(Volts.of(0));
            });
  }

  public static Command clearShooter(
      Flywheel flywheel, Prestage prestage, UpperFeeder upperFeeder, LowerFeeder lowerFeeder) {
    return Commands.parallel(
            FlywheelCommands.setFlywheelVelocity(
                flywheel, HardwareConstants.CompConstants.SpitVelocities.flywheelSpitVelocity),
            PrestageCommands.setPrestageVelocity(
                prestage, HardwareConstants.CompConstants.SpitVelocities.prestageSpitVelocity),
            FeederCommands.setUpperFeederVelocity(
                upperFeeder, HardwareConstants.CompConstants.SpitVelocities.feederSpitVelocity),
            FeederCommands.setLowerFeederVelocity(
                lowerFeeder, HardwareConstants.CompConstants.SpitVelocities.feederSpitVelocity))
        .finallyDo(
            () -> {
              flywheel.setFlywheelVelocity(RotationsPerSecond.of(0));
              prestage.setPrestageVelocity(RotationsPerSecond.of(0));
              upperFeeder.setUpperFeederVelocity(RotationsPerSecond.of(0));
              lowerFeeder.setLowerFeederVelocity(RotationsPerSecond.of(0));
            });
  }

  public static Command spitAfterShoot(
      Flywheel flywheel,
      Prestage prestage,
      UpperFeeder upperFeeder,
      LowerFeeder lowerFeeder,
      Transport transport,
      intakeRoller intakeRoller) {
    // Use deadline group: WaitCommand(0.5) is the deadline that controls duration.
    // The motor commands use Commands.run() so they continuously apply setpoints
    // and don't finish immediately (which would prematurely end a race group).
    return Commands.sequence(
        new WaitCommand(0.5)
            .deadlineFor(
                Commands.run(
                    () ->
                        flywheel.setFlywheelVelocity(
                            HardwareConstants.CompConstants.SpitVelocities.flywheelSpitVelocity),
                    flywheel),
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
                    lowerFeeder),
                Commands.run(
                    () ->
                        transport.setTransportVoltage(
                            HardwareConstants.CompConstants.SpitVoltages.transportSpitVoltage),
                    transport),
                Commands.run(
                    () ->
                        intakeRoller.setRollerVoltage(
                            HardwareConstants.CompConstants.SpitVoltages.intakeRollerSpitVoltage),
                    intakeRoller)),
        Commands.parallel(
            FlywheelCommands.setFlywheelVelocity(flywheel, RotationsPerSecond.of(0)),
            PrestageCommands.setPrestageVelocity(prestage, RotationsPerSecond.of(0)),
            FeederCommands.setUpperFeederVelocity(upperFeeder, RotationsPerSecond.of(0)),
            FeederCommands.setLowerFeederVelocity(lowerFeeder, RotationsPerSecond.of(0)),
            TransportCommands.setTransportVoltage(transport, Volts.of(0)),
            intakeRollerCommands.setRollerVoltage(intakeRoller, Volts.of(0))));
  }
}
