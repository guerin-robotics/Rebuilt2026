package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intakeRoller.intakeRoller;
import frc.robot.subsystems.prestage.Prestage;
import frc.robot.subsystems.transport.Transport;

public class SpitSequences {

  public static Command spitAll(
      Flywheel flywheel,
      Prestage prestage,
      Feeder feeder,
      Transport transport,
      intakeRoller intakeRoller) {
    return Commands.parallel(
            flywheel.setFlywheelVelocityCommand(
                HardwareConstants.CompConstants.SpitVelocities.flywheelSpitVelocity),
            prestage.setPrestageVelocityCommand(
                HardwareConstants.CompConstants.SpitVelocities.prestageSpitVelocity),
            feeder.setFeederVelocityCommand(
                HardwareConstants.CompConstants.SpitVelocities.feederSpitVelocity),
            transport.setTransportVoltageCommand(
                HardwareConstants.CompConstants.SpitVoltages.transportSpitVoltage),
            intakeRoller.setRollerVoltageCommand(
                HardwareConstants.CompConstants.SpitVoltages.intakeRollerSpitVoltage))
        .finallyDo(
            () -> {
              flywheel.setFlywheelVelocity(RotationsPerSecond.of(0));
              prestage.setPrestageVelocity(RotationsPerSecond.of(0));
              feeder.setFeederVelocity(RotationsPerSecond.of(0));
              transport.setTransportVoltage(Volts.of(0));
              intakeRoller.setRollerVoltage(Volts.of(0));
            });
  }

  public static Command spitHopper(Feeder feeder, Transport transport, intakeRoller intakeRoller) {
    return Commands.parallel(
            feeder.setFeederVelocityCommand(
                HardwareConstants.CompConstants.SpitVelocities.feederSpitVelocity),
            transport.setTransportVoltageCommand(
                HardwareConstants.CompConstants.SpitVoltages.transportSpitVoltage),
            intakeRoller.setRollerVoltageCommand(
                HardwareConstants.CompConstants.SpitVoltages.intakeRollerSpitVoltage))
        .finallyDo(
            () -> {
              feeder.setFeederVelocity(RotationsPerSecond.of(0));
              transport.setTransportVoltage(Volts.of(0));
              intakeRoller.setRollerVoltage(Volts.of(0));
            });
  }

  public static Command clearShooter(Flywheel flywheel, Prestage prestage, Feeder feeder) {
    return Commands.parallel(
            flywheel.setFlywheelVelocityCommand(
                HardwareConstants.CompConstants.SpitVelocities.flywheelSpitVelocity),
            prestage.setPrestageVelocityCommand(
                HardwareConstants.CompConstants.SpitVelocities.prestageSpitVelocity),
            feeder.setFeederVelocityCommand(
                HardwareConstants.CompConstants.SpitVelocities.feederSpitVelocity))
        .finallyDo(
            () -> {
              flywheel.setFlywheelVelocity(RotationsPerSecond.of(0));
              prestage.setPrestageVelocity(RotationsPerSecond.of(0));
              feeder.setFeederVelocity(RotationsPerSecond.of(0));
            });
  }

  public static Command spitAfterShoot(
      Flywheel flywheel,
      Prestage prestage,
      Feeder feeder,
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
                        feeder.setFeederVelocity(
                            HardwareConstants.CompConstants.SpitVelocities.feederSpitVelocity),
                    feeder),
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
            flywheel.setFlywheelVelocityCommand(RotationsPerSecond.of(0)),
            prestage.setPrestageVelocityCommand(RotationsPerSecond.of(0)),
            feeder.setFeederVelocityCommand(RotationsPerSecond.of(0)),
            transport.setTransportVoltageCommand(Volts.of(0)),
            intakeRoller.setRollerVoltageCommand(Volts.of(0))));
  }
}
