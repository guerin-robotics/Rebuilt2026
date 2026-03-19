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
            FlywheelCommands.setFlywheelVelocity(
                flywheel, HardwareConstants.SpitVelocities.FlywheelSpitVelocity),
            PrestageCommands.setPrestageVelocity(
                prestage, HardwareConstants.SpitVelocities.prestageSpitVelocity),
            FeederCommands.setFeederVelocity(
                feeder, HardwareConstants.SpitVelocities.feederSpitVelocity),
            TransportCommands.setTransportVoltage(
                transport, HardwareConstants.SpitVoltages.transportSpitVolts),
            intakeRollerCommands.setRollerVelocity(
                intakeRoller, HardwareConstants.SpitVelocities.rollerSpitVelocity))
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
            FeederCommands.setFeederVelocity(
                feeder, HardwareConstants.SpitVelocities.feederSpitVelocity),
            TransportCommands.setTransportVoltage(
                transport, HardwareConstants.SpitVoltages.transportSpitVolts),
            intakeRollerCommands.setRollerVelocity(
                intakeRoller, HardwareConstants.SpitVelocities.rollerSpitVelocity))
        .finallyDo(
            () -> {
              feeder.setFeederVelocity(RotationsPerSecond.of(0));
              transport.setTransportVoltage(Volts.of(0));
              intakeRoller.setRollerVoltage(Volts.of(0));
            });
  }

  public static Command clearShooter(Flywheel flywheel, Prestage prestage, Feeder feeder) {
    return Commands.parallel(
            FlywheelCommands.setFlywheelVelocity(
                flywheel, HardwareConstants.SpitVelocities.FlywheelSpitVelocity),
            PrestageCommands.setPrestageVelocity(
                prestage, HardwareConstants.SpitVelocities.prestageSpitVelocity),
            FeederCommands.setFeederVelocity(
                feeder, HardwareConstants.SpitVelocities.feederSpitVelocity))
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
                            HardwareConstants.SpitVelocities.FlywheelSpitVelocity),
                    flywheel),
                Commands.run(
                    () ->
                        prestage.setPrestageVelocity(
                            HardwareConstants.SpitVelocities.prestageSpitVelocity),
                    prestage),
                Commands.run(
                    () ->
                        feeder.setFeederVelocity(
                            HardwareConstants.SpitVelocities.feederSpitVelocity),
                    feeder),
                Commands.run(
                    () ->
                        transport.setTransportVoltage(
                            HardwareConstants.SpitVoltages.transportSpitVolts),
                    transport),
                Commands.run(
                    () ->
                        intakeRoller.setRollerVoltage(
                            HardwareConstants.SpitVoltages.rollerSpitVolts),
                    intakeRoller)),
        Commands.parallel(
            FlywheelCommands.setFlywheelVelocity(flywheel, RotationsPerSecond.of(0)),
            PrestageCommands.setPrestageVelocity(prestage, RotationsPerSecond.of(0)),
            FeederCommands.setFeederVelocity(feeder, RotationsPerSecond.of(0)),
            TransportCommands.setTransportVoltage(transport, Volts.of(0)),
            intakeRollerCommands.setRollerVoltage(intakeRoller, Volts.of(0))));
  }
}
