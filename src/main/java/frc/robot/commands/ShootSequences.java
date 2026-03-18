package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intakeRoller.intakeRoller;
import frc.robot.subsystems.prestage.Prestage;
import frc.robot.subsystems.transport.Transport;
import org.littletonrobotics.junction.Logger;

public class ShootSequences {

  public static Command mapTuningShoot(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      Feeder feeder,
      Transport transport,
      intakeRoller intakeRoller) {
    final boolean zoneSafeToShoot = Flywheel.zoneSafeToShoot();
    Logger.recordOutput("Flywheel/zoneSafeToShoot", zoneSafeToShoot);
    if (zoneSafeToShoot) {
    return Commands.parallel(
            Commands.parallel(
                FlywheelCommands.setFlywheelVelocity(
                    flywheel, HardwareConstants.TuningConstants.FlywheelTuningVelocity),
                PrestageCommands.setPrestageVelocity(
                    prestage, HardwareConstants.TestVelocities.prestageVelocity),
                HoodCommands.setHoodPos(hood, HardwareConstants.TestPositions.hoodPos1Test)),
            Commands.sequence(
                new WaitCommand(0.5),
                FeederCommands.setFeederVelocity(
                    feeder, HardwareConstants.TestVelocities.feederVelocity),
                TransportCommands.setTransportVelocity(
                    transport, HardwareConstants.TestVelocities.transportVelocity),
                intakeRollerCommands.setRollerVoltage(
                    intakeRoller, HardwareConstants.TestVoltages.intakeRollerAgitateVoltage)))
        .finallyDo(
            () -> {
              flywheel.setFlywheelVelocity(RotationsPerSecond.of(0));
              prestage.setPrestageVelocity(RotationsPerSecond.of(0));
              hood.setHoodPos(HardwareConstants.TestPositions.hoodPos1Test);
              feeder.setFeederVelocity(RotationsPerSecond.of(0));
              transport.setTransportVelocity(RotationsPerSecond.of(0));
              intakeRoller.setRollerVoltage(Volts.of(0));
            });
        } else {
            return stopAll(flywheel, prestage, hood, feeder, transport, intakeRoller);
        }
  }

  public static Command shootForTower(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      Feeder feeder,
      Transport transport,
      intakeRoller intakeRoller) {
    final boolean zoneSafeToShoot = Flywheel.zoneSafeToShoot();
    Logger.recordOutput("Flywheel/zoneSafeToShoot", zoneSafeToShoot);
    if (zoneSafeToShoot) {
    return Commands.parallel(
            Commands.parallel(
                FlywheelCommands.setFlywheelVelocity(
                    flywheel, HardwareConstants.TowerConstants.FlywheelTowerVelocity),
                PrestageCommands.setPrestageVelocity(
                    prestage, HardwareConstants.TestVelocities.prestageVelocity),
                HoodCommands.setHoodPos(hood, HardwareConstants.TowerConstants.hoodTowerPos)),
            Commands.sequence(
                new WaitCommand(0.15),
                FeederCommands.setFeederVelocity(
                    feeder, HardwareConstants.TestVelocities.feederVelocity),
                TransportCommands.setTransportVelocity(
                    transport, HardwareConstants.TestVelocities.transportVelocity),
                intakeRollerCommands.setRollerVoltage(
                    intakeRoller, HardwareConstants.TestVoltages.intakeRollerAgitateVoltage)))
        .finallyDo(
            () -> {
              flywheel.setFlywheelVelocity(RotationsPerSecond.of(0));
              prestage.setPrestageVelocity(RotationsPerSecond.of(0));
              hood.setHoodPos(HardwareConstants.TestPositions.hoodPos1Test);
              feeder.setFeederVelocity(RotationsPerSecond.of(0));
              transport.setTransportVelocity(RotationsPerSecond.of(0));
              intakeRoller.setRollerVoltage(Volts.of(0));
            });
        } else {
            return stopAll(flywheel, prestage, hood, feeder, transport, intakeRoller);
        }
  }

  public static Command shootForTowerNoDelay(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      Feeder feeder,
      Transport transport,
      intakeRoller intakeRoller) {
    final boolean zoneSafeToShoot = Flywheel.zoneSafeToShoot();
    Logger.recordOutput("Flywheel/zoneSafeToShoot", zoneSafeToShoot);
    if (zoneSafeToShoot) {
    return Commands.parallel(
        FlywheelCommands.setFlywheelVelocity(
            flywheel, HardwareConstants.TowerConstants.FlywheelTowerVelocity),
        PrestageCommands.setPrestageVelocity(
            prestage, HardwareConstants.TestVelocities.prestageVelocity),
        HoodCommands.setHoodPos(hood, HardwareConstants.TowerConstants.hoodTowerPos),
        FeederCommands.setFeederVelocity(feeder, HardwareConstants.TestVelocities.feederVelocity),
        TransportCommands.setTransportVelocity(
            transport, HardwareConstants.TestVelocities.transportVelocity),
        intakeRollerCommands.setRollerVoltage(
            intakeRoller, HardwareConstants.TestVoltages.intakeRollerAgitateVoltage));
    } else {
        return stopAll(flywheel, prestage, hood, feeder, transport, intakeRoller);
    }
  }

  public static Command shootToHub(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      Feeder feeder,
      Transport transport,
      intakeRoller intakeRoller) {
    final boolean zoneSafeToShoot = Flywheel.zoneSafeToShoot();
    Logger.recordOutput("Flywheel/zoneSafeToShoot", zoneSafeToShoot);
    if(Flywheel.zoneSafeToShoot()) {
    return Commands.parallel(
            Commands.parallel(
                FlywheelCommands.setVelocityForHub(flywheel),
                PrestageCommands.setPrestageVelocity(
                    prestage, HardwareConstants.TestVelocities.prestageVelocity),
                HoodCommands.setHoodPosForHub(hood)),
            Commands.sequence(
                new WaitCommand(0.15),
                FeederCommands.setFeederVelocity(
                    feeder, HardwareConstants.TestVelocities.feederVelocity),
                TransportCommands.setTransportVelocity(
                    transport, HardwareConstants.TestVelocities.transportVelocity),
                intakeRollerCommands.setRollerVoltage(
                    intakeRoller, HardwareConstants.TestVoltages.intakeRollerAgitateVoltage)))
        .finallyDo(
            () -> {
              flywheel.setFlywheelVelocity(RotationsPerSecond.of(0));
              prestage.setPrestageVelocity(RotationsPerSecond.of(0));
              hood.setHoodPos(HardwareConstants.TestPositions.hoodPos1Test);
              feeder.setFeederVelocity(RotationsPerSecond.of(0));
              transport.setTransportVelocity(RotationsPerSecond.of(0));
              intakeRoller.setRollerVoltage(Volts.of(0));
            })
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        } else {
            return stopAll(flywheel, prestage, hood, feeder, transport, intakeRoller);
        }
  }

  public static Command pass(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      Feeder feeder,
      Transport transport,
      intakeRoller intakeRoller) {
    return Commands.parallel(
            Commands.parallel(
                FlywheelCommands.setFlywheelVelocity(
                    flywheel, HardwareConstants.PassConstants.FlywheelPassVelocity),
                PrestageCommands.setPrestageVelocity(
                    prestage, HardwareConstants.TestVelocities.prestageVelocity),
                HoodCommands.setHoodPos(hood, HardwareConstants.PassConstants.hoodPassPos)),
            Commands.sequence(
                new WaitCommand(0.5),
                FeederCommands.setFeederVelocity(
                    feeder, HardwareConstants.TestVelocities.feederVelocity),
                TransportCommands.setTransportVelocity(
                    transport, HardwareConstants.TestVelocities.transportVelocity),
                intakeRollerCommands.setRollerVoltage(
                    intakeRoller, HardwareConstants.TestVoltages.intakeRollerAgitateVoltage)))
        .finallyDo(
            () -> {
              flywheel.setFlywheelVelocity(RotationsPerSecond.of(0));
              prestage.setPrestageVelocity(RotationsPerSecond.of(0));
              hood.setHoodPos(HardwareConstants.TestPositions.hoodPos1Test);
              feeder.setFeederVelocity(RotationsPerSecond.of(0));
              transport.setTransportVelocity(RotationsPerSecond.of(0));
              intakeRoller.setRollerVoltage(Volts.of(0));
            });
  }

  public static Command zonePassOrShoot(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      Feeder feeder,
      Transport transport,
      intakeRoller intakeRoller) {
    final boolean zoneSafeToShoot = Flywheel.zoneSafeToShoot();
    Logger.recordOutput("Flywheel/zoneSafeToShoot", zoneSafeToShoot);
    if (Flywheel.zoneSafeToShoot()) {
      return shootToHub(flywheel, prestage, hood, feeder, transport, intakeRoller);
    } else {
      return pass(flywheel, prestage, hood, feeder, transport, intakeRoller);
    }
  }

  public static Command FirstSet(Flywheel flywheel, Prestage prestage, Hood hood) {
    return Commands.parallel(
            FlywheelCommands.setFlywheelVelocity(
                flywheel, HardwareConstants.TowerConstants.FlywheelTowerVelocity),
            PrestageCommands.setPrestageVelocity(
                prestage, HardwareConstants.TestVelocities.prestageVelocity),
            HoodCommands.setHoodPos(hood, HardwareConstants.TowerConstants.hoodTowerPos))
        .finallyDo(
            () -> {
              flywheel.setFlywheelVelocity(RotationsPerSecond.of(0));
              prestage.setPrestageVelocity(RotationsPerSecond.of(0));
              hood.setHoodPos(HardwareConstants.TestPositions.hoodPos1Test);
            });
  }

  public static Command SecondSet(Feeder feeder, Transport transport) {
    return Commands.parallel(
            FeederCommands.setFeederVelocity(
                feeder, HardwareConstants.TestVelocities.feederVelocity),
            TransportCommands.setTransportVelocity(
                transport, HardwareConstants.TestVelocities.transportVelocity))
        .finallyDo(
            () -> {
              feeder.setFeederVelocity(RotationsPerSecond.of(0));
              transport.setTransportVelocity(RotationsPerSecond.of(0));
            });
  }

  public static Command stopAll(
          Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      Feeder feeder,
      Transport transport,
      intakeRoller intakeRoller
  ) {
    return Commands.parallel(
        FlywheelCommands.setFlywheelVelocity(flywheel, RotationsPerSecond.of(0)),
        PrestageCommands.setPrestageVelocity(prestage, RotationsPerSecond.of(0)),
        HoodCommands.setHoodPos(hood, HardwareConstants.TestPositions.hoodPos1Test),
        FeederCommands.setFeederVelocity(feeder, RotationsPerSecond.of(0)),
        TransportCommands.setTransportVelocity(transport, RotationsPerSecond.of(0)),
        intakeRollerCommands.setRollerVelocity(intakeRoller, RotationsPerSecond.of(0))
    );
  }
}
