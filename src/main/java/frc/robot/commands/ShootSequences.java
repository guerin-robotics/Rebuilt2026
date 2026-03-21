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
import frc.robot.subsystems.intakePivot.IntakePivot;
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
    return Commands.parallel(
        Commands.parallel(
            FlywheelCommands.setFlywheelVelocity(
                flywheel, HardwareConstants.TuningConstants.FlywheelTuningVelocity),
            PrestageCommands.setPrestageVelocity(
                prestage, HardwareConstants.TestVelocities.prestageVelocity)),
        Commands.sequence(
            new WaitCommand(0.5),
            FeederCommands.setFeederVelocity(
                feeder, HardwareConstants.TestVelocities.feederVelocity),
            TransportCommands.setTransportVoltage(
                transport, HardwareConstants.TestVoltages.TransportTestVoltage),
            intakeRollerCommands.setRollerVoltage(
                intakeRoller, HardwareConstants.TestVoltages.intakeRollerAgitateVoltage)));
  }

  public static Command shootForTower(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      Feeder feeder,
      Transport transport,
      intakeRoller intakeRoller) {
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
            TransportCommands.setTransportVoltage(
                transport, HardwareConstants.TestVoltages.TransportTestVoltage),
            intakeRollerCommands.setRollerVoltage(
                intakeRoller, HardwareConstants.TestVoltages.intakeRollerAgitateVoltage)));
  }

  public static Command shootForTowerNoDelay(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      Feeder feeder,
      Transport transport,
      intakeRoller intakeRoller) {
    return Commands.parallel(
        FlywheelCommands.setFlywheelVelocity(
            flywheel, HardwareConstants.TowerConstants.FlywheelTowerVelocity),
        PrestageCommands.setPrestageVelocity(
            prestage, HardwareConstants.TestVelocities.prestageVelocity),
        HoodCommands.setHoodPos(hood, HardwareConstants.TowerConstants.hoodTowerPos),
        FeederCommands.setFeederVelocity(feeder, HardwareConstants.TestVelocities.feederVelocity),
        TransportCommands.setTransportVoltage(
            transport, HardwareConstants.TestVoltages.TransportTestVoltage),
        intakeRollerCommands.setRollerVoltage(
            intakeRoller, HardwareConstants.TestVoltages.intakeRollerAgitateVoltage));
  }

  public static Command shootToHub(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      Feeder feeder,
      Transport transport,
      intakeRoller intakeRoller,
      IntakePivot intakePivot) {
    return Commands.parallel(
            Commands.parallel(
                FlywheelCommands.setVelocityForHub(flywheel),
                PrestageCommands.setPrestageVelocity(
                    prestage, HardwareConstants.TestVelocities.prestageVelocity),
                HoodCommands.setHoodPosForHub(hood),
                IntakePivotCommands.jostlePivotByPos(intakePivot)),
            Commands.sequence(
                new WaitCommand(0.15),
                FeederCommands.setFeederVelocity(
                    feeder, HardwareConstants.TestVelocities.feederVelocity),
                TransportCommands.setTransportVoltage(
                    transport, HardwareConstants.TestVoltages.TransportTestVoltage),
                intakeRollerCommands.setRollerVoltage(
                    intakeRoller, HardwareConstants.TestVoltages.intakeRollerAgitateVoltage)))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
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
                FlywheelCommands.setPassVelocity(flywheel),
                PrestageCommands.setPrestageVelocity(
                    prestage, HardwareConstants.TestVelocities.prestageVelocity),
                HoodCommands.setHoodPos(hood, HardwareConstants.PassConstants.hoodPassPos)),
            Commands.sequence(
                new WaitCommand(0.5),
                FeederCommands.setFeederVelocity(
                    feeder, HardwareConstants.TestVelocities.feederVelocity),
                TransportCommands.setTransportVoltage(
                    transport, HardwareConstants.TestVoltages.TransportTestVoltage),
                intakeRollerCommands.setRollerVoltage(
                    intakeRoller, HardwareConstants.TestVoltages.intakeRollerAgitateVoltage)))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public static Command zonePassOrShoot(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      Feeder feeder,
      Transport transport,
      intakeRoller intakeRoller,
      IntakePivot intakePivot) {
    // if (RobotState.getInstance().zoneSafeToShoot()) {
    if (true) {
      Logger.recordOutput("Flywheel/shootOrPass", "shooting");
      return shootToHub(flywheel, prestage, hood, feeder, transport, intakeRoller, intakePivot);
    } else {
      Logger.recordOutput("Flywheel/shootOrPass", "passing");
      return pass(flywheel, prestage, hood, feeder, transport, intakeRoller);
    }
  }

  public static Command shootEndBehavior(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      Feeder feeder,
      Transport transport,
      intakeRoller intakeRoller,
      IntakePivot intakePivot) {
    return Commands.sequence(
        Commands.parallel(
            PrestageCommands.stop(prestage),
            FeederCommands.stop(feeder),
            TransportCommands.stop(transport),
            intakeRollerCommands.stopIntakeRoller(intakeRoller)),
        new WaitCommand(0.25),
        FlywheelCommands.setFlywheelVelocity(
            flywheel, HardwareConstants.TestVelocities.FlywheelVelocity));
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
            TransportCommands.setTransportVoltage(
                transport, HardwareConstants.TestVoltages.TransportTestVoltage))
        .finallyDo(
            () -> {
              feeder.setFeederVelocity(RotationsPerSecond.of(0));
              transport.setTransportVoltage(Volts.of(0));
            });
  }

  public static Command stopAll(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      Feeder feeder,
      Transport transport,
      intakeRoller intakeRoller) {
    return Commands.parallel(
        FlywheelCommands.stop(flywheel),
        PrestageCommands.stop(prestage),
        HoodCommands.setHoodPos(hood, HardwareConstants.TestPositions.hoodPos1Test),
        FeederCommands.stop(feeder),
        TransportCommands.stop(transport),
        intakeRollerCommands.stopIntakeRoller(intakeRoller));
  }
}
