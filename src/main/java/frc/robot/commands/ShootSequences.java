package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.HardwareConstants;
import frc.robot.Triggers;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakeRoller.intakeRoller;
import frc.robot.subsystems.prestage.Prestage;
import frc.robot.subsystems.transport.Transport;
import frc.robot.util.HubShiftUtil;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class ShootSequences {

  public static Command mapTuningShoot(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      IntakePivot intakePivot,
      Feeder feeder,
      Transport transport,
      intakeRoller intakeRoller) {
    return Commands.parallel(
        Commands.parallel(
            FlywheelCommands.setFlywheelVelocity(
                flywheel, HardwareConstants.TuningConstants.FlywheelTuningVelocity),
            PrestageCommands.setPrestageVelocity(
                prestage, HardwareConstants.CompConstants.Velocities.prestageVelocity),
            HoodCommands.setHoodPos(hood, HardwareConstants.TuningConstants.HoodTuningPos),
            IntakePivotCommands.jostlePivotByPos(intakePivot)),
        Commands.sequence(
            new WaitCommand(HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
            Commands.parallel(
                FeederCommands.setFeederVelocity(
                    feeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
                TransportCommands.setTransportVoltage(
                    transport, HardwareConstants.CompConstants.Voltages.transportVoltage),
                intakeRollerCommands.setRollerVoltage(
                    intakeRoller,
                    HardwareConstants.CompConstants.Voltages.intakeRollerAgitateVoltage))));
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
                prestage, HardwareConstants.CompConstants.Velocities.prestageVelocity),
            HoodCommands.setHoodPos(hood, HardwareConstants.TowerConstants.hoodTowerPos)),
        Commands.sequence(
            new WaitCommand(0.15),
            FeederCommands.setFeederVelocity(
                feeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
            TransportCommands.setTransportVoltage(
                transport, HardwareConstants.CompConstants.Voltages.transportVoltage),
            intakeRollerCommands.setRollerVoltage(
                intakeRoller,
                HardwareConstants.CompConstants.Voltages.intakeRollerAgitateVoltage)));
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
            prestage, HardwareConstants.CompConstants.Velocities.prestageVelocity),
        HoodCommands.setHoodPos(hood, HardwareConstants.TowerConstants.hoodTowerPos),
        FeederCommands.setFeederVelocity(
            feeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
        TransportCommands.setTransportVoltage(
            transport, (HardwareConstants.CompConstants.Voltages.transportVoltage)),
        intakeRollerCommands.setRollerVoltage(
            intakeRoller, HardwareConstants.CompConstants.Voltages.intakeRollerAgitateVoltage));
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
                    prestage, HardwareConstants.CompConstants.Velocities.prestageVelocity),
                HoodCommands.setHoodPosForShoot(hood)),
            Commands.sequence(
                new WaitCommand(HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
                Commands.parallel(
                    FeederCommands.setFeederVelocity(
                        feeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
                    TransportCommands.setTransportVoltage(
                        transport, HardwareConstants.CompConstants.Voltages.transportVoltage),
                    intakeRollerCommands.setRollerVoltage(
                        intakeRoller,
                        (HardwareConstants.CompConstants.Voltages.intakeRollerVoltage)))),
            Commands.sequence(
                new WaitCommand(HardwareConstants.CompConstants.Waits.waitToCompressSeconds),
                IntakePivotCommands.compressPivot(intakePivot)))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public static Command autoShootToHub(
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
                    prestage, HardwareConstants.CompConstants.Velocities.prestageVelocity),
                HoodCommands.setHoodPosForHub(hood),
                IntakePivotCommands.jostlePivotByPos(intakePivot)),
            Commands.sequence(
                new WaitCommand(0.15),
                FeederCommands.setFeederVelocity(
                    feeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
                TransportCommands.setTransportVoltage(
                    transport, HardwareConstants.CompConstants.Voltages.transportVoltage),
                intakeRollerCommands.setRollerVoltage(
                    intakeRoller, HardwareConstants.CompConstants.Voltages.intakeRollerVoltage)))
        .finallyDo(
            () -> {
              FlywheelCommands.flywheelIdle(flywheel);
              PrestageCommands.setPrestageVelocity(prestage, RotationsPerSecond.of(0));
              FeederCommands.setFeederVelocity(feeder, RotationsPerSecond.of(0));
              TransportCommands.setTransportVoltage(transport, Volts.of(0));
              HoodCommands.setHoodPos(hood, HardwareConstants.CompConstants.Positions.hoodDownPos);
            });
  }

  public static Command pass(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      Feeder feeder,
      Transport transport,
      IntakePivot intakePivot,
      intakeRoller intakeRoller) {
    return Commands.parallel(
            Commands.parallel(
                FlywheelCommands.setPassVelocity(flywheel),
                PrestageCommands.setPrestageVelocity(
                    prestage, HardwareConstants.CompConstants.Velocities.prestageVelocity),
                HoodCommands.setHoodPos(hood, HardwareConstants.PassConstants.hoodPassPos)),
            Commands.sequence(
                new WaitCommand(HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
                FeederCommands.setFeederVelocity(
                    feeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
                TransportCommands.setTransportVoltage(
                    transport, HardwareConstants.CompConstants.Voltages.transportVoltage),
                intakeRollerCommands.setRollerVoltage(
                    intakeRoller, HardwareConstants.CompConstants.Voltages.intakeRollerVoltage),
                IntakePivotCommands.jostlePivotByPos(intakePivot)))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public static Command zoneAndTimePassOrShoot(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      Feeder feeder,
      Transport transport,
      intakeRoller intakeRoller,
      IntakePivot intakePivot,
      Trigger override) {
    return Commands.either(
        shootToHub(flywheel, prestage, hood, feeder, transport, intakeRoller, intakePivot),
        pass(flywheel, prestage, hood, feeder, transport, intakePivot, intakeRoller),
        () -> {
          boolean safe = Triggers.getInstance().isShootSafe();
          Logger.recordOutput("Flywheel/shootOrPass", safe ? "shooting" : "passing");
          boolean timeCorrect = HubShiftUtil.getShiftedShiftInfo().active();
          Logger.recordOutput("Flywheel/timeCorrect", timeCorrect);
          boolean overriden = override.getAsBoolean();
          Logger.recordOutput("Flywheel/overriden", overriden);
          if (!overriden) {
            return safe && timeCorrect;
          } else {
            return safe;
          }
        });
  }

  public static Command zonePassOrShoot(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      Feeder feeder,
      Transport transport,
      intakeRoller intakeRoller,
      IntakePivot intakePivot) {
    return Commands.either(
        shootToHub(flywheel, prestage, hood, feeder, transport, intakeRoller, intakePivot),
        pass(flywheel, prestage, hood, feeder, transport, intakePivot, intakeRoller),
        () -> {
          boolean safe = Triggers.getInstance().isShootSafe();
          Logger.recordOutput("Flywheel/shootOrPass", safe ? "shooting" : "passing");
          return safe;
        });
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
            intakeRollerCommands.stopIntakeRoller(intakeRoller),
            IntakePivotCommands.setPivotRotations(
                intakePivot, HardwareConstants.CompConstants.Positions.pivotDownPos)),
        new WaitCommand(0.25),
        FlywheelCommands.setFlywheelVelocity(
            flywheel, HardwareConstants.CompConstants.Velocities.flywheelIdleVelocity));
  }

  public static Command FirstSet(Flywheel flywheel, Prestage prestage, Hood hood) {
    return Commands.parallel(
            FlywheelCommands.setFlywheelVelocity(
                flywheel, HardwareConstants.TowerConstants.FlywheelTowerVelocity),
            PrestageCommands.setPrestageVelocity(
                prestage, HardwareConstants.CompConstants.Velocities.prestageVelocity),
            HoodCommands.setHoodPos(hood, HardwareConstants.TowerConstants.hoodTowerPos))
        .finallyDo(
            () -> {
              flywheel.setFlywheelVelocity(RotationsPerSecond.of(0));
              prestage.setPrestageVelocity(RotationsPerSecond.of(0));
              hood.setHoodPos(HardwareConstants.CompConstants.Positions.hoodDownPos);
            });
  }

  public static Command SecondSet(Feeder feeder, Transport transport) {
    return Commands.parallel(
            FeederCommands.setFeederVelocity(
                feeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
            TransportCommands.setTransportVoltage(
                transport, HardwareConstants.CompConstants.Voltages.transportVoltage))
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
        HoodCommands.setHoodPos(hood, HardwareConstants.CompConstants.Positions.hoodDownPos),
        FeederCommands.stop(feeder),
        TransportCommands.stop(transport),
        intakeRollerCommands.stopIntakeRoller(intakeRoller));
  }

  public static Command flipAlliance() {
    if (HubShiftUtil.getAllianceWinOverride().isEmpty()) {
      // () -> Optional.empty() should be something else but I'm not sure what
      return Commands.runOnce(() -> HubShiftUtil.setAllianceWinOverride(() -> Optional.empty()));
    } else {
      return Commands.runOnce(() -> HubShiftUtil.setAllianceWinOverride(() -> Optional.empty()));
    }
  }
}
