package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.ContinuousConditionalCommand;
import frc.robot.HardwareConstants;
import frc.robot.Triggers;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakeRoller.intakeRoller;
import frc.robot.subsystems.lowerFeeder.LowerFeeder;
import frc.robot.subsystems.prestage.Prestage;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.upperFeeder.UpperFeeder;
import frc.robot.util.HubShiftUtil;
import org.littletonrobotics.junction.Logger;

public class ShootSequences {

  public static Command mapTuningShoot(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      IntakePivot intakePivot,
      UpperFeeder upperFeeder,
      LowerFeeder lowerFeeder,
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
                    FeederCommands.setLowerFeederVelocity(
                        lowerFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
                    FeederCommands.setUpperFeederVelocity(
                        upperFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
                    TransportCommands.setTransportVoltage(
                        transport, HardwareConstants.CompConstants.Voltages.transportVoltage),
                    intakeRollerCommands.setRollerVoltage(
                        intakeRoller,
                        HardwareConstants.CompConstants.Voltages.intakeRollerAgitateVoltage))))
        .withName("MapTuningShoot");
  }

  public static Command shootForTower(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      UpperFeeder upperFeeder,
      LowerFeeder lowerFeeder,
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
                Commands.parallel(
                    FeederCommands.setLowerFeederVelocity(
                        lowerFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
                    FeederCommands.setUpperFeederVelocity(
                        upperFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
                    TransportCommands.setTransportVoltage(
                        transport, HardwareConstants.CompConstants.Voltages.transportVoltage),
                    intakeRollerCommands.setRollerVoltage(
                        intakeRoller,
                        HardwareConstants.CompConstants.Voltages.intakeRollerAgitateVoltage))))
        .withName("ShootForTower");
  }

  public static Command shootToHub(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      UpperFeeder upperFeeder,
      LowerFeeder lowerFeeder,
      Transport transport,
      intakeRoller intakeRoller,
      IntakePivot intakePivot) {
    return Commands.parallel(
            Commands.runOnce(() -> Logger.recordOutput("RobotState/shooting", true)),
            Commands.parallel(
                FlywheelCommands.setVelocityForHub(flywheel),
                PrestageCommands.setPrestageVelocity(
                    prestage, HardwareConstants.CompConstants.Velocities.prestageVelocity),
                HoodCommands.setHoodPosForHub(hood)),
            Commands.sequence(
                new WaitCommand(HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
                Commands.parallel(
                    FeederCommands.setLowerFeederVelocity(
                        lowerFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
                    FeederCommands.setUpperFeederVelocity(
                        upperFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
                    TransportCommands.setTransportVoltage(
                        transport, HardwareConstants.CompConstants.Voltages.transportVoltage),
                    intakeRollerCommands.setRollerVoltage(
                        intakeRoller,
                        (HardwareConstants.CompConstants.Voltages.intakeRollerAgitateVoltage)))),
            Commands.sequence(new WaitCommand(1.5), IntakePivotCommands.compressPivot(intakePivot)))
        // .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("ShootToHub");
  }

  public static Command pass(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      UpperFeeder upperFeeder,
      LowerFeeder lowerFeeder,
      Transport transport,
      IntakePivot intakePivot,
      intakeRoller intakeRoller) {
    return Commands.parallel(
            Commands.runOnce(() -> Logger.recordOutput("RobotState/shooting", false)),
            Commands.parallel(
                FlywheelCommands.setPassVelocity(flywheel),
                PrestageCommands.setPrestageVelocity(
                    prestage, HardwareConstants.CompConstants.Velocities.prestageVelocity),
                HoodCommands.setHoodPos(hood, HardwareConstants.PassConstants.hoodPassPos)),
            Commands.sequence(
                new WaitCommand(HardwareConstants.CompConstants.Waits.passSpinUpSeconds),
                Commands.parallel(
                    FeederCommands.setLowerFeederVelocity(
                        lowerFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
                    FeederCommands.setUpperFeederVelocity(
                        upperFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
                    TransportCommands.setTransportVoltage(
                        transport, HardwareConstants.CompConstants.Voltages.transportVoltage),
                    intakeRollerCommands.setRollerVoltage(
                        intakeRoller,
                        HardwareConstants.CompConstants.Voltages.intakeRollerAgitateVoltage))),
            Commands.sequence(new WaitCommand(1.5), IntakePivotCommands.compressPivot(intakePivot)))
        .withName("Pass");
  }

  public static Command passOrIdle(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      UpperFeeder upperFeeder,
      LowerFeeder lowerFeeder,
      Transport transport,
      intakeRoller intakeRoller,
      IntakePivot intakePivot) {
    return new ContinuousConditionalCommand(
            pass(
                flywheel,
                prestage,
                hood,
                upperFeeder,
                lowerFeeder,
                transport,
                intakePivot,
                intakeRoller),
            FlywheelCommands.flywheelIdle(flywheel),
            () -> {
              boolean zoneSafe = Triggers.getInstance().isShootSafeZone.getAsBoolean();
              return !zoneSafe;
            })
        .withName("PassOrIdle");
  }

  public static Command zonePassOrShoot(
      Drive drive,
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      UpperFeeder upperFeeder,
      LowerFeeder lowerFeeder,
      Transport transport,
      intakeRoller intakeRoller,
      IntakePivot intakePivot) {
    return Commands.either(
            shootToHub(
                flywheel,
                prestage,
                hood,
                upperFeeder,
                lowerFeeder,
                transport,
                intakeRoller,
                intakePivot),
            pass(
                flywheel,
                prestage,
                hood,
                upperFeeder,
                lowerFeeder,
                transport,
                intakePivot,
                intakeRoller),
            () -> {
              boolean safe = Triggers.getInstance().isShootSafeZone.getAsBoolean();
              Logger.recordOutput("Flywheel/shootOrPass", safe ? "shooting" : "passing");
              return safe;
            })
        .withName("ZonePassOrShoot");
  }

  public static Command zoneAndTimePassOrShoot(
      Drive drive,
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      UpperFeeder upperFeeder,
      LowerFeeder lowerFeeder,
      Transport transport,
      intakeRoller intakeRoller,
      IntakePivot intakePivot) {
    return new ContinuousConditionalCommand(
            passOrIdle(
                flywheel,
                prestage,
                hood,
                upperFeeder,
                lowerFeeder,
                transport,
                intakeRoller,
                intakePivot),
            zonePassOrShoot(
                drive,
                flywheel,
                prestage,
                hood,
                upperFeeder,
                lowerFeeder,
                transport,
                intakeRoller,
                intakePivot),
            () -> {
              boolean zoneSafe = Triggers.getInstance().isShootSafeZone.getAsBoolean();
              boolean timeSafe = Triggers.getInstance().isShootSafeTime.getAsBoolean();
              boolean disabled = HubShiftUtil.disabled;
              return (!zoneSafe && !disabled) || (!timeSafe && !disabled);
            })
        .withName("ZoneAndTimePassOrShoot");
  }

  public static Command shootEndBehavior(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      UpperFeeder upperFeeder,
      LowerFeeder lowerFeeder,
      Transport transport,
      intakeRoller intakeRoller,
      IntakePivot intakePivot) {
    return Commands.sequence(
            Commands.parallel(
                PrestageCommands.stop(prestage),
                FeederCommands.stopUpper(upperFeeder),
                FeederCommands.stopLower(lowerFeeder),
                TransportCommands.stop(transport),
                intakeRollerCommands.stopIntakeRoller(intakeRoller),
                IntakePivotCommands.setPivotPosition(
                    intakePivot, HardwareConstants.CompConstants.Positions.pivotDownPos)),
            new WaitCommand(0.25),
            FlywheelCommands.stop(flywheel))
        .withName("ShootEndBehavior");
  }

  public static Command stopAll(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      UpperFeeder upperFeeder,
      LowerFeeder lowerFeeder,
      Transport transport,
      intakeRoller intakeRoller) {
    return Commands.parallel(
            FlywheelCommands.stop(flywheel),
            PrestageCommands.stop(prestage),
            HoodCommands.stowHood(hood),
            FeederCommands.stopUpper(upperFeeder),
            FeederCommands.stopLower(lowerFeeder),
            TransportCommands.stop(transport),
            intakeRollerCommands.stopIntakeRoller(intakeRoller))
        .withName("StopAll");
  }
}
