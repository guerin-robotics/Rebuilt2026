package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.ContinuousConditionalCommand;
import frc.robot.HardwareConstants;
import frc.robot.Triggers;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakeRoller.intakeRoller;
import frc.robot.subsystems.prestage.Prestage;
import frc.robot.subsystems.transport.Transport;
import frc.robot.util.HubShiftUtil;
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

  public static Command shootToHub(
      Drive drive,
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      Feeder feeder,
      Transport transport,
      intakeRoller intakeRoller,
      IntakePivot intakePivot) {
    Logger.recordOutput("RobotState/shooting", true);
    return Commands.parallel(
        Commands.parallel(
            FlywheelCommands.setVelocityForHub(flywheel),
            PrestageCommands.setPrestageVelocity(
                prestage, HardwareConstants.CompConstants.Velocities.prestageVelocity),
            HoodCommands.setHoodPosForHub(hood)),
        Commands.sequence(
            new WaitCommand(HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
            Commands.parallel(
                FeederCommands.setFeederVelocity(
                    feeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
                TransportCommands.setTransportVoltage(
                    transport, HardwareConstants.CompConstants.Voltages.transportVoltage),
                intakeRollerCommands.setRollerVoltage(
                    intakeRoller,
                    (HardwareConstants.CompConstants.Voltages.intakeRollerAgitateVoltage)))),
        Commands.sequence(new WaitCommand(1.5), IntakePivotCommands.compressPivot(intakePivot)),
        Commands.sequence(new WaitCommand(2), Commands.runOnce(drive::stopWithX, drive)))
    // .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
    ;
  }

  public static Command pass(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      Feeder feeder,
      Transport transport,
      IntakePivot intakePivot,
      intakeRoller intakeRoller) {
    Logger.recordOutput("RobotState/shooting", false);
    return Commands.parallel(
        Commands.parallel(
            FlywheelCommands.setPassVelocity(flywheel),
            PrestageCommands.setPrestageVelocity(
                prestage, HardwareConstants.CompConstants.Velocities.prestageVelocity),
            HoodCommands.setHoodPos(hood, HardwareConstants.PassConstants.hoodPassPos)),
        Commands.sequence(
            new WaitCommand(HardwareConstants.CompConstants.Waits.passSpinUpSeconds),
            Commands.parallel(
                FeederCommands.setFeederVelocity(
                    feeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
                TransportCommands.setTransportVoltage(
                    transport, HardwareConstants.CompConstants.Voltages.transportVoltage),
                intakeRollerCommands.setRollerVoltage(
                    intakeRoller,
                    HardwareConstants.CompConstants.Voltages.intakeRollerAgitateVoltage))),
        Commands.sequence(new WaitCommand(1.5), IntakePivotCommands.compressPivot(intakePivot)));
  }

  public static Command passOrIdle(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      Feeder feeder,
      Transport transport,
      intakeRoller intakeRoller,
      IntakePivot intakePivot) {
    return new ContinuousConditionalCommand(
        pass(flywheel, prestage, hood, feeder, transport, intakePivot, intakeRoller),
        FlywheelCommands.flywheelIdle(flywheel),
        () -> {
          boolean zoneSafe = Triggers.getInstance().isShootSafeZone().getAsBoolean();
          return !zoneSafe;
        });
  }

  public static Command zonePassOrShoot(
      Drive drive,
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      Feeder feeder,
      Transport transport,
      intakeRoller intakeRoller,
      IntakePivot intakePivot) {
    return Commands.either(
        shootToHub(drive, flywheel, prestage, hood, feeder, transport, intakeRoller, intakePivot),
        pass(flywheel, prestage, hood, feeder, transport, intakePivot, intakeRoller),
        () -> {
          boolean safe = Triggers.getInstance().isShootSafeZone().getAsBoolean();
          Logger.recordOutput("Flywheel/shootOrPass", safe ? "shooting" : "passing");
          return safe;
        });
  }

  public static Command zoneAndTimePassOrShoot(
      Drive drive,
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      Feeder feeder,
      Transport transport,
      intakeRoller intakeRoller,
      IntakePivot intakePivot) {
    return new ContinuousConditionalCommand(
        passOrIdle(flywheel, prestage, hood, feeder, transport, intakeRoller, intakePivot),
        zonePassOrShoot(
            drive, flywheel, prestage, hood, feeder, transport, intakeRoller, intakePivot),
        () -> {
          boolean zoneSafe = Triggers.getInstance().isShootSafeZone().getAsBoolean();
          boolean timeSafe = Triggers.getInstance().isShootSafeTime().getAsBoolean();
          boolean disabled = HubShiftUtil.disabled;
          return (!zoneSafe && !disabled) || (!timeSafe && !disabled);
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
}
