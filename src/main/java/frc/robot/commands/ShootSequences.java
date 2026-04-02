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
            flywheel.setFlywheelVelocityCommand(
                HardwareConstants.TuningConstants.FlywheelTuningVelocity),
            prestage.setPrestageVelocityCommand(
                HardwareConstants.CompConstants.Velocities.prestageVelocity),
            hood.setHoodPosCommand(HardwareConstants.TuningConstants.HoodTuningPos),
            intakePivot.jostlePivotByPosCommand()),
        Commands.sequence(
            new WaitCommand(HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
            Commands.parallel(
                feeder.setFeederVelocityCommand(
                    HardwareConstants.CompConstants.Velocities.feederVelocity),
                transport.setTransportVoltageCommand(
                    HardwareConstants.CompConstants.Voltages.transportVoltage),
                intakeRoller.setRollerVoltageCommand(
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
            flywheel.setFlywheelVelocityCommand(
                HardwareConstants.TowerConstants.FlywheelTowerVelocity),
            prestage.setPrestageVelocityCommand(
                HardwareConstants.CompConstants.Velocities.prestageVelocity),
            hood.setHoodPosCommand(HardwareConstants.TowerConstants.hoodTowerPos)),
        Commands.sequence(
            new WaitCommand(0.15),
            feeder.setFeederVelocityCommand(
                HardwareConstants.CompConstants.Velocities.feederVelocity),
            transport.setTransportVoltageCommand(
                HardwareConstants.CompConstants.Voltages.transportVoltage),
            intakeRoller.setRollerVoltageCommand(
                HardwareConstants.CompConstants.Voltages.intakeRollerAgitateVoltage)));
  }

  public static Command shootToHub(
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
            flywheel.setVelocityForHubCommand(),
            prestage.setPrestageVelocityCommand(
                HardwareConstants.CompConstants.Velocities.prestageVelocity),
            hood.setHoodPosForHubCommand()),
        Commands.sequence(
            new WaitCommand(HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
            Commands.parallel(
                feeder.setFeederVelocityCommand(
                    HardwareConstants.CompConstants.Velocities.feederVelocity),
                transport.setTransportVoltageCommand(
                    HardwareConstants.CompConstants.Voltages.transportVoltage),
                intakeRoller.setRollerVoltageCommand(
                    HardwareConstants.CompConstants.Voltages.intakeRollerAgitateVoltage))),
        Commands.sequence(new WaitCommand(1.5), intakePivot.compressPivotCommand()))
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
            flywheel.setPassVelocityCommand(),
            prestage.setPrestageVelocityCommand(
                HardwareConstants.CompConstants.Velocities.prestageVelocity),
            hood.setHoodPosCommand(HardwareConstants.PassConstants.hoodPassPos)),
        Commands.sequence(
            new WaitCommand(HardwareConstants.CompConstants.Waits.passSpinUpSeconds),
            Commands.parallel(
                feeder.setFeederVelocityCommand(
                    HardwareConstants.CompConstants.Velocities.feederVelocity),
                transport.setTransportVoltageCommand(
                    HardwareConstants.CompConstants.Voltages.transportVoltage),
                intakeRoller.setRollerVoltageCommand(
                    HardwareConstants.CompConstants.Voltages.intakeRollerAgitateVoltage))),
        Commands.sequence(new WaitCommand(1.5), intakePivot.compressPivotCommand()));
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
        flywheel.flywheelIdleCommand(),
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
        shootToHub(flywheel, prestage, hood, feeder, transport, intakeRoller, intakePivot),
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
            prestage.stopCommand(),
            feeder.stopCommand(),
            transport.stopCommand(),
            intakeRoller.stopIntakeRollerCommand(),
            intakePivot.setPivotRotationsCommand(
                HardwareConstants.CompConstants.Positions.pivotDownPos)),
        new WaitCommand(0.25),
        flywheel.setFlywheelVelocityCommand(
            HardwareConstants.CompConstants.Velocities.flywheelIdleVelocity));
  }

  public static Command stopAll(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      Feeder feeder,
      Transport transport,
      intakeRoller intakeRoller) {
    return Commands.parallel(
        flywheel.stopCommand(),
        prestage.stopCommand(),
        hood.setHoodPosCommand(HardwareConstants.CompConstants.Positions.hoodDownPos),
        feeder.stopCommand(),
        transport.stopCommand(),
        intakeRoller.stopIntakeRollerCommand());
  }
}
