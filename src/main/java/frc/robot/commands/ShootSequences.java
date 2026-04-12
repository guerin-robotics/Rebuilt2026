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
                TransportCommands.setTransportVelocity(
                    transport, HardwareConstants.CompConstants.Velocities.transportVelocity),
                intakeRollerCommands.setRollerVelocity(
                    intakeRoller,
                    HardwareConstants.CompConstants.Velocities.intakeRollerVelocity))));
  }

  /**
   * Tuning shoot WITHOUT hood control. The hood is left to the button panel so the operator can
   * increment/reset the hood position independently while shooting.
   *
   * <p>Runs: flywheel at tuning velocity, prestage, intake jostle, and (after spinup delay) feeders
   * + transport + intake agitate.
   */
  public static Command mapTuningShootNoHood(
      Flywheel flywheel,
      Prestage prestage,
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
            IntakePivotCommands.jostlePivotByPos(intakePivot)),
        Commands.sequence(
            new WaitCommand(HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
            Commands.parallel(
                FeederCommands.setLowerFeederVelocity(
                    lowerFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
                FeederCommands.setUpperFeederVelocity(
                    upperFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
                TransportCommands.setTransportVelocity(
                    transport, HardwareConstants.CompConstants.Velocities.transportVelocity),
                intakeRollerCommands.setRollerVelocity(
                    intakeRoller,
                    HardwareConstants.CompConstants.Velocities.intakeRollerVelocity))));
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
            FeederCommands.setLowerFeederVelocity(
                lowerFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
            FeederCommands.setUpperFeederVelocity(
                upperFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
            TransportCommands.setTransportVelocity(
                transport, HardwareConstants.CompConstants.Velocities.transportVelocity),
            intakeRollerCommands.setRollerVelocity(
                intakeRoller, HardwareConstants.CompConstants.Velocities.intakeRollerVelocity)));
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
                FeederCommands.setLowerFeederVelocity(
                    lowerFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
                FeederCommands.setUpperFeederVelocity(
                    upperFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
                TransportCommands.setTransportVelocity(
                    transport, HardwareConstants.CompConstants.Velocities.transportVelocity),
                intakeRollerCommands.setRollerVelocity(
                    intakeRoller,
                    HardwareConstants.CompConstants.Velocities.intakeRollerVelocity))),
        Commands.sequence(new WaitCommand(1.5), IntakePivotCommands.compressPivot(intakePivot)))
    // .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
    ;
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
                FeederCommands.setLowerFeederVelocity(
                    lowerFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
                FeederCommands.setUpperFeederVelocity(
                    upperFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
                TransportCommands.setTransportVelocity(
                    transport, HardwareConstants.CompConstants.Velocities.transportVelocity),
                intakeRollerCommands.setRollerVelocity(
                    intakeRoller,
                    HardwareConstants.CompConstants.Velocities.intakeRollerVelocity))),
        Commands.sequence(new WaitCommand(1.5), IntakePivotCommands.compressPivot(intakePivot)));
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
          boolean zoneSafe = Triggers.getInstance().isShootSafeZone().getAsBoolean();
          return !zoneSafe;
        });
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
        intakeRollerCommands.stopIntakeRoller(intakeRoller));
  }
}
