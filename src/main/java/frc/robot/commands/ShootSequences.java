package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakeRoller.intakeRoller;
import frc.robot.subsystems.lowerFeeder.LowerFeeder;
import frc.robot.subsystems.prestage.Prestage;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.upperFeeder.UpperFeeder;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class ShootSequences {

  /**
   * Auto hub shot: spin up the flywheel/hood/prestage immediately, then feed once the shooter is
   * ready AND the drivetrain is pointed at the hub.
   *
   * <p>Mirrors the teleop gating, where the feeders, agitator, and compress are all held off until
   * {@code isAlignedLooser} is true. Uses the standard Spinup → Align → Act budget: phase 1 waits
   * for spin-up, phase 2 waits for alignment with the remainder of {@code alignmentTimeoutSeconds},
   * so feeding always begins within that total budget even if alignment never arrives.
   *
   * @param isAligned true when the drivetrain is pointed at the hub within tolerance. Supplied by
   *     the caller so the alignment target always matches whatever the paired drive command is
   *     aiming at.
   */
  public static Command autoShootToHub(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      UpperFeeder upperFeeder,
      LowerFeeder lowerFeeder,
      Transport transport,
      intakeRoller intakeRoller,
      IntakePivot intakePivot,
      BooleanSupplier isAligned) {
    return Commands.parallel(
            Commands.runOnce(() -> Logger.recordOutput("RobotState/shooting", true)),
            Commands.parallel(
                FlywheelCommands.setVelocityForHub(flywheel),
                PrestageCommands.setPrestageVelocity(
                    prestage, HardwareConstants.CompConstants.Velocities.prestageVelocity),
                HoodCommands.setHoodPosForHub(hood)),
            Commands.sequence(
                Commands.waitUntil(flywheel.isFlywheelSpunUp)
                    .withTimeout(HardwareConstants.CompConstants.Waits.spinUpTimeOut),
                // Hold the feed until we're aimed. Timeout is the remaining alignment budget, so
                // a shot that never lines up still fires rather than stalling the auto.
                Commands.waitUntil(isAligned)
                    .withTimeout(
                        HardwareConstants.CompConstants.Waits.alignmentTimeoutSeconds
                            - HardwareConstants.CompConstants.Waits.spinUpTimeOut),
                // Records whether we actually got aligned or fell through on the timeout.
                Commands.runOnce(
                    () ->
                        Logger.recordOutput(
                            "RobotState/autoShotAlignedAtFeed", isAligned.getAsBoolean())),
                Commands.parallel(
                    FeederCommands.setLowerFeederVelocity(
                        lowerFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
                    FeederCommands.setUpperFeederVelocity(
                        upperFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
                    TransportCommands.setTransportVelocity(
                        transport, HardwareConstants.CompConstants.Velocities.transportVelocity),
                    intakeRollerCommands.setRollerVoltage(
                        intakeRoller,
                        HardwareConstants.CompConstants.Voltages.intakeRollerAgitateVoltage),
                    IntakePivotCommands.autoPivotCompress(intakePivot))))
        // .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("ShootToHub");
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
