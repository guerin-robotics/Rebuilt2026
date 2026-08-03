package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HardwareConstants;
import frc.robot.Triggers;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakeRoller.intakeRoller;
import frc.robot.subsystems.lowerFeeder.LowerFeeder;
import frc.robot.subsystems.prestage.Prestage;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.upperFeeder.UpperFeeder;
import org.littletonrobotics.junction.Logger;

public class ShootSequences {

  public static Command autoShootToHub(
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
            // Roller duty cycle, identical to the teleop shoot binding: held at zero from the
            // start of the sequence, then agitate from the loop the feeders are gated on. It is
            // gated on the same condition the feeder branch below waits for — keep the two in
            // sync or the roller will lead or lag the shot.
            //
            // .asProxy() keeps intakeRoller OUT of this command's requirements. PathPlannerAuto
            // requires the union of every command in the auto for the auto's whole duration, so
            // without the proxy the roller's always-on default command could never run between
            // shots and the roller would be dead for the rest of the auto.
            intakeRollerCommands
                .setVoltageAfterWait(
                    intakeRoller,
                    HardwareConstants.CompConstants.Voltages.intakeRollerAgitateVoltage,
                    flywheel.isFlywheelSpunUp)
                .asProxy(),
            Commands.sequence(
                Commands.waitUntil(
                        flywheel.isFlywheelSpunUp.and(Triggers.getInstance().isAlignedLooser))
                    .withTimeout(HardwareConstants.CompConstants.Waits.spinUpTimeOut),
                Commands.parallel(
                    FeederCommands.setLowerFeederVelocity(
                        lowerFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
                    FeederCommands.setUpperFeederVelocity(
                        upperFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity),
                    TransportCommands.setTransportVelocity(
                        transport, HardwareConstants.CompConstants.Velocities.transportVelocity),
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
            // Proxied for the same reason as in autoShootToHub: stopAll runs between path
            // segments in every auto, so if it contributed intakeRoller to the auto's
            // requirements the roller's default command could never run.
            intakeRollerCommands.stopIntakeRoller(intakeRoller).asProxy())
        .withName("StopAll");
  }
}
