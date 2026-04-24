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
            Commands.sequence(
                Commands.waitUntil(flywheel.isFlywheelSpunUp.and(prestage.isPrestageSpunUp))
                    .withTimeout(HardwareConstants.CompConstants.Waits.spinUpTimeOut),
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
            IntakePivotCommands.autoPivotCompress(intakePivot))
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
