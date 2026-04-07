package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HardwareConstants;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeederCommands;
import frc.robot.commands.FlywheelCommands;
import frc.robot.commands.HoodCommands;
import frc.robot.commands.IntakePivotCommands;
import frc.robot.commands.PrestageCommands;
import frc.robot.commands.TransportCommands;
import frc.robot.commands.intakeRollerCommands;
import frc.robot.commands.autos.utils.AutoContext;

/**
 * Reusable command factories for autonomous routines.
 *
 * <p>These methods build small, composable commands that can be triggered by Choreo event markers
 * or chained together inside {@link AutoPaths}. Keeping them here avoids duplicating logic across
 * multiple auto routines.
 */
public class AutoCommands {

  // ─── private constructor ─ utility class ────────────────────────────────────
  private AutoCommands() {}

  // ─── Intake ─────────────────────────────────────────────────────────────────

  /**
   * Deploys the intake pivot to the down position and runs the intake rollers + transport.
   *
   * <p>When the command ends (interrupted or finished), the rollers and transport stop. The pivot is
   * left in the deployed position so a separate retract command can handle it.
   */
  public static Command deployAndRunIntake(AutoContext ctx) {
    return Commands.parallel(
        IntakePivotCommands.setPivotRotations(
            ctx.intakePivot(), HardwareConstants.CompConstants.Positions.pivotDownPos),
        Commands.startEnd(
            () -> {
              ctx.intakeRoller()
                  .setRollerVoltage(
                      HardwareConstants.CompConstants.Voltages.intakeRollerVoltage);
              ctx.transport()
                  .setTransportVoltage(HardwareConstants.CompConstants.Voltages.transportVoltage);
            },
            () -> {
              ctx.intakeRoller().setRollerVoltage(Volts.of(0));
              ctx.transport().setTransportVoltage(Volts.of(0));
            },
            ctx.intakeRoller(),
            ctx.transport()));
  }

  /**
   * Deploys the intake pivot only (no rollers). Useful as a lightweight event-marker action.
   *
   * <p>This is a fire-and-forget command — it sets the position and finishes immediately.
   */
  public static Command deployIntake(AutoContext ctx) {
    return IntakePivotCommands.setPivotRotations(
        ctx.intakePivot(), HardwareConstants.CompConstants.Positions.pivotDownPos);
  }

  /** Retracts the intake pivot to the up position. */
  public static Command retractIntake(AutoContext ctx) {
    return IntakePivotCommands.setPivotRotations(
        ctx.intakePivot(), HardwareConstants.CompConstants.Positions.pivotUpPos);
  }

  // ─── Shooting ───────────────────────────────────────────────────────────────

  /**
   * Full shoot-to-hub sequence used at the end of auto paths.
   *
   * <p>Spins up the flywheel, sets hood angle, engages prestage, then after the flywheel spin-up
   * wait feeds the ball through the upper/lower feeders, transport, and intake rollers. Also aligns
   * the drivetrain heading toward the alliance hub while shooting.
   */
  public static Command shootSequence(AutoContext ctx) {
    return Commands.parallel(
        // Align drivetrain heading toward the hub while shooting
        DriveCommands.joystickDriveAtAngle(
            ctx.drive(), () -> 0, () -> 0, () -> RobotState.getInstance().getAngleToAllianceHub()),
        // Spin up flywheel, set hood, engage prestage
        FlywheelCommands.setVelocityForHub(ctx.flywheel()),
        PrestageCommands.setPrestageVelocity(
            ctx.prestage(), HardwareConstants.CompConstants.Velocities.prestageVelocity),
        HoodCommands.setHoodPosForHub(ctx.hood()),
        // After spin-up delay, feed the ball
        Commands.sequence(
            new WaitCommand(HardwareConstants.CompConstants.Waits.flywheelSpinupSeconds),
            Commands.parallel(
                FeederCommands.setLowerFeederVelocity(
                    ctx.lowerFeeder(),
                    HardwareConstants.CompConstants.Velocities.feederVelocity),
                FeederCommands.setUpperFeederVelocity(
                    ctx.upperFeeder(),
                    HardwareConstants.CompConstants.Velocities.feederVelocity),
                TransportCommands.setTransportVoltage(
                    ctx.transport(), HardwareConstants.CompConstants.Voltages.transportVoltage),
                intakeRollerCommands.setRollerVoltage(
                    ctx.intakeRoller(),
                    HardwareConstants.CompConstants.Voltages.intakeRollerAgitateVoltage))));
  }

  /**
   * Stops all scoring-related subsystems.
   *
   * <p>Call this after a shoot sequence to cleanly wind everything down.
   */
  public static Command stopAll(AutoContext ctx) {
    return Commands.parallel(
        FlywheelCommands.stop(ctx.flywheel()),
        PrestageCommands.stop(ctx.prestage()),
        HoodCommands.setHoodPos(
            ctx.hood(), HardwareConstants.CompConstants.Positions.hoodDownPos),
        FeederCommands.stopUpper(ctx.upperFeeder()),
        FeederCommands.stopLower(ctx.lowerFeeder()),
        TransportCommands.stop(ctx.transport()),
        intakeRollerCommands.stopIntakeRoller(ctx.intakeRoller()));
  }
}
