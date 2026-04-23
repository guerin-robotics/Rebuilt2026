package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.ContinuousConditionalCommand;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.intakePivot.IntakePivot;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Command factories for the intake pivot subsystem.
 *
 * <p>Each method returns a Command that controls the pivot. Commands properly stop the mechanism
 * when they end (either naturally or by interruption).
 */
public class IntakePivotCommands {

  /** Run the pivot at a given voltage; stop (0 V) when the command ends. */
  public static Command setPivotVoltage(IntakePivot intakePivot, Voltage voltage) {
    return Commands.startEnd(
            () -> intakePivot.setPivotVoltage(voltage),
            () -> intakePivot.setPivotVoltage(Volts.of(0)),
            intakePivot)
        .withName("IntakePivotVoltage_" + voltage.in(Volts) + "V");
  }

  /** Run the pivot at a given velocity; stop (0 rps) when the command ends. */
  public static Command setPivotVelocity(IntakePivot intakePivot, AngularVelocity pivotVelo) {
    return Commands.startEnd(
            () -> intakePivot.setPivotVelocity(pivotVelo),
            () -> intakePivot.setPivotVelocity(RotationsPerSecond.of(0)),
            intakePivot)
        .withName("IntakePivotVelocity");
  }

  /** Stop the pivot immediately (set voltage to 0). */
  public static Command stopPivot(IntakePivot intakePivot) {
    return Commands.runOnce(() -> intakePivot.setPivotVoltage(Volts.of(0)), intakePivot)
        .withName("IntakePivotStop");
  }

  /** Move the pivot to a specific position. */
  public static Command setPivotPosition(IntakePivot intakePivot, Angle position) {
    return Commands.runOnce(() -> intakePivot.setPivotPosition(position), intakePivot)
        .withName("IntakePivotSetPos_" + position);
  }

  /**
   * Compress the pivot to jostle game pieces into position.
   *
   * <p>Uses a {@link BooleanSupplier} evaluated at schedule time (not creation time) so the correct
   * branch is always chosen when the command actually runs. {@link ContinuousConditionalCommand}
   * also re-evaluates the condition while running, so if the condition changes mid-execution the
   * command will switch branches automatically.
   *
   * @param intakePivot The intake pivot subsystem
   * @param halfHopper If the operator hits the half-hopper override button, the intake runs full compress immediately
   */
  public static Command compressPivot(IntakePivot intakePivot, BooleanSupplier halfHopper) {
    Logger.recordOutput("RobotState/IntakePivot", "JostleCalled");

    // Branch A: run full compress after initial wait
    Command halfHopperBranch =
        Commands.sequence(
                new WaitCommand(HardwareConstants.CompConstants.Waits.waitToCompressSeconds),
                setPivotPosition(
                    intakePivot, HardwareConstants.CompConstants.Positions.pivotJostleUpPos))
            .withName("IntakePivotCompress_HalfHopper");

    // Branch B: initial wait, then half compress, then full compress
    Command fullHopperBranch =
        Commands.sequence(
                new WaitCommand(HardwareConstants.CompConstants.Waits.waitToCompressSeconds),
                setPivotPosition(
                    intakePivot, HardwareConstants.CompConstants.Positions.pivotJostleFirstPos),
                new WaitCommand((HardwareConstants.CompConstants.Waits.waitToDropSeconds)),
                setPivotPosition(
                    intakePivot, HardwareConstants.CompConstants.Positions.pivotDownPos),
                new WaitCommand(HardwareConstants.CompConstants.Waits.waitBetweenCompressSeconds),
                setPivotPosition(
                    intakePivot, HardwareConstants.CompConstants.Positions.pivotJostleUpPos))
            .withName("IntakePivotCompress_FullHopper");

    // ContinuousConditionalCommand evaluates halfHopper each loop, so the right
    // branch is always active — even if the condition changes while the command is running.
    return new ContinuousConditionalCommand(halfHopperBranch, fullHopperBranch, halfHopper)
        .withName("IntakePivotCompress");
  }

  // If no boolean given, defaults to full hopper
  public static Command compressPivot(IntakePivot intakePivot) {
    return compressPivot(intakePivot, () -> false);
  }

  public static Command manualPivotCompress(IntakePivot intakePivot) {
    return Commands.sequence(
                setPivotPosition(
                    intakePivot, HardwareConstants.CompConstants.Positions.pivotJostleFirstPos),
                new WaitCommand((HardwareConstants.CompConstants.Waits.waitToDropSeconds)),
                setPivotPosition(
                    intakePivot, HardwareConstants.CompConstants.Positions.pivotDownPos),
                new WaitCommand(HardwareConstants.CompConstants.Waits.waitBetweenCompressSeconds),
                setPivotPosition(
                    intakePivot, HardwareConstants.CompConstants.Positions.pivotJostleUpPos))
            .withName("IntakePivotCompress_SkipWait");
  }

  public static Command autoPivotCompress(IntakePivot intakePivot) {
    return Commands.sequence(
            new WaitCommand(HardwareConstants.CompConstants.Waits.autoWaitToCompressSeconds),
                setPivotPosition(
                    intakePivot, HardwareConstants.CompConstants.Positions.pivotJostleFirstPos),
            new WaitCommand((HardwareConstants.CompConstants.Waits.autoWaitToDropSeconds)),
                setPivotPosition(
                    intakePivot, HardwareConstants.CompConstants.Positions.pivotDownPos),
            new WaitCommand(HardwareConstants.CompConstants.Waits.autoWaitBetweenCompressSeconds),
                setPivotPosition(
                    intakePivot, HardwareConstants.CompConstants.Positions.pivotJostleUpPos))
        .withName("IntakePivotCompress");
  }

  /** Zero the pivot encoder at the current position. */
  public static Command zeroPivot(IntakePivot intakePivot) {
    return Commands.runOnce(() -> intakePivot.zeroPivotEncoder()).withName("IntakePivotZero");
  }
}
