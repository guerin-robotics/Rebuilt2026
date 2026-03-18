package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.intakePivot.IntakePivot;

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
        intakePivot);
  }

  /** Run the pivot at a given velocity; stop (0 rps) when the command ends. */
  public static Command setPivotVelocity(IntakePivot intakePivot, AngularVelocity pivotVelo) {
    return Commands.startEnd(
        () -> intakePivot.setPivotVelocity(pivotVelo),
        () -> intakePivot.setPivotVelocity(RotationsPerSecond.of(0)),
        intakePivot);
  }

  /** Stop the pivot immediately (set voltage to 0). */
  public static Command stopPivot(IntakePivot intakePivot) {
    return Commands.runOnce(() -> intakePivot.setPivotVoltage(Volts.of(0)), intakePivot);
  }

  /** Move the pivot to a specific position in rotations. */
  public static Command setPivotRotations(IntakePivot intakePivot, double angleRotations) {
    return Commands.runOnce(() -> intakePivot.setPivotPosition(angleRotations), intakePivot);
  }

  /** Jostle the pivot by monitoring stator current and pulsing the position. */
  public static Command jostlePivotByCurrent(
      IntakePivot intakePivot,
      AngularVelocity upVelocity,
      AngularVelocity downVelocity,
      double degreesDown,
      double seconds) {
    return Commands.startEnd(
        () -> intakePivot.intakeJostleByCurrent(upVelocity, downVelocity, degreesDown, seconds),
        () -> intakePivot.setPivotVoltage(Volts.of(0)),
        intakePivot);
  }

  public static Command jostlePivotByPos(IntakePivot intakePivot) {
    return Commands.sequence(
            setPivotRotations(intakePivot, HardwareConstants.TestPositions.intakeJostleTest),
            new WaitCommand(0.25),
            setPivotRotations(intakePivot, HardwareConstants.TestPositions.intakeDegreesDownTest),
            new WaitCommand(0.25))
        .repeatedly();
  }

  /** Zero the pivot encoder at the current position. */
  public static Command zeroPivot(IntakePivot intakePivot) {
    return Commands.runOnce(() -> intakePivot.zeroPivotEncoder());
  }
}
