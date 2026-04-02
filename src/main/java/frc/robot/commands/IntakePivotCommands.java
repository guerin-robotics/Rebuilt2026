package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakePivot.IntakePivot;

/**
 * Command factories for the intake pivot subsystem.
 *
 * <p><b>DEPRECATED:</b> Prefer calling commands directly on the subsystem instance (e.g., {@code
 * intakePivot.setPivotVoltageCommand(...)}).
 */
public class IntakePivotCommands {

  /**
   * @deprecated Use {@link IntakePivot#setPivotVoltageCommand(Voltage)} instead.
   */
  public static Command setPivotVoltage(IntakePivot intakePivot, Voltage voltage) {
    return intakePivot.setPivotVoltageCommand(voltage);
  }

  /**
   * @deprecated Use {@link IntakePivot#setPivotVelocityCommand(AngularVelocity)} instead.
   */
  public static Command setPivotVelocity(IntakePivot intakePivot, AngularVelocity pivotVelo) {
    return intakePivot.setPivotVelocityCommand(pivotVelo);
  }

  /**
   * @deprecated Use {@link IntakePivot#stopPivotCommand()} instead.
   */
  public static Command stopPivot(IntakePivot intakePivot) {
    return intakePivot.stopPivotCommand();
  }

  /**
   * @deprecated Use {@link IntakePivot#setPivotRotationsCommand(double)} instead.
   */
  public static Command setPivotRotations(IntakePivot intakePivot, double angleRotations) {
    return intakePivot.setPivotRotationsCommand(angleRotations);
  }

  /**
   * @deprecated Use {@link IntakePivot#jostlePivotByCurrentCommand(AngularVelocity,
   *     AngularVelocity, double, double)} instead.
   */
  public static Command jostlePivotByCurrent(
      IntakePivot intakePivot,
      AngularVelocity upVelocity,
      AngularVelocity downVelocity,
      double degreesDown,
      double seconds) {
    return intakePivot.jostlePivotByCurrentCommand(upVelocity, downVelocity, degreesDown, seconds);
  }

  /**
   * @deprecated Use {@link IntakePivot#jostlePivotByPosCommand()} instead.
   */
  public static Command jostlePivotByPos(IntakePivot intakePivot) {
    return intakePivot.jostlePivotByPosCommand();
  }

  /**
   * @deprecated Use {@link IntakePivot#compressPivotCommand()} instead.
   */
  public static Command compressPivot(IntakePivot intakePivot) {
    return intakePivot.compressPivotCommand();
  }

  /**
   * @deprecated Use {@link IntakePivot#zeroPivotCommand()} instead.
   */
  public static Command zeroPivot(IntakePivot intakePivot) {
    return intakePivot.zeroPivotCommand();
  }
}
