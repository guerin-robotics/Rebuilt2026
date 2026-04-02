package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.intakePivot.io.IntakePivotIO;
import frc.robot.subsystems.intakePivot.io.IntakePivotIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for the intake pivot mechanism.
 *
 * <p>The pivot rotates the intake assembly between a stowed (retracted) position and an extended
 * (deployed) position. Position is measured in rotations via a CANcoder.
 */
public class IntakePivot extends SubsystemBase {

  private final IntakePivotIO io;
  public final IntakePivotIOInputsAutoLogged inputs;
  private final IntakePivotVisualizer visualizer;

  /** The last goal position set by the user, in rotations. Used for visualization. */
  private double goalPositionRotations = 0.0;

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
    this.inputs = new IntakePivotIOInputsAutoLogged();
    this.visualizer = new IntakePivotVisualizer("IntakePivot");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake Pivot", inputs);

    // Determine if we are within tolerance of our goal
    boolean atGoal =
        Math.abs(inputs.intakePivotPosition - goalPositionRotations)
            < IntakePivotConstants.Visualization.POSITION_TOLERANCE_ROTATIONS;

    // Update the visualizer every loop
    visualizer.update(inputs.intakePivotPosition, goalPositionRotations, atGoal);
  }

  public void setPivotVoltage(Voltage volts) {
    io.setPivotVoltage(volts);
  }

  public void setPivotVelocity(AngularVelocity pivotVelo) {
    io.setPivotVelocity(pivotVelo);
  }

  public void setPivotPosition(double positionRotations) {
    this.goalPositionRotations = positionRotations;
    io.setPivotPosition(positionRotations);
  }

  public void zeroPivotEncoder() {
    io.zeroPivotEncoder();
  }

  /**
   * Returns the current pivot position in rotations.
   *
   * @return current position from the CANcoder in rotations
   */
  public double getPosition() {
    return inputs.intakePivotPosition;
  }

  /**
   * Jostle the intake by current. If stator current is below the threshold, drive downward.
   * Otherwise, retract by a set amount and pause briefly.
   */
  public void intakeJostleByCurrent(
      AngularVelocity upVelocity,
      AngularVelocity downVelocity,
      double degreesDown,
      double seconds) {
    double currentPos = inputs.intakePivotPosition;
    if (inputs.intakePivotStatorCurrent.in(Amps)
        < IntakePivotConstants.Mechanical.pivotJostleCurrentLimit) {
      io.setPivotVelocity(downVelocity);
    } else {
      setPivotPosition(currentPos + degreesDown);
      new WaitCommand(seconds);
    }
  }

  public void intakeJostleByPos() {
    io.setPivotPosition(IntakePivotConstants.Mechanical.pivotJostleDegreesUp);
    // new WaitCommand(1);
    // io.setPivotPosition(IntakePivotConstants.Mechanical.pivotDegreesDown);
    // new WaitCommand(1);
  }

  /**
   * Slowly drive the pivot toward the home position until stator current indicates a hard stop,
   * then zero the encoder.
   */
  public void intakeHome(AngularVelocity homeVelo) {
    if (inputs.intakePivotStatorCurrent.in(Amps) > 0.5) {
      io.setPivotVelocity(homeVelo);
    } else {
      io.zeroPivotEncoder();
    }
  }

  // ==================== COMMAND FACTORIES ====================

  /** Run the pivot at a given voltage; stop (0 V) when the command ends. */
  public Command setPivotVoltageCommand(Voltage voltage) {
    return Commands.startEnd(
        () -> setPivotVoltage(voltage), () -> setPivotVoltage(Volts.of(0)), this);
  }

  /** Run the pivot at a given velocity; stop (0 rps) when the command ends. */
  public Command setPivotVelocityCommand(AngularVelocity pivotVelo) {
    return Commands.startEnd(
        () -> setPivotVelocity(pivotVelo), () -> setPivotVelocity(RotationsPerSecond.of(0)), this);
  }

  /** Stop the pivot immediately (set voltage to 0). */
  public Command stopPivotCommand() {
    return Commands.runOnce(() -> setPivotVoltage(Volts.of(0)), this);
  }

  /** Move the pivot to a specific position in rotations. */
  public Command setPivotRotationsCommand(double angleRotations) {
    return Commands.runOnce(() -> setPivotPosition(angleRotations), this);
  }

  /** Jostle the pivot by monitoring stator current and pulsing the position. */
  public Command jostlePivotByCurrentCommand(
      AngularVelocity upVelocity,
      AngularVelocity downVelocity,
      double degreesDown,
      double seconds) {
    return Commands.startEnd(
        () -> intakeJostleByCurrent(upVelocity, downVelocity, degreesDown, seconds),
        () -> setPivotVoltage(Volts.of(0)),
        this);
  }

  /** Repeatedly jostle the pivot by alternating positions. */
  public Command jostlePivotByPosCommand() {
    return Commands.sequence(
            setPivotRotationsCommand(HardwareConstants.CompConstants.Positions.pivotJostleUpPos),
            new WaitCommand(0.25),
            setPivotRotationsCommand(HardwareConstants.CompConstants.Positions.pivotDownPos),
            new WaitCommand(0.25))
        .repeatedly()
        .finallyDo(() -> setPivotPosition(HardwareConstants.CompConstants.Positions.pivotDownPos));
  }

  /** Compress the pivot by voltage pulse + position hold, repeating. */
  public Command compressPivotCommand() {
    return Commands.sequence(
            Commands.deadline(new WaitCommand(0.75), setPivotVoltageCommand(Volts.of(1.7))),
            new WaitCommand(0.5),
            setPivotRotationsCommand(HardwareConstants.CompConstants.Positions.pivotDownPos))
        .repeatedly()
        .finallyDo(() -> setPivotPosition(HardwareConstants.CompConstants.Positions.pivotDownPos));
  }

  /** Zero the pivot encoder at the current position. */
  public Command zeroPivotCommand() {
    return Commands.runOnce(() -> zeroPivotEncoder());
  }
}
