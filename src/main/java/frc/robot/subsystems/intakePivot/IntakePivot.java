package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private final IntakePivotIOInputsAutoLogged inputs;
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
        Math.abs(inputs.intakePivotPosition.in(Rotations) - goalPositionRotations)
            < IntakePivotConstants.Visualization.POSITION_TOLERANCE_ROTATIONS;

    // Update the visualizer every loop
    visualizer.update(inputs.intakePivotPosition.in(Rotations), goalPositionRotations, atGoal);
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
    return inputs.intakePivotPosition.in(Rotations);
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
    double currentPos = inputs.intakePivotPosition.in(Rotations);
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
    new WaitCommand(0.25);
    io.setPivotPosition(IntakePivotConstants.Mechanical.pivotDegreesDown);
    new WaitCommand(0.25);
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
}
