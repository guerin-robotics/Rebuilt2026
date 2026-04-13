package frc.robot.subsystems.intakePivot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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

    // Report intake pivot current usage to the battery logger
    Robot.batteryLogger.reportCurrentUsage(
        "Intake/Pivot",
        false,
        inputs.intakePivotSupplyCurrent != null
            ? inputs.intakePivotSupplyCurrent.in(Units.Amps)
            : 0.0);

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
}
