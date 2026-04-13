package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
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

  /** The last goal position set by the user. Used for visualization. */
  private Angle goalPosition = Rotations.of(0.0);

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
    double currentRotations = inputs.intakePivotPosition.in(Rotations);
    double goalRotations = goalPosition.in(Rotations);
    boolean atGoal =
        Math.abs(currentRotations - goalRotations)
            < IntakePivotConstants.Visualization.POSITION_TOLERANCE_ROTATIONS;

    // Update the visualizer every loop
    visualizer.update(currentRotations, goalRotations, atGoal);

    // Log the currently running command for this subsystem
    Logger.recordOutput(
        "Intake Pivot/CurrentCommand",
        getCurrentCommand() != null ? getCurrentCommand().getName() : "none");
  }

  public void setPivotVoltage(Voltage volts) {
    io.setPivotVoltage(volts);
  }

  public void setPivotVelocity(AngularVelocity pivotVelo) {
    io.setPivotVelocity(pivotVelo);
  }

  /** Sets the pivot to a target position using closed-loop control. */
  public void setPivotPosition(Angle position) {
    this.goalPosition = position;
    io.setPivotPosition(position);
  }

  public void zeroPivotEncoder() {
    io.zeroPivotEncoder();
  }

  /**
   * Returns the current pivot position.
   *
   * @return current position from the CANcoder
   */
  public Angle getPosition() {
    return inputs.intakePivotPosition;
  }
}
