package frc.robot.subsystems.prestage;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.prestage.io.PrestageIO;
import frc.robot.subsystems.prestage.io.PrestageIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Prestage extends SubsystemBase {
  private final PrestageIO io;

  private final PrestageIOInputsAutoLogged inputs;

  public Prestage(PrestageIO io) {
    this.io = io;
    this.inputs = new PrestageIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Prestage", inputs);

    // Report prestage current usage to the battery logger (left + right motors)
    Robot.batteryLogger.reportCurrentUsage(
        "Prestage/Left",
        false,
        inputs.prestageLeftSupplyAmps != null ? inputs.prestageLeftSupplyAmps.in(Units.Amps) : 0.0);
    Robot.batteryLogger.reportCurrentUsage(
        "Prestage/Right",
        false,
        inputs.prestageRightSupplyAmps != null
            ? inputs.prestageRightSupplyAmps.in(Units.Amps)
            : 0.0);
  }

  public void setPrestageVoltage(Voltage volts) {
    io.setPrestageVoltage(volts);
  }

  public void setPrestageVelocity(AngularVelocity prestageVelo) {
    io.setPrestageVelocity(prestageVelo);
  }

  /**
   * Returns true if both prestage motors are within {@code toleranceRPM} of their current
   * closed-loop setpoints.
   *
   * <p>Returns {@code false} when no non-zero setpoint has been commanded, preventing a false
   * "ready" signal when the prestage is stopped and closed-loop error defaults to zero.
   *
   * <p>Use the tunable constant {@link
   * frc.robot.HardwareConstants.CompConstants.Thresholds#prestageToleranceRPM} when calling this
   * method.
   *
   * @param toleranceRPM Acceptable RPM error between actual velocity and the setpoint
   * @return true if both prestage motors are within tolerance of their setpoints
   */
  public boolean isPrestageAtSetpoint(double toleranceRPM) {
    // Guard: treat as not-ready if no velocity target has been commanded yet
    if (inputs.prestageLeftClosedLoopReference.in(RPM) <= 0) return false;
    double leftErrorRPM = Math.abs(inputs.prestageLeftClosedLoopError.in(RPM));
    double rightErrorRPM = Math.abs(inputs.prestageRightClosedLoopError.in(RPM));
    return leftErrorRPM < toleranceRPM && rightErrorRPM < toleranceRPM;
  }
}
