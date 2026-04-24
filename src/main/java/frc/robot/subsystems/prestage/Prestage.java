package frc.robot.subsystems.prestage;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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

  /**
   * The final velocity setpoint most recently commanded via a velocity control call.
   *
   * <p>Updated by {@link #setPrestageVelocity}. Reset to zero by {@link #setPrestageVoltage} so
   * that {@link #isPrestageAtSetpoint} returns {@code false} when voltage control is active.
   */
  private AngularVelocity targetVelocity = RotationsPerSecond.of(0);

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
    // Voltage control has no velocity target — reset so isPrestageAtSetpoint returns false.
    targetVelocity = RotationsPerSecond.of(0);
    io.setPrestageVoltage(volts);
  }

  public void setPrestageVelocity(AngularVelocity prestageVelo) {
    targetVelocity = prestageVelo;
    io.setPrestageVelocity(prestageVelo);
  }

  /**
   * Returns true if both prestage motors are within {@code toleranceRPM} of the most recently
   * commanded velocity setpoint.
   *
   * <p>Compares the measured velocities against the last {@link #setPrestageVelocity} call, not the
   * motion-profile's current closed-loop reference. This ensures the check only passes once the
   * motors have fully reached the final target.
   *
   * <p>Returns {@code false} when no velocity has been commanded (e.g. after voltage control), so
   * the safety timeout in the shoot sequence will always kick in as a fallback.
   *
   * <p>Tolerance constant: {@link
   * frc.robot.HardwareConstants.CompConstants.Thresholds#prestageToleranceRPM}.
   *
   * @param toleranceRPM Acceptable RPM error between measured and commanded velocity
   * @return true if both prestage motors are within tolerance of the commanded setpoint
   */
  public boolean isPrestageAtSetpoint(double toleranceRPM) {
    // Guard: not ready if no velocity target has been commanded yet
    if (targetVelocity.in(RPM) <= 0) return false;
    double leftErrorRPM =
        Math.abs(inputs.prestageLeftVelocity.in(RPM) - targetVelocity.in(RPM));
    double rightErrorRPM =
        Math.abs(inputs.prestageRightVelocity.in(RPM) - targetVelocity.in(RPM));
    return leftErrorRPM <= toleranceRPM && rightErrorRPM <= toleranceRPM;
  }
}
