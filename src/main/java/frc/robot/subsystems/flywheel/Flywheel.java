package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareConstants;
import frc.robot.Robot;
import frc.robot.subsystems.flywheel.io.FlywheelIO;
import frc.robot.subsystems.flywheel.io.ShooterIOInputsAutoLogged;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * The Shooter subsystem controls the robot's game piece launching mechanism.
 *
 * <p><b>Hardware:</b> 4x TalonFX (Phoenix 6) motors driving a flywheel (1 leader + 3 followers).
 */
public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final ShooterIOInputsAutoLogged inputs;
  private final FlywheelVisualizer visualizer;
  private LoggedNetworkNumber tuningRPM;

  /**
   * Supplier for the current hood angle. Set via {@link #setHoodAngleSupplier} after construction
   * so the Flywheel doesn't depend directly on the Hood subsystem. Defaults to 0° if not set.
   */
  private Supplier<Angle> hoodAngleSupplier = () -> Degrees.of(0);

  /**
   * Creates a new Shooter subsystem.
   *
   * @param shooterIO The hardware interface for shooter control
   */
  public Flywheel(FlywheelIO shooterIO) {
    this.io = shooterIO;
    inputs = new ShooterIOInputsAutoLogged();
    tuningRPM = new LoggedNetworkNumber("Tune/flywheel/tuningRPM", 20);
    visualizer = new FlywheelVisualizer();
  }

  /**
   * Sets the supplier that provides the current hood angle for trajectory visualization.
   *
   * <p>Call this from RobotContainer after both Flywheel and Hood are constructed:
   *
   * <pre>flywheel.setHoodAngleSupplier(hood::getPosition);</pre>
   *
   * @param supplier A supplier returning the current hood mechanism angle
   */
  public void setHoodAngleSupplier(Supplier<Angle> supplier) {
    this.hoodAngleSupplier = supplier;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    // Report flywheel current usage to the battery logger (leader + 4 followers)
    Robot.batteryLogger.reportCurrentUsage(
        "Flywheel/Leader",
        false,
        inputs.leaderSupplyCurrentAmps != null
            ? inputs.leaderSupplyCurrentAmps.in(Units.Amps)
            : 0.0);
    Robot.batteryLogger.reportCurrentUsage(
        "Flywheel/Follower1",
        false,
        inputs.follower1SupplyCurrentAmps != null
            ? inputs.follower1SupplyCurrentAmps.in(Units.Amps)
            : 0.0);
    Robot.batteryLogger.reportCurrentUsage(
        "Flywheel/Follower2",
        false,
        inputs.follower2SupplyCurrentAmps != null
            ? inputs.follower2SupplyCurrentAmps.in(Units.Amps)
            : 0.0);
    Robot.batteryLogger.reportCurrentUsage(
        "Flywheel/Follower3",
        false,
        inputs.follower3SupplyCurrentAmps != null
            ? inputs.follower3SupplyCurrentAmps.in(Units.Amps)
            : 0.0);
    Robot.batteryLogger.reportCurrentUsage(
        "Flywheel/Follower4",
        false,
        inputs.follower4SupplyCurrentAmps != null
            ? inputs.follower4SupplyCurrentAmps.in(Units.Amps)
            : 0.0);

    // Update trajectory visualization every loop
    visualizer.updateTrajectory(inputs.flywheelVelocity, hoodAngleSupplier.get());
  }

  public void setFlywheelVoltage(Voltage volts) {
    io.setFlywheelVoltage(volts);
  }

  public void setFlywheelVelocity(AngularVelocity velocity) {
    io.setFlywheelVelocity(velocity);
  }

  public void setFlywheelIdle() {
    io.setFlywheelVelocity(HardwareConstants.CompConstants.Velocities.flywheelIdleVelocity);
  }

  public void setSpeedForHub() {
    AngularVelocity velocity = ShotCalculator.getInstance().getFlywheelSpeedForAllianceHub();
    io.setFlywheelVelocity(velocity);
  }

  public void setSpeedForTarget(Translation3d target) {
    AngularVelocity velocity = ShotCalculator.getInstance().getFlywheelSpeedForTarget(target);
    io.setFlywheelVelocity(velocity);
  }

  public void setSpeedForDistance(Distance distance) {
    AngularVelocity velocity = ShotCalculator.getInstance().getFlywheelSpeedForDistance(distance);
    io.setFlywheelVelocity(velocity);
  }

  /**
   * Returns true if the flywheel velocity is within {@code toleranceRPM} of its current
   * closed-loop setpoint.
   *
   * <p>Returns {@code false} when no non-zero setpoint has been commanded (prevents a misleading
   * "ready" signal while the flywheel is at rest and closed-loop error is zero by default).
   *
   * <p>Use the tunable constants in {@link
   * frc.robot.HardwareConstants.CompConstants.Thresholds} (e.g. {@code hubFlywheelToleranceRPM},
   * {@code passFlywheelToleranceRPM}) to control how tight the tolerance is for each shot type.
   *
   * @param toleranceRPM Acceptable RPM error between actual velocity and the setpoint
   * @return true if the flywheel is within tolerance of its current setpoint
   */
  public boolean isFlywheelAtSetpoint(double toleranceRPM) {
    // Guard: treat as not-ready if no velocity target has been commanded yet
    if (inputs.closedLoopReference.in(RPM) <= 0) return false;
    return Math.abs(inputs.closedLoopError.in(RPM)) <= toleranceRPM;
  }

  public AngularVelocity getTuningRPM() {
    return RPM.of(tuningRPM.get());
  }

  public void setTuningRPM() {
    AngularVelocity velocity = getTuningRPM();
    io.setFlywheelVelocity(velocity);
  }
}
