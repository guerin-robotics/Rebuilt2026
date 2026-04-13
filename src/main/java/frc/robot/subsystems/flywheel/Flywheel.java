package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.FieldConstants;
import frc.robot.HardwareConstants;
import frc.robot.Robot;
import frc.robot.RobotState;
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

    // Log the currently running command for this subsystem
    Logger.recordOutput(
        "Flywheel/CurrentCommand",
        getCurrentCommand() != null ? getCurrentCommand().getName() : "none");
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

  public AngularVelocity getTuningRPM() {
    return RPM.of(tuningRPM.get());
  }

  public void setTuningRPM() {
    AngularVelocity velocity = getTuningRPM();
    io.setFlywheelVelocity(velocity);
  }

  // Definitely getting ahead of ourselves but when we get to shooting on the move...
  /**
   * For dynamic shooting, we'll want to give the flywheel a velocity based not only on its distance
   * from the hub (basic distance-based shooting), but also on the robot's movement. This function
   * calculates that velocity based on known quantities: robot odometry and hub position.
   *
   * @param hoodRadians The angle, given in radians, at which the hood position causes the fuel to
   *     shoot. Distinct from hood position, which is a 0.0-1.0 scale. Eventually, we'll want this
   *     to update based on distance, but for now, it's fine to keep it fixed and handle trajectory
   *     adjustments by RPM.
   */
  public void shootDynamic(double hoodRadians) {
    Translation2d fuelToGroundVector =
        new Translation2d(
            (FieldConstants.Hub.topCenterPoint.getX()
                - RobotState.getInstance().getEstimatedPose().getX()),
            (FieldConstants.Hub.topCenterPoint.getY()
                - RobotState.getInstance().getEstimatedPose().getY()));
    Translation2d robotToGroundVector =
        new Translation2d(
            (RobotState.getInstance().getFieldRelativeVelocity().vxMetersPerSecond),
            (RobotState.getInstance().getFieldRelativeVelocity().vyMetersPerSecond));
    Translation2d fuelToRobotVector =
        new Translation2d(
            (fuelToGroundVector.getX() - robotToGroundVector.getX()),
            (fuelToGroundVector.getY() - robotToGroundVector.getY()));
    // Our fuelToRobotVector gave us a linear velocity (m/s) which we now convert to rps using
    // flywheel rotations/meter
    LinearVelocity fuelVelocity =
        MetersPerSecond.of((fuelToRobotVector.getNorm() / Math.cos(hoodRadians)));
    AngularVelocity targetVelocity =
        RotationsPerSecond.of(
            fuelVelocity.magnitude() * FlywheelConstants.Mechanical.flywheelRotationsPerMeter);
    io.setFlywheelVelocity(targetVelocity);
  }
}
