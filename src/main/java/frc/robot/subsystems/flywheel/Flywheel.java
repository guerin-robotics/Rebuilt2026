package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.flywheel.io.FlywheelIO;
import frc.robot.subsystems.flywheel.io.ShooterIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * The Shooter subsystem controls the robot's game piece launching mechanism.
 *
 * <p><b>Hardware:</b> 4x TalonFX (Phoenix 6) motors driving a flywheel (1 leader + 3 followers).
 */
public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final ShooterIOInputsAutoLogged inputs;

  /**
   * Creates a new Shooter subsystem.
   *
   * @param shooterIO The hardware interface for shooter control
   */
  public Flywheel(FlywheelIO shooterIO) {
    this.io = shooterIO;
    inputs = new ShooterIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }

  /** Manual duty cycle control - for testing motor direction/wiring only. */
  public void setFlywheelDutyCycle(double output) {
    io.setFlywheelDutyCycle(output);
  }

  public void setFlywheelVoltage(Voltage volts) {
    io.setFlywheelVoltage(volts);
  }

  /**
   * Sets flywheel speed using feedforward-only control (no PID). Computes voltage from kS and kV,
   * then applies via open-loop voltage output.
   *
   * @param targetSpeed Desired angular velocity (e.g., RPM.of(3000))
   */
  public void setFlywheelSpeed(AngularVelocity targetSpeed) {
    io.setFlywheelSpeed(targetSpeed);
  }

  public void setFlywheelTorque(AngularVelocity velocity) {
    io.setFlywheelTorque(velocity);
  }

  public void setSpeedForHub() {
    AngularVelocity velocity = ShotCalculator.getInstance().getFlywheelSpeedForAllianceHub();
    io.setFlywheelTorque(velocity);
  }

  public void setSpeedForTarget(Translation3d target) {
    AngularVelocity velocity = ShotCalculator.getInstance().getFlywheelSpeedForTarget(target);
    io.setFlywheelTorque(velocity);
  }

  public void setSpeedForDistance(Distance distance) {
    AngularVelocity velocity = ShotCalculator.getInstance().getFlywheelSpeedForDistance(distance);
    io.setFlywheelTorque(velocity);
  }

  // Definitely getting ahead of ourselves but when we get to shooting on the move...
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
    // This quantity is the angle to the goal - it needs passed to the drivetrain
    Rotation2d targetHeading = fuelToRobotVector.getAngle();
    // Our fuelToRobotVector gave us a linear velocity (m/s) which we now convert to rps using
    // flywheel rotations/meter
    LinearVelocity fuelVelocity =
        MetersPerSecond.of((fuelToRobotVector.getNorm() / Math.cos(hoodRadians)));
    AngularVelocity targetVelocity =
        RotationsPerSecond.of(
            fuelVelocity.magnitude() * FlywheelConstants.Mechanical.flywheelRotationsPerMeter);
    io.setFlywheelTorque(targetVelocity);
  }

  /**
   * Checks if flywheel is above acceptable threshold below target velocity
   *
   * @param targetRPM
   * @return true if flywheel is above threshold, false otherwise
   */
  public boolean isFlywheelAtVelocity(AngularVelocity targetRPM) {
    if (inputs.flywheelVelocity == null) {
      inputs.flywheelVelocity = RotationsPerSecond.of(0);
    }
    if (inputs.flywheelVelocity.magnitude()
        >= (targetRPM.magnitude() - FlywheelConstants.Limits.velocityThreshold.magnitude())) {
      return true;
    } else {
      return false;
    }
  }
}
