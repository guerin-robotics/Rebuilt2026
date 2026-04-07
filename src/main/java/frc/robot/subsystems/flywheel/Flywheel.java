package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.FieldConstants;
import frc.robot.HardwareConstants;
import frc.robot.RobotState;
import frc.robot.Triggers;
import frc.robot.subsystems.flywheel.io.FlywheelIO;
import frc.robot.subsystems.flywheel.io.ShooterIOInputsAutoLogged;
import frc.robot.util.HubShiftUtil;
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
  private LoggedNetworkNumber tuningRPM;

  /**
   * Creates a new Shooter subsystem.
   *
   * @param shooterIO The hardware interface for shooter control
   */
  public Flywheel(FlywheelIO shooterIO) {
    this.io = shooterIO;
    inputs = new ShooterIOInputsAutoLogged();
    tuningRPM = new LoggedNetworkNumber("Tune/flywheel/tuningRPM", 20);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
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

  public Translation3d getPassTarget() {
    Translation3d passTarget;
    // if (RobotState.getInstance().getEstimatedPose().getY()
    //     > (AllianceFlipUtil.applyY(FieldConstants.fieldWidth / 2))) {
    //   passTarget =
    //       new Translation3d(
    //           (AllianceFlipUtil.applyY(FieldConstants.LinesVertical.neutralZoneNear / 2)),
    //           ((3 * AllianceFlipUtil.applyY(FieldConstants.LinesHorizontal.center)) / 4),
    //           0);
    // } else {
    //   passTarget =
    //       new Translation3d(
    //           (AllianceFlipUtil.applyY(FieldConstants.LinesVertical.neutralZoneNear / 2)),
    //           ((AllianceFlipUtil.applyY(FieldConstants.LinesHorizontal.center)) / 4),
    //           0);
    // }
    if (DriverStation.getAlliance().isEmpty()
        || DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      if (RobotState.getInstance().getEstimatedPose().getY() > (FieldConstants.fieldWidth / 2)) {
        passTarget =
            new Translation3d(Meters.of(13.373).magnitude(), Meters.of(6.136).magnitude(), 0);
      } else {
        passTarget =
            new Translation3d(Meters.of(12.950).magnitude(), Meters.of(2.293).magnitude(), 0);
      }
    } else {
      if (RobotState.getInstance().getEstimatedPose().getY() < (FieldConstants.fieldWidth / 2)) {
        passTarget =
            new Translation3d(Meters.of(2.993).magnitude(), Meters.of(2.114).magnitude(), 0);
      } else {
        passTarget =
            new Translation3d(Meters.of(3.135).magnitude(), Meters.of(6.129).magnitude(), 0);
      }
    }
    Logger.recordOutput("Flywheel/passTarget", passTarget);
    return passTarget;
  }

  public AngularVelocity getTuningRPM() {
    return RPM.of(tuningRPM.get());
  }

  public void setTuningRPM() {
    AngularVelocity velocity = getTuningRPM();
    io.setFlywheelVelocity(velocity);
  }

  // Returns angle to hub if shooting, returns angle to passing target if passing
  // Includes time logic, both zone and time logic overrideable
  // Returns current rotation if in alliance zone but hub inactive
  public Rotation2d getShootAngleForZoneAndTime() {
    if (!HubShiftUtil.disabled) {
      if (Triggers.getInstance().isShootClear()) {
        Logger.recordOutput("RobotState/zoneSafeToShoot", true);
        return RobotState.getInstance().getAngleToAllianceHub();
      } else {
        if (Triggers.getInstance().isShootSafeZone()) {
          return RobotState.getInstance().getEstimatedPose().getRotation();
        } else {
          Logger.recordOutput("RobotState/zoneSafeToShoot", false);
          return RobotState.getInstance()
              .getAngleToTarget(new Translation2d(getPassTarget().getX(), getPassTarget().getY()));
        }
      }
    } else {
      Logger.recordOutput("RobotState/shootAngleOverriden", true);
      return RobotState.getInstance().getAngleToAllianceHub();
    }
  }

  // No time logic
  public Rotation2d getShootAngleForZone() {
    if (Triggers.getInstance().isShootSafeZone()) {
      Logger.recordOutput("RobotState/zoneSafeToShoot", true);
      return RobotState.getInstance().getAngleToAllianceHub();
    } else {
      Logger.recordOutput("RobotState/zoneSafeToShoot", false);
      return RobotState.getInstance()
          .getAngleToTarget(new Translation2d(getPassTarget().getX(), getPassTarget().getY()));
    }
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
