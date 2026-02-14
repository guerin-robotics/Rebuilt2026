package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.shooter.io.ShooterIO;

/**
 * The Shooter subsystem controls the robot's game piece launching mechanism.
 *
 * <p><b>Hardware:</b> 4x TalonFX (Phoenix 6) motors driving a flywheel (1 leader + 3 followers).
 *
 * <p><b>Software Features:</b>
 *
 * <ul>
 *   <li>Closed-loop (PID) velocity control via TalonFX Slot0
 *   <li>Distance-based shooting using InterpolatingDoubleTreeMap
 *   <li>FuelSim trajectory visualization for AdvantageScope
 *   <li>SysId integration for feedforward characterization
 * </ul>
 */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();
  private AngularVelocity targetFlywheelSpeed;
  private final SysIdRoutine sysId;

  /**
   * Creates a new Shooter subsystem.
   *
   * @param shooterIO The hardware interface for shooter control
   */
  public Shooter(ShooterIO shooterIO) {
    this.io = shooterIO;
    this.stopFlywheels();

    // Configure SysId for feedforward characterization
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> SmartDashboard.putString("Shooter/SysId/State", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Log current and target speeds
    if (inputs.flywheelVelocity != null) {
      double currentRPM = inputs.flywheelVelocity.in(RevolutionsPerSecond) * 60.0;
      SmartDashboard.putNumber("Shooter/FlywheelSpeed/Current_RPM", currentRPM);
      SmartDashboard.putNumber(
          "Shooter/FlywheelSpeed/Current_RadPerSec", inputs.flywheelVelocity.in(RadiansPerSecond));
    }

    double targetRPM = targetFlywheelSpeed.in(RevolutionsPerSecond) * 60.0;
    SmartDashboard.putNumber("Shooter/FlywheelSpeed/Desired_RPM", targetRPM);
    SmartDashboard.putNumber(
        "Shooter/FlywheelSpeed/Desired_RadPerSec", targetFlywheelSpeed.in(RadiansPerSecond));
    SmartDashboard.putBoolean("Shooter/AtTargetSpeed", areFlywheelsAtTargetSpeed());
  }

  /** Stops the flywheel by setting target speed to zero. */
  public void stopFlywheels() {
    targetFlywheelSpeed = RadiansPerSecond.of(0);
    io.stopFlywheel();
  }

  /**
   * Sets the target flywheel speed. The motor controller's PID will automatically adjust voltage to
   * reach and maintain this speed.
   *
   * @param speed The desired flywheel angular velocity
   */
  public void setFlywheelSpeed(AngularVelocity speed) {
    targetFlywheelSpeed = speed;
    io.setFlywheelSpeed(speed);
  }

  /**
   * Calculates the required flywheel speed for a given distance using an interpolating lookup
   * table. Clamped to configured min/max speeds.
   *
   * @param distance Distance to the target
   * @return Required flywheel speed
   */
  public AngularVelocity getSpeedForDistance(Distance distance) {
    double distanceMeters = distance.in(Meters);
    double speedRPM = ShooterConstants.DistanceMap.SPEED_MAP.get(distanceMeters);

    double minRPM = ShooterConstants.Limits.MIN_SPEED.in(RevolutionsPerSecond) * 60.0;
    double maxRPM = ShooterConstants.Limits.MAX_SPEED.in(RevolutionsPerSecond) * 60.0;
    speedRPM = Math.max(minRPM, Math.min(maxRPM, speedRPM));

    SmartDashboard.putNumber("Shooter/ShotCalculator/Distance_m", distanceMeters);
    SmartDashboard.putNumber("Shooter/ShotCalculator/CalculatedSpeed_RPM", speedRPM);

    return RPM.of(speedRPM);
  }

  /**
   * Checks if the flywheel has reached its target speed within percentage tolerance.
   *
   * <p>Tolerance is defined by {@link ShooterConstants.Software#PID_TOLERANCE}. For example, with
   * 5% tolerance and 3000 RPM target, returns true between 2850-3150 RPM.
   */
  public boolean areFlywheelsAtTargetSpeed() {
    if (inputs.flywheelVelocity == null) {
      return false;
    }
    double targetRadPerSec = targetFlywheelSpeed.in(RadiansPerSecond);
    double currentRadPerSec = inputs.flywheelVelocity.in(RadiansPerSecond);
    return Math.abs(targetRadPerSec - currentRadPerSec)
        <= Math.abs(targetRadPerSec * ShooterConstants.Software.PID_TOLERANCE);
  }

  /** Manual duty cycle control - for testing motor direction/wiring only. */
  public void setFlywheelDutyCycle(double output) {
    io.setFlywheelDutyCycle(output);
  }

  /** Run characterization for SysId feedforward identification. */
  public void runCharacterization(Voltage volts) {
    io.setFlywheelVoltage(volts);
  }

  /** Returns current flywheel velocity in rad/s for SysId. */
  public double getFFCharacterizationVelocity() {
    if (inputs.flywheelVelocity == null) {
      return 0.0;
    }
    return inputs.flywheelVelocity.in(RadiansPerSecond);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(Volts.of(0.0)))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(Volts.of(0.0)))
        .withTimeout(1.0)
        .andThen(sysId.dynamic(direction));
  }
}
