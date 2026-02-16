package frc.robot.subsystems.flywheel;

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
import frc.robot.subsystems.flywheel.io.FlywheelIO;

/**
 * The Shooter subsystem controls the robot's game piece launching mechanism.
 *
 * <p><b>Hardware:</b> 4x TalonFX (Phoenix 6) motors driving a flywheel (1 leader + 3 followers).
  *
 */
public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIO.ShooterIOInputs inputs = new FlywheelIO.ShooterIOInputs();
  private final SysIdRoutine sysId;

  /**
   * Creates a new Shooter subsystem.
   *
   * @param shooterIO The hardware interface for shooter control
   */
  public Flywheel(FlywheelIO shooterIO) {
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
            new SysIdRoutine.Mechanism((voltage) -> setFlywheelVoltage(voltage), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  /** Stops the flywheel by setting target speed to zero. */
  public void stopFlywheels() {
    io.stopFlywheel();
  }

  /** Manual duty cycle control - for testing motor direction/wiring only. */
  public void setFlywheelDutyCycle(double output) {
    io.setFlywheelDutyCycle(output);
  }

  /** Run characterization for SysId feedforward identification. */
  public void setFlywheelVoltage(Voltage volts) {
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
    return run(() -> setFlywheelVoltage(Volts.of(0.0)))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> setFlywheelVoltage(Volts.of(0.0)))
        .withTimeout(1.0)
        .andThen(sysId.dynamic(direction));
  }
}
