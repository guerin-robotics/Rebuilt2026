package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.simulation.SimulationConstants;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 * Module IO backed by a MapleSim {@link SwerveModuleSimulation}.
 *
 * <p>Drop-in replacement for {@link ModuleIOSim}. The difference is where the wheel speed comes
 * from: {@code ModuleIOSim} spins an isolated {@link edu.wpi.first.wpilibj.simulation.DCMotorSim}
 * and the chassis motion is inferred from kinematics, so the robot can accelerate at physically
 * impossible rates and drives straight through walls. Here the module reports the speed the physics
 * engine actually produced after applying mass, traction limits, and any collision — so wheel slip
 * and impacts show up in odometry exactly as they would on the field.
 *
 * <p>Closed-loop control still runs on this side (WPILib {@link PIDController} + feedforward),
 * matching {@code ModuleIOSim}'s structure and gains. MapleSim only supplies the plant.
 */
public class ModuleIOMapleSim implements ModuleIO {

  // Gains mirror ModuleIOSim so switching between the two sim backends doesn't also
  // change the control tuning — only the physics model changes.
  private static final double DRIVE_KP = 0.05;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_KS = 0.0;
  private static final double DRIVE_KV_ROT = 0.91035; // (volt * secs) / rotation
  private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
  private static final double TURN_KP = 8.0;
  private static final double TURN_KD = 0.0;

  private final SwerveModuleSimulation moduleSimulation;
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController turnMotor;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private final PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
  private final PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOMapleSim(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;

    // The current limit is what makes wheel slip physical: MapleSim caps propelling force by
    // the lesser of available torque and the traction the COF allows.
    this.driveMotor =
        moduleSimulation
            .useGenericMotorControllerForDrive()
            .withCurrentLimit(SimulationConstants.DRIVE_CURRENT_LIMIT);
    this.turnMotor = moduleSimulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(20));

    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control against the physics engine's measured state
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts
              + driveController.calculate(
                  moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond));
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts =
          turnController.calculate(moduleSimulation.getSteerAbsoluteFacing().getRadians());
    } else {
      turnController.reset();
    }

    driveMotor.requestVoltage(Volts.of(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0)));
    turnMotor.requestVoltage(Volts.of(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0)));

    // Drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocityRadPerSec = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveAppliedVolts = moduleSimulation.getDriveMotorAppliedVoltage().in(Volts);
    inputs.driveCurrentAmps = Math.abs(moduleSimulation.getDriveMotorStatorCurrent().in(Amps));

    // Turn inputs
    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = moduleSimulation.getSteerAbsoluteFacing();
    inputs.turnPosition = moduleSimulation.getSteerAbsoluteFacing();
    inputs.turnVelocityRadPerSec =
        moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    inputs.turnAppliedVolts = moduleSimulation.getSteerMotorAppliedVoltage().in(Volts);
    inputs.turnCurrentAmps = Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Amps));

    // High-frequency odometry: MapleSim caches one sample per physics sub-tick (5 per 20 ms
    // by default), so sim odometry sees the same multi-sample cadence as the real CANivore bus.
    inputs.odometryTimestamps = getSimulationOdometryTimestamps();
    inputs.odometryDrivePositionsRad =
        java.util.Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();
    inputs.odometryTurnPositions = moduleSimulation.getCachedSteerAbsolutePositions();
  }

  /**
   * Reconstructs a timestamp for each physics sub-tick in the loop that just completed. MapleSim
   * caches sensor samples per sub-tick but not their timestamps, so they are evenly spaced across
   * the robot period ending now.
   */
  private static double[] getSimulationOdometryTimestamps() {
    int subTicks = org.ironmaple.simulation.SimulatedArena.getSimulationSubTicksIn1Period();
    double[] timestamps = new double[subTicks];
    double periodSeconds = 0.02;
    double now = Timer.getFPGATimestamp();
    for (int i = 0; i < subTicks; i++) {
      // Oldest sample first, most recent last.
      timestamps[i] = now - periodSeconds + periodSeconds * (i + 1) / subTicks;
    }
    return timestamps;
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
