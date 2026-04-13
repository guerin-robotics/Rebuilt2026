package frc.robot.subsystems.intakePivot.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.intakePivot.IntakePivotConstants;

/**
 * Simulated implementation of {@link IntakePivotIO} using a WPILib {@link DCMotorSim} and a
 * software PID controller.
 *
 * <p>This approach mirrors how the swerve module turning motor is simulated in {@code ModuleIOSim}:
 * a simple physics model driven by voltage, with a PID loop running in software. This avoids the
 * complexity (and sporadic position updates) of the CTRE TalonFX/CANcoder sim state pipeline.
 *
 * <p>Control modes:
 *
 * <ul>
 *   <li><b>Open-loop voltage:</b> Applied directly to the DCMotorSim.
 *   <li><b>Closed-loop velocity:</b> Software PID tracks a velocity setpoint.
 *   <li><b>Closed-loop position:</b> Software PID tracks a position setpoint.
 * </ul>
 */
public class IntakePivotIOSim implements IntakePivotIO {

  // ---------- Sim PID gains (tuned for the DCMotorSim model) ----------
  // Modeled after the swerve turning motor gains in ModuleIOSim
  private static final double POSITION_KP = 8.0;
  private static final double POSITION_KD = 0.0;
  private static final double VELOCITY_KP = 0.05;
  private static final double VELOCITY_KD = 0.0;

  // ---------- Motor & physics ----------
  private static final DCMotor GEARBOX = DCMotor.getKrakenX44Foc(1);

  private final DCMotorSim pivotSim;

  // ---------- Control state ----------
  private enum ControlMode {
    OPEN_LOOP,
    POSITION,
    VELOCITY
  }

  private ControlMode controlMode = ControlMode.OPEN_LOOP;
  private final PIDController positionController = new PIDController(POSITION_KP, 0, POSITION_KD);
  private final PIDController velocityController = new PIDController(VELOCITY_KP, 0, VELOCITY_KD);
  private double appliedVolts = 0.0;

  public IntakePivotIOSim() {
    pivotSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                GEARBOX,
                IntakePivotConstants.Sim.PIVOT_MOI,
                IntakePivotConstants.Mechanical.pivotRatio),
            GEARBOX);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    // Run the appropriate control loop
    if (controlMode == ControlMode.POSITION) {
      appliedVolts = positionController.calculate(pivotSim.getAngularPositionRad());
    } else if (controlMode == ControlMode.VELOCITY) {
      appliedVolts = velocityController.calculate(pivotSim.getAngularVelocityRadPerSec());
    }
    // OPEN_LOOP: appliedVolts is set directly by setPivotVoltage

    // Clamp and step the physics sim
    pivotSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    pivotSim.update(0.02);

    // Read out state
    double positionRotations = Units.radiansToRotations(pivotSim.getAngularPositionRad());
    double velocityRPS = Units.radiansToRotations(pivotSim.getAngularVelocityRadPerSec());

    inputs.intakePivotPosition = Rotations.of(positionRotations);
    inputs.intakePivotVelocity = RotationsPerSecond.of(velocityRPS);
    inputs.intakePivotVoltage = Volts.of(appliedVolts);
    inputs.intakePivotStatorCurrent = Amps.of(Math.abs(pivotSim.getCurrentDrawAmps()));
    inputs.intakePivotSupplyCurrent = Amps.of(Math.abs(pivotSim.getCurrentDrawAmps()));
    inputs.intakePivotTemperature = Celsius.of(25.0); // sim doesn't model temperature

    // Closed-loop reference and error in mechanism rotations
    if (controlMode == ControlMode.POSITION) {
      double refRotations = Units.radiansToRotations(positionController.getSetpoint());
      inputs.intakePivotClosedLoopReference = refRotations;
      inputs.intakePivotClosedLoopError = refRotations - positionRotations;
    } else if (controlMode == ControlMode.VELOCITY) {
      double refRPS = Units.radiansToRotations(velocityController.getSetpoint());
      inputs.intakePivotClosedLoopReference = refRPS;
      inputs.intakePivotClosedLoopError = refRPS - velocityRPS;
    } else {
      inputs.intakePivotClosedLoopReference = 0.0;
      inputs.intakePivotClosedLoopError = 0.0;
    }
  }

  @Override
  public void setPivotVoltage(Voltage volts) {
    controlMode = ControlMode.OPEN_LOOP;
    appliedVolts = volts.in(Volts);
  }

  @Override
  public void setPivotVelocity(AngularVelocity velocity) {
    controlMode = ControlMode.VELOCITY;
    velocityController.setSetpoint(Units.rotationsToRadians(velocity.in(RotationsPerSecond)));
  }

  @Override
  public void setPivotPosition(Angle position) {
    controlMode = ControlMode.POSITION;
    positionController.setSetpoint(Units.rotationsToRadians(position.in(Rotations)));
  }

  @Override
  public void zeroPivotEncoder() {
    // In sim, reset the physics model to 0
    pivotSim.setAngle(0.0);
    positionController.reset();
  }
}
