package frc.robot.subsystems.intakePivot.io;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.intakePivot.IntakePivotConstants;
import frc.robot.subsystems.intakePivot.IntakePivotConstants.PivotMagicConstants;

/**
 * Simulated implementation of {@link IntakePivotIO} using CTRE's TalonFXSimState and
 * CANcoderSimState.
 *
 * <p>Creates real TalonFX and CANcoder objects so the internal closed-loop controllers run in
 * simulation. A {@link SingleJointedArmSim} models the pivot's physics (gravity, inertia, etc.),
 * and the resulting position/velocity are fed back into both the TalonFX and CANcoder sim states.
 *
 * <p>This sim IO mirrors the configuration of {@link IntakePivotIOReal} as closely as possible,
 * including MotionMagic position control, CANcoder remote feedback, and current limits.
 */
public class IntakePivotIOSim implements IntakePivotIO {

  // Real TalonFX and CANcoder objects — their internal firmware runs in sim
  private final TalonFX pivotMotor;
  private final CANcoder pivotEncoder;

  // Sim state objects for injecting physics
  private final TalonFXSimState motorSimState;
  private final CANcoderSimState encoderSimState;

  // WPILib physics model for the pivot arm
  private final SingleJointedArmSim pivotPhysicsSim;

  // Control requests — use voltage-based control in sim because TorqueCurrentFOC
  // does not produce motor voltage output in the TalonFX sim state, which means
  // the physics sim never receives any input and the position never moves.
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0);
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

  public IntakePivotIOSim() {
    pivotMotor = new TalonFX(HardwareConstants.CanIds.INTAKE_PIVOT_MOTOR_ID);
    pivotEncoder = new CANcoder(HardwareConstants.CanIds.INTAKE_PIVOT_ENCODER_ID);

    configurePivotMotor();
    configureEncoder();

    motorSimState = pivotMotor.getSimState();
    encoderSimState = pivotEncoder.getSimState();

    // Per the CTRE docs: SensorOffset is subtracted from the raw position before
    // the magnet offset config is added. Setting it to the magnet offset cancels
    // out the offset, so setRawPosition(mechanismPos) → reported = mechanismPos.
    encoderSimState.SensorOffset = IntakePivotConstants.Mechanical.magnetOffset;

    // SingleJointedArmSim models a pivot joint with gravity
    pivotPhysicsSim =
        new SingleJointedArmSim(
            IntakePivotConstants.Sim.PIVOT_MOTOR,
            IntakePivotConstants.Mechanical.pivotRatio,
            IntakePivotConstants.Sim.PIVOT_MOI,
            IntakePivotConstants.Sim.ARM_LENGTH_METERS,
            IntakePivotConstants.Sim.MIN_ANGLE_RAD,
            IntakePivotConstants.Sim.MAX_ANGLE_RAD,
            IntakePivotConstants.Sim.SIMULATE_GRAVITY,
            IntakePivotConstants.Sim.STARTING_ANGLE_RAD);
  }

  /**
   * Configures the TalonFX with PID gains, MotionMagic, current limits, and CANcoder feedback for
   * simulation. Mirrors the real robot configuration as closely as possible.
   */
  private void configurePivotMotor() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.MotorOutput.Inverted =
        IntakePivotConstants.SoftwareConstants.MOTOR_INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    // In sim we use the internal rotor sensor — we inject rotor position directly
    // via setRawRotorPosition(mechanismPos * gearRatio), so no RemoteCANcoder needed.
    config.Feedback.RotorToSensorRatio = IntakePivotConstants.Mechanical.pivotRatio;
    config.Feedback.SensorToMechanismRatio = 1;

    // Slot0 gains — Arm_Cosine gravity compensation for a pivot
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.kS = IntakePivotConstants.Sim.KS;
    config.Slot0.kV = IntakePivotConstants.Sim.KV;
    config.Slot0.kP = IntakePivotConstants.Sim.KP;
    config.Slot0.kI = IntakePivotConstants.Sim.KI;
    config.Slot0.kD = IntakePivotConstants.Sim.KD;

    // MotionMagic profile constraints
    var pivotMotionMagic = config.MotionMagic;
    pivotMotionMagic.MotionMagicAcceleration = PivotMagicConstants.pivotAccel;
    pivotMotionMagic.MotionMagicCruiseVelocity = PivotMagicConstants.pivotVelo;

    // Current limits (match real robot)
    var limits = new CurrentLimitsConfigs();
    limits.SupplyCurrentLimit = IntakePivotConstants.CurrentLimits.INTAKE_PIVOT_MAIN_SUPPLY_AMP;
    limits.SupplyCurrentLimitEnable = false;
    limits.StatorCurrentLimit = IntakePivotConstants.CurrentLimits.INTAKE_PIVOT_MAIN_STATOR_AMP;
    limits.StatorCurrentLimitEnable = false;

    // Disable software limits in simulation.
    // The soft limits exist to protect real hardware from over-travel.
    // In sim, gravity can push the arm past the lower limit at startup before
    // the controller has a chance to react, which immediately clamps motor
    // output to 0 V and freezes the simulation permanently.
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    pivotMotor.getConfigurator().apply(config);
    pivotMotor.getConfigurator().apply(limits);
  }

  /** Configures the CANcoder for simulation. */
  private void configureEncoder() {
    var encoderConfig = new CANcoderConfiguration();

    var magnetConfig = new MagnetSensorConfigs();
    magnetConfig.withAbsoluteSensorDiscontinuityPoint(
        IntakePivotConstants.Mechanical.magnetSensorDiscontinuityPoint);
    // Keep the real magnet offset so the CANcoder reports positions in the same
    // range as on real hardware. CANcoderSimState.SensorOffset compensates for this
    // automatically — set it equal to the magnet offset so that when we call
    // setRawPosition(mechanismPos), the reported position equals mechanismPos.
    magnetConfig.withMagnetOffset(IntakePivotConstants.Mechanical.magnetOffset);
    magnetConfig.SensorDirection = IntakePivotConstants.SoftwareConstants.ENCODER_DIRECTION;

    encoderConfig.withMagnetSensor(magnetConfig);
    pivotEncoder.getConfigurator().apply(encoderConfig);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    // 1. Tell the sim what the battery voltage is
    motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    encoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // 2. Get the motor voltage output from the TalonFX's internal controller
    double motorVolts = motorSimState.getMotorVoltageMeasure().in(Volts);

    // 3. Feed the voltage into the pivot arm physics sim and step it forward
    pivotPhysicsSim.setInputVoltage(motorVolts);
    pivotPhysicsSim.update(0.02);

    // 4. Convert the arm sim's position (radians) to rotations for the TalonFX and CANcoder
    double mechanismPositionRotations = Units.radiansToRotations(pivotPhysicsSim.getAngleRads());
    double mechanismVelocityRPS = Units.radiansToRotations(pivotPhysicsSim.getVelocityRadPerSec());

    // 5. Write the resulting position and velocity back to the TalonFX sim state
    //    The TalonFX expects rotor position, so multiply by the gear ratio
    double gearRatio = IntakePivotConstants.Mechanical.pivotRatio;
    motorSimState.setRawRotorPosition(mechanismPositionRotations * gearRatio);
    motorSimState.setRotorVelocity(mechanismVelocityRPS * gearRatio);

    // 6. Update the CANcoder sim state with the mechanism position/velocity
    //    The CANcoder reads mechanism position directly (no gear ratio needed)
    encoderSimState.setRawPosition(mechanismPositionRotations);
    encoderSimState.setVelocity(mechanismVelocityRPS);

    // 7. Simulate battery voltage sag
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(pivotPhysicsSim.getCurrentDrawAmps()));

    // 8. Read values from the hardware objects (just like on real hardware)
    inputs.intakePivotVelocity = RotationsPerSecond.of(pivotMotor.getVelocity().getValueAsDouble());
    inputs.intakePivotPosition = pivotEncoder.getAbsolutePosition().getValueAsDouble();

    // Voltage, current, temperature from the motor
    inputs.intakePivotVoltage = pivotMotor.getMotorVoltage().getValue();
    inputs.intakePivotStatorCurrent = pivotMotor.getStatorCurrent().getValue();
    inputs.intakePivotSupplyCurrent = pivotMotor.getSupplyCurrent().getValue();
    inputs.intakePivotTemperature = pivotMotor.getDeviceTemp().getValue();
    inputs.intakePivotClosedLoopReference = pivotMotor.getClosedLoopReference().getValueAsDouble();
    inputs.intakePivotClosedLoopError = pivotMotor.getClosedLoopError().getValueAsDouble();
  }

  @Override
  public void setPivotVoltage(Voltage volts) {
    pivotMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPivotVelocity(AngularVelocity velocity) {
    pivotMotor.setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public void setPivotPosition(double positionRotations) {
    pivotMotor.setControl(positionRequest.withPosition(positionRotations));
  }

  @Override
  public void zeroPivotEncoder() {
    pivotEncoder.setPosition(0);
  }
}
