package frc.robot.subsystems.intakePivot.io;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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

  // Control requests (reused to avoid allocations)
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVelocityTorqueCurrentFOC velocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);
  private final MotionMagicTorqueCurrentFOC positionRequest = new MotionMagicTorqueCurrentFOC(0);

  public IntakePivotIOSim() {
    pivotMotor = new TalonFX(HardwareConstants.CanIds.INTAKE_PIVOT_MOTOR_ID);
    pivotEncoder = new CANcoder(HardwareConstants.CanIds.INTAKE_PIVOT_ENCODER_ID);

    configurePivotMotor();
    configureEncoder();

    motorSimState = pivotMotor.getSimState();
    encoderSimState = pivotEncoder.getSimState();

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

    // CANcoder remote feedback (match real robot)
    var feedback = new FeedbackConfigs();
    feedback.withFeedbackRemoteSensorID(HardwareConstants.CanIds.INTAKE_PIVOT_ENCODER_ID);
    feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder);

    // Software limits
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        IntakePivotConstants.SoftwareConstants.softwareUpperRotationLimit;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        IntakePivotConstants.SoftwareConstants.softwareLowerRotationLimit;

    pivotMotor.getConfigurator().apply(config);
    pivotMotor.getConfigurator().apply(limits);
    pivotMotor.getConfigurator().apply(feedback);
  }

  /** Configures the CANcoder with magnet offset and sensor direction (matches real robot). */
  private void configureEncoder() {
    var encoderConfig = new CANcoderConfiguration();

    var magnetConfig = new MagnetSensorConfigs();
    magnetConfig.withAbsoluteSensorDiscontinuityPoint(
        IntakePivotConstants.Mechanical.magnetSensorDiscontinuityPoint);
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
    //    Velocity comes from the motor, position comes from the CANcoder
    inputs.intakePivotVelocity = RotationsPerSecond.of(pivotMotor.getVelocity().getValueAsDouble());
    inputs.intakePivotPosition = pivotEncoder.getAbsolutePosition().getValueAsDouble();

    // Voltage, current, temperature from the motor
    inputs.intakePivotVoltage = pivotMotor.getMotorVoltage().getValue();
    inputs.intakePivotStatorCurrent = pivotMotor.getStatorCurrent().getValueAsDouble();
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
