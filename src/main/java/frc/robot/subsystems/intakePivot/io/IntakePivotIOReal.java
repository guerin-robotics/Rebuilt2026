package frc.robot.subsystems.intakePivot.io;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.intakePivot.IntakePivotConstants;
import frc.robot.subsystems.intakePivot.IntakePivotConstants.PivotMagicConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Real hardware implementation of {@link IntakePivotIO}.
 *
 * <p>Controls the intake pivot using a TalonFX motor and a CANcoder for absolute position feedback.
 * Uses MotionMagic for smooth position and velocity control.
 */
public class IntakePivotIOReal implements IntakePivotIO {

  private static final CANBus CAN_BUS = new CANBus("rio");

  private final TalonFX intakePivotMotor;
  private final CANcoder intakePivotEncoder;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVelocityTorqueCurrentFOC velocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);
  private final MotionMagicTorqueCurrentFOC positionRequest = new MotionMagicTorqueCurrentFOC(0);

  // Cached status signals for motor
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> deviceTemp;
  private final StatusSignal<Double> closedLoopReference;
  private final StatusSignal<Double> closedLoopError;

  // Cached status signal for encoder
  private final StatusSignal<Angle> encoderPosition;

  public IntakePivotIOReal() {
    intakePivotMotor = new TalonFX(HardwareConstants.CanIds.INTAKE_PIVOT_MOTOR_ID, CAN_BUS);
    intakePivotEncoder = new CANcoder(HardwareConstants.CanIds.INTAKE_PIVOT_ENCODER_ID, CAN_BUS);

    configurePivotMotor();
    configureEncoder();

    // Cache signal references once in the constructor — motor signals
    velocity = intakePivotMotor.getVelocity();
    motorVoltage = intakePivotMotor.getMotorVoltage();
    statorCurrent = intakePivotMotor.getStatorCurrent();
    supplyCurrent = intakePivotMotor.getSupplyCurrent();
    deviceTemp = intakePivotMotor.getDeviceTemp();
    closedLoopReference = intakePivotMotor.getClosedLoopReference();
    closedLoopError = intakePivotMotor.getClosedLoopError();

    // Cache signal reference — encoder
    encoderPosition = intakePivotEncoder.getAbsolutePosition();

    // Set update frequency for all signals (50Hz is plenty for non-odometry)
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        velocity,
        motorVoltage,
        statorCurrent,
        supplyCurrent,
        deviceTemp,
        closedLoopReference,
        closedLoopError,
        encoderPosition);

    // Stop sending signals we didn't register — reduces CAN bus traffic
    intakePivotMotor.optimizeBusUtilization();
    intakePivotEncoder.optimizeBusUtilization();
  }

  private void configurePivotMotor() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.MotorOutput.Inverted =
        IntakePivotConstants.SoftwareConstants.MOTOR_INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    config.Feedback.RotorToSensorRatio = IntakePivotConstants.Mechanical.pivotRatio;
    config.Feedback.SensorToMechanismRatio = 1;

    // Slot0 gains for position control — Arm_Cosine gravity compensation for a pivot
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.kS = IntakePivotConstants.PID.KG;
    config.Slot0.kP = IntakePivotConstants.PID.KP;

    var pivotMotionMagic = config.MotionMagic;
    pivotMotionMagic.MotionMagicAcceleration = PivotMagicConstants.pivotAccel;
    pivotMotionMagic.MotionMagicCruiseVelocity = PivotMagicConstants.pivotVelo;

    // Current limits
    var limits = new CurrentLimitsConfigs();
    limits.SupplyCurrentLimit = IntakePivotConstants.CurrentLimits.INTAKE_PIVOT_MAIN_SUPPLY_AMP;
    limits.SupplyCurrentLimitEnable = false;
    limits.SupplyCurrentLowerLimit =
        IntakePivotConstants.CurrentLimits.INTAKE_PIVOT_MAIN_SUPPLY_TRIGGER_AMP;
    limits.SupplyCurrentLowerTime =
        IntakePivotConstants.CurrentLimits.INTAKE_PIVOT_MAIN_SUPPLY_TRIGGER_TIME_SEC.in(Second);
    limits.StatorCurrentLimit = IntakePivotConstants.CurrentLimits.INTAKE_PIVOT_MAIN_STATOR_AMP;
    limits.StatorCurrentLimitEnable = false;

    // CANcoder remote feedback
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

    intakePivotMotor.getConfigurator().apply(config);
    intakePivotMotor.getConfigurator().apply(limits);
    intakePivotMotor.getConfigurator().apply(feedback);
  }

  public void configureEncoder() {
    var encoderConfig = new CANcoderConfiguration();

    var magnetConfig = new MagnetSensorConfigs();

    magnetConfig.withAbsoluteSensorDiscontinuityPoint(
        IntakePivotConstants.Mechanical.magnetSensorDiscontinuityPoint);
    magnetConfig.withMagnetOffset(IntakePivotConstants.Mechanical.magnetOffset);
    magnetConfig.SensorDirection = IntakePivotConstants.SoftwareConstants.ENCODER_DIRECTION;

    encoderConfig.withMagnetSensor(magnetConfig);

    intakePivotEncoder.getConfigurator().apply(encoderConfig);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    // One batched CAN read for all signals — much faster than individual reads
    BaseStatusSignal.refreshAll(
        velocity,
        motorVoltage,
        statorCurrent,
        supplyCurrent,
        deviceTemp,
        closedLoopReference,
        closedLoopError,
        encoderPosition);

    // Read from cache — no additional CAN traffic
    inputs.intakePivotVelocity = velocity.getValue();
    inputs.intakePivotPosition = encoderPosition.getValue();
    inputs.intakePivotVoltage = motorVoltage.getValue();
    inputs.intakePivotStatorCurrent = statorCurrent.getValue();
    inputs.intakePivotSupplyCurrent = supplyCurrent.getValue();
    inputs.intakePivotTemperature = deviceTemp.getValue();
    inputs.intakePivotClosedLoopReference =
        RotationsPerSecond.of(closedLoopReference.getValueAsDouble());
    inputs.intakePivotClosedLoopError = RotationsPerSecond.of(closedLoopError.getValueAsDouble());
  }

  @Override
  public void setPivotVoltage(Voltage volts) {
    intakePivotMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPivotVelocity(AngularVelocity velocity) {
    intakePivotMotor.setControl(velocityRequest.withVelocity(velocity));
    Logger.recordOutput("Intake pivot torque controls", velocity);
  }

  @Override
  public void setPivotPosition(double positionRotations) {
    intakePivotMotor.setControl(positionRequest.withPosition(positionRotations));
  }

  @Override
  public void zeroPivotEncoder() {
    intakePivotEncoder.setPosition(0);
  }
}
