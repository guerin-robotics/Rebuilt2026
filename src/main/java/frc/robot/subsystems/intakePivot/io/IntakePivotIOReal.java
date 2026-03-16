package frc.robot.subsystems.intakePivot.io;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.CANBus;
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
import edu.wpi.first.units.measure.AngularVelocity;
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

  public IntakePivotIOReal() {
    intakePivotMotor = new TalonFX(HardwareConstants.CanIds.INTAKE_PIVOT_MOTOR_ID, CAN_BUS);
    intakePivotEncoder = new CANcoder(HardwareConstants.CanIds.INTAKE_PIVOT_ENCODER_ID, CAN_BUS);

    configurePivotMotor();
    configureEncoder();
  }

  private void configurePivotMotor() {
    // Set current limits, neutral mode, etc. as needed
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
    config.Slot0.kG = IntakePivotConstants.PID.KG;
    // config.Slot0.kV = IntakePivotConstants.PID.KV;
    config.Slot0.kP = IntakePivotConstants.PID.KP;
    // config.Slot0.kI = IntakePivotConstants.PID.KI;
    config.Slot0.kD = IntakePivotConstants.PID.KD;

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
    // Get velocity and position from encoder
    inputs.intakePivotVelocity =
        RotationsPerSecond.of(intakePivotMotor.getVelocity().getValueAsDouble());
    inputs.intakePivotPosition = intakePivotEncoder.getAbsolutePosition().getValueAsDouble();

    // Get voltage, current, temperature, and setpoint/error from motor
    inputs.intakePivotVoltage = intakePivotMotor.getMotorVoltage().getValue();
    inputs.intakePivotStatorCurrent = intakePivotMotor.getStatorCurrent().getValueAsDouble();
    inputs.intakePivotSupplyCurrent = intakePivotMotor.getSupplyCurrent().getValue();
    inputs.intakePivotTemperature = intakePivotMotor.getDeviceTemp().getValue();
    inputs.intakePivotClosedLoopReference =
        intakePivotMotor.getClosedLoopReference().getValueAsDouble();
    inputs.intakePivotClosedLoopError = intakePivotMotor.getClosedLoopError().getValueAsDouble();
  }

  public void setPivotVoltage(Voltage volts) {
    intakePivotMotor.setControl(voltageRequest.withOutput(volts));
  }

  public void setPivotVelocity(AngularVelocity velocity) {
    intakePivotMotor.setControl(velocityRequest.withVelocity(velocity));
    Logger.recordOutput("Intake pivot torque controls", velocity);
  }

  public void setPivotPosition(double positionRotations) {
    intakePivotMotor.setControl(positionRequest.withPosition(positionRotations));
  }

  public void zeroPivotEncoder() {
    intakePivotEncoder.setPosition(0);
  }
}
