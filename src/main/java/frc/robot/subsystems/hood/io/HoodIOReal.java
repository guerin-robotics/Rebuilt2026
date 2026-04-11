package frc.robot.subsystems.hood.io;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
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
import frc.robot.subsystems.hood.HoodConstants;

public class HoodIOReal implements HoodIO {

  // private final PWM hoodServo;
  // private final PWM hoodLeftServo;

  private final TalonFX hoodMotor;
  private final CANcoder hoodEncoder;

  private final MotionMagicTorqueCurrentFOC positionRequest = new MotionMagicTorqueCurrentFOC(0);

  // Cached status signals for motor
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> deviceTemp;
  private final StatusSignal<Angle> devicePos;
  private final StatusSignal<Double> closedLoopReference;
  private final StatusSignal<Double> closedLoopError;

  public HoodIOReal() {
    hoodMotor = new TalonFX(HardwareConstants.CanIds.HOOD_MOTOR);
    hoodEncoder = new CANcoder(HardwareConstants.CanIds.HOOD_ENCODER);

    // Configure encoder first so the CANcoder is ready before the motor tries to use it
    configureEncoder();
    configureMotor();

    // Cache signal references once — motor signals
    velocity = hoodMotor.getVelocity();
    motorVoltage = hoodMotor.getMotorVoltage();
    statorCurrent = hoodMotor.getStatorCurrent();
    supplyCurrent = hoodMotor.getSupplyCurrent();
    deviceTemp = hoodMotor.getDeviceTemp();
    devicePos = hoodMotor.getPosition();
    closedLoopReference = hoodMotor.getClosedLoopReference();
    closedLoopError = hoodMotor.getClosedLoopError();

    // 50Hz for signals we need every loop (velocity, voltage, current, position, closed-loop
    // reference)
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity, motorVoltage, statorCurrent, supplyCurrent, devicePos, closedLoopReference);

    // 10Hz for diagnostic-only signals (temperature, closed-loop error)
    BaseStatusSignal.setUpdateFrequencyForAll(10.0, deviceTemp, closedLoopError);

    // Stop sending signals we didn't register — reduces CAN bus traffic
    hoodMotor.optimizeBusUtilization();
    hoodEncoder.optimizeBusUtilization();
  }

  private void configureMotor() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted =
        HoodConstants.SoftwareConstants.MOTOR_INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

    // Remote CANcoder feedback — set directly on the config object so everything is applied at once
    config.Feedback.FeedbackRemoteSensorID = HardwareConstants.CanIds.HOOD_ENCODER;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.Feedback.RotorToSensorRatio = 1;
    config.Feedback.SensorToMechanismRatio = HoodConstants.Mechanical.hoodRatio;

    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.kG = HoodConstants.PID.KG;
    config.Slot0.kP = HoodConstants.PID.KP;
    config.Slot0.kD = HoodConstants.PID.KD;

    var hoodMotionMagic = config.MotionMagic;
    hoodMotionMagic.MotionMagicAcceleration = HoodConstants.HoodMagicConstants.hoodAccel;
    hoodMotionMagic.MotionMagicCruiseVelocity = HoodConstants.HoodMagicConstants.hoodVelo;

    // Current limits — set directly on the config object
    config.CurrentLimits.SupplyCurrentLimit = HoodConstants.CurrentLimits.HOOD__MAIN_SUPPLY_AMP;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit =
        HoodConstants.CurrentLimits.HOOD_MAIN_SUPPLY_TRIGGER_AMP;
    config.CurrentLimits.SupplyCurrentLowerTime =
        HoodConstants.CurrentLimits.HOOD_MAIN_SUPPLY_TRIGGER_TIME_SEC.in(Seconds);
    config.CurrentLimits.StatorCurrentLimit = HoodConstants.CurrentLimits.HOOD_MAIN_STATOR_AMP;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Software limits
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        HoodConstants.SoftwareConstants.softwareUpperRotationLimit;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        HoodConstants.SoftwareConstants.softwareLowerRotationLimit;

    // Apply everything in a single call — this avoids separate apply() calls overwriting each other
    hoodMotor.getConfigurator().apply(config);
  }

  private void configureEncoder() {
    var encoderConfig = new CANcoderConfiguration();

    var magnetConfig = new MagnetSensorConfigs();

    magnetConfig.withAbsoluteSensorDiscontinuityPoint(
        HoodConstants.Mechanical.magnetSensorDiscontinuityPoint);
    magnetConfig.withMagnetOffset(HoodConstants.Mechanical.magnetOffset);
    magnetConfig.SensorDirection = HoodConstants.SoftwareConstants.ENCODER_DIRECTION;

    encoderConfig.withMagnetSensor(magnetConfig);

    hoodEncoder.getConfigurator().apply(encoderConfig);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // inputs.servoSpeed = hoodServo.getSpeed();
    // inputs.servoPos = hoodServo.getPosition();
    BaseStatusSignal.refreshAll(
        velocity,
        motorVoltage,
        statorCurrent,
        supplyCurrent,
        deviceTemp,
        devicePos,
        closedLoopReference,
        closedLoopError);

    // Read from cache — no additional CAN traffic
    inputs.hoodVelocity = velocity.getValue();
    inputs.hoodPosition = devicePos.getValue();
    inputs.hoodVoltage = motorVoltage.getValue();
    inputs.hoodStatorCurrent = statorCurrent.getValue();
    inputs.hoodSupplyCurrent = supplyCurrent.getValue();
    inputs.hoodTemperature = deviceTemp.getValue();
    inputs.hoodClosedLoopReference = Rotation.of(closedLoopReference.getValueAsDouble());
    inputs.hoodClosedLoopError = Rotation.of(closedLoopError.getValueAsDouble());
  }

  @Override
  public void setHoodPos(Angle position) {
    // hoodServo.setPosition(position);
    // hoodLeftServo.setPosition(position + HoodConstants.Mechanical.leftServoOffset);
    hoodMotor.setControl(positionRequest.withPosition(position));
  }

  public void stopHood() {
    // hoodServo.setSpeed(0);
    // hoodLeftServo.setSpeed(0);
    hoodMotor.setVoltage(0);
  }
}
