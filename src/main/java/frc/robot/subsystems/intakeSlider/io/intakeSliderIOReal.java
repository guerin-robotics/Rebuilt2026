package frc.robot.subsystems.intakeSlider.io;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.intakeSlider.intakeSliderConstants;
import frc.robot.subsystems.intakeSlider.intakeSliderConstants.sliderMagicConstants;
import org.littletonrobotics.junction.Logger;

public class intakeSliderIOReal implements intakeSliderIO {

  private static final CANBus CAN_BUS = new CANBus("rio");

  private final TalonFX intakeSliderMotor;
  private final CANcoder intakeSliderEncoder;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVelocityTorqueCurrentFOC velocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrentFOC(0);

  public intakeSliderIOReal() {
    intakeSliderMotor = new TalonFX(HardwareConstants.CanIds.INTAKE_SLIDER_MOTOR_ID, CAN_BUS);
    intakeSliderEncoder = new CANcoder(HardwareConstants.CanIds.INTAKE_SLIDER_ENCODER_ID, CAN_BUS);

    configureSliderMotor();
    configureEncoder();
  }

  private void configureSliderMotor() {
    // Set current limits, neutral mode, etc. as needed
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.MotorOutput.Inverted =
        intakeSliderConstants.SoftwareConstants.INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    config.Feedback.RotorToSensorRatio = intakeSliderConstants.Mechanical.sliderRatio;
    config.Feedback.SensorToMechanismRatio = 1;

    // Slot0 gains for position control
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.kS = intakeSliderConstants.PID.KG;
    // config.Slot0.kV = intakeSliderConstants.PID.KV;
    config.Slot0.kP = intakeSliderConstants.PID.KP;
    // config.Slot0.kI = intakeSliderConstants.PID.KI;
    config.Slot0.kD = intakeSliderConstants.PID.KD;

    var sliderMotionMagic = config.MotionMagic;
    sliderMotionMagic.MotionMagicAcceleration = sliderMagicConstants.sliderAccel;

    // Current limits
    var limits = new CurrentLimitsConfigs();
    limits.SupplyCurrentLimit = intakeSliderConstants.CurrentLimits.INTAKE_SLIDER_MAIN_SUPPLY_AMP;
    limits.SupplyCurrentLimitEnable = false;
    limits.SupplyCurrentLowerLimit =
        intakeSliderConstants.CurrentLimits.INTAKE_SLIDER_MAIN_SUPPLY_TRIGGER_AMP;
    limits.SupplyCurrentLowerTime =
        intakeSliderConstants.CurrentLimits.INTAKE_SLIDER_MAIN_SUPPLY_TRIGGER_TIME_SEC.in(Second);
    limits.StatorCurrentLimit = intakeSliderConstants.CurrentLimits.INTAKE_SLIDER_MAIN_STATOR_AMP;
    limits.StatorCurrentLimitEnable = false;

    // Cancoder
    var feedback = new FeedbackConfigs();
    feedback.withFusedCANcoder(intakeSliderEncoder);
    feedback.withFeedbackRemoteSensorID(42);
    // feedback.withAbsoluteSensorDiscontinuityPoint(0.0);
    // feedback.withAbsoluteSensorOffset(0.0);

    intakeSliderMotor.getConfigurator().apply(config);
    intakeSliderMotor.getConfigurator().apply(limits);
    intakeSliderMotor.getConfigurator().apply(feedback);
  }

  public void configureEncoder() {
    var encoderConfig = new CANcoderConfiguration();

    var magnetConfig = new MagnetSensorConfigs();

    magnetConfig.withAbsoluteSensorDiscontinuityPoint(0.625);
    magnetConfig.withMagnetOffset(intakeSliderConstants.Mechanical.magnetOffset);

    encoderConfig.withMagnetSensor(magnetConfig);

    intakeSliderEncoder.getConfigurator().apply(encoderConfig);
  }

  @Override
  public void updateInputs(IntakeSliderIOInputs inputs) {
    // Get velocity and position from encoder
    inputs.intakeSliderVelocity =
        RotationsPerSecond.of(intakeSliderMotor.getVelocity().getValueAsDouble());
    inputs.intakeSliderPosition = intakeSliderEncoder.getAbsolutePosition().getValueAsDouble();

    // Get voltage, current, temperature, and setpoint/error from motor
    inputs.intakeSliderVoltage = intakeSliderMotor.getMotorVoltage().getValue();
    inputs.intakeSliderStatorCurrent = intakeSliderMotor.getStatorCurrent().getValueAsDouble();
    inputs.intakeSliderSupplyCurrent = intakeSliderMotor.getSupplyCurrent().getValue();
    inputs.intakeSliderTemperature = intakeSliderMotor.getDeviceTemp().getValue();
    inputs.intakeSliderClosedLoopReference =
        RotationsPerSecond.of(intakeSliderMotor.getClosedLoopReference().getValueAsDouble());
    inputs.intakeSliderClosedLoopError =
        RotationsPerSecond.of(intakeSliderMotor.getClosedLoopError().getValueAsDouble());
  }

  public void setSliderVoltage(Voltage volts) {
    intakeSliderMotor.setControl(voltageRequest.withOutput(volts));
  }

  public void setSliderVelocity(AngularVelocity velocity) {
    intakeSliderMotor.setControl(velocityRequest.withVelocity(velocity));
    Logger.recordOutput("Intake slider torque controls", velocity);
  }

  public void setSliderPosition(double positionRotations) {
    intakeSliderMotor.setControl(positionRequest.withPosition(positionRotations));
  }

  public void zeroSliderEncoder() {
    intakeSliderEncoder.setPosition(0);
  }
}
