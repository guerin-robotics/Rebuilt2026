package frc.robot.subsystems.prestage.io;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.prestage.PrestageConstants;

public class PrestageIOReal implements PrestageIO {

  private static final CANBus CAN_BUS = new CANBus("rio");

  private final TalonFX prestageLeft;
  private final TalonFX prestageRight;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVelocityTorqueCurrentFOC torqueRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  // Cached status signals for LEFT motor
  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<Current> leftStatorCurrent;
  private final StatusSignal<Current> leftSupplyCurrent;
  private final StatusSignal<Voltage> leftMotorVoltage;
  private final StatusSignal<Temperature> leftDeviceTemp;
  private final StatusSignal<Double> leftClosedLoopReference;
  private final StatusSignal<Double> leftClosedLoopError;
  private final StatusSignal<Angle> leftPos;

  // Cached status signals for RIGHT motor
  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<Current> rightStatorCurrent;
  private final StatusSignal<Current> rightSupplyCurrent;
  private final StatusSignal<Voltage> rightMotorVoltage;
  private final StatusSignal<Temperature> rightDeviceTemp;
  private final StatusSignal<Double> rightClosedLoopReference;
  private final StatusSignal<Double> rightClosedLoopError;
  private final StatusSignal<Angle> rightPos;

  public PrestageIOReal() {
    prestageLeft = new TalonFX(HardwareConstants.CanIds.PRESTAGE_LEADER_ID, CAN_BUS);
    prestageRight = new TalonFX(HardwareConstants.CanIds.PRESTAGE_FOLLOWER_ID, CAN_BUS);

    configurePrestageMotor();

    prestageRight.setControl(
        new Follower(HardwareConstants.CanIds.PRESTAGE_LEADER_ID, MotorAlignmentValue.Opposed));

    // Cache signal references once in the constructor — LEFT motor
    leftVelocity = prestageLeft.getVelocity();
    leftStatorCurrent = prestageLeft.getStatorCurrent();
    leftSupplyCurrent = prestageLeft.getSupplyCurrent();
    leftMotorVoltage = prestageLeft.getMotorVoltage();
    leftDeviceTemp = prestageLeft.getDeviceTemp();
    leftClosedLoopReference = prestageLeft.getClosedLoopReference();
    leftClosedLoopError = prestageLeft.getClosedLoopError();
    leftPos = prestageLeft.getPosition();

    // Cache signal references once in the constructor — RIGHT motor
    rightVelocity = prestageRight.getVelocity();
    rightStatorCurrent = prestageRight.getStatorCurrent();
    rightSupplyCurrent = prestageRight.getSupplyCurrent();
    rightMotorVoltage = prestageRight.getMotorVoltage();
    rightDeviceTemp = prestageRight.getDeviceTemp();
    rightClosedLoopReference = prestageRight.getClosedLoopReference();
    rightClosedLoopError = prestageRight.getClosedLoopError();
    rightPos = prestageRight.getPosition();

    // 50Hz for signals we need every loop (velocity, voltage, current, closed-loop reference)
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftVelocity,
        leftStatorCurrent,
        leftSupplyCurrent,
        leftMotorVoltage,
        leftClosedLoopReference,
        rightVelocity,
        rightStatorCurrent,
        rightSupplyCurrent,
        rightMotorVoltage,
        rightClosedLoopReference);

    // 10Hz for diagnostic-only signals (temperature, closed-loop error)
    BaseStatusSignal.setUpdateFrequencyForAll(
        10.0, leftDeviceTemp, leftClosedLoopError, rightDeviceTemp, rightClosedLoopError);

    // Stop sending signals we didn't register — reduces CAN bus traffic
    prestageLeft.optimizeBusUtilization();
    prestageRight.optimizeBusUtilization();
  }

  private void configurePrestageMotor() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted =
        PrestageConstants.SoftwareConstants.INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = PrestageConstants.Mechanical.prestageRatio;

    // Slot0 PID/FF gains for velocity control
    config.Slot0.kS = PrestageConstants.PID.KS;
    config.Slot0.kV = PrestageConstants.PID.KV;
    config.Slot0.kP = PrestageConstants.PID.KP;

    var prestageMagic = config.MotionMagic;
    prestageMagic.MotionMagicAcceleration = PrestageConstants.prestageMagicConstants.prestageAccel;

    // Current limits
    var limits = new CurrentLimitsConfigs();
    limits.SupplyCurrentLimit = PrestageConstants.CurrentLimits.PRESTAGE_MAIN_SUPPLY_AMP;
    limits.SupplyCurrentLimitEnable = true;
    limits.SupplyCurrentLowerLimit =
        PrestageConstants.CurrentLimits.PRESTAGE_MAIN_SUPPLY_TRIGGER_AMP;
    limits.SupplyCurrentLowerTime =
        PrestageConstants.CurrentLimits.PRESTAGE_MAIN_SUPPLY_TRIGGER_TIME_SEC.in(Second);
    limits.StatorCurrentLimit = PrestageConstants.CurrentLimits.PRESTAGE_MAIN_STATOR_AMP;
    limits.StatorCurrentLimitEnable = true;

    prestageLeft.getConfigurator().apply(config);
    prestageLeft.getConfigurator().apply(limits);
    prestageRight.getConfigurator().apply(config);
    prestageRight.getConfigurator().apply(limits);
  }

  @Override
  public void updateInputs(PrestageIOInputs inputs) {
    // One batched CAN read for ALL signals on BOTH motors
    BaseStatusSignal.refreshAll(
        leftVelocity,
        leftStatorCurrent,
        leftSupplyCurrent,
        leftMotorVoltage,
        leftDeviceTemp,
        leftClosedLoopReference,
        leftClosedLoopError,
        leftPos,
        rightVelocity,
        rightStatorCurrent,
        rightSupplyCurrent,
        rightMotorVoltage,
        rightDeviceTemp,
        rightClosedLoopReference,
        rightClosedLoopError,
        rightPos);

    // Left motor — read from cache
    inputs.prestageLeftVelocity = leftVelocity.getValue();
    inputs.prestageLeftStatorAmps = leftStatorCurrent.getValue();
    inputs.prestageLeftSupplyAmps = leftSupplyCurrent.getValue();
    inputs.prestageLeftVoltage = leftMotorVoltage.getValue();
    inputs.prestageLeftTemperature = leftDeviceTemp.getValue();
    inputs.prestageLeftClosedLoopReference =
        RotationsPerSecond.of(leftClosedLoopReference.getValueAsDouble());
    inputs.prestageLeftClosedLoopError =
        RotationsPerSecond.of(leftClosedLoopError.getValueAsDouble());
    inputs.prestageLeftPos = leftPos.getValue();

    // Right motor — read from cache (BUG FIX: was previously reading left motor signals)
    inputs.prestageRightVelocity = rightVelocity.getValue();
    inputs.prestageRightStatorAmps = rightStatorCurrent.getValue();
    inputs.prestageRightSupplyAmps = rightSupplyCurrent.getValue();
    inputs.prestageRightVoltage = rightMotorVoltage.getValue();
    inputs.prestageRightTemperature = rightDeviceTemp.getValue();
    inputs.prestageRightClosedLoopReference =
        RotationsPerSecond.of(rightClosedLoopReference.getValueAsDouble());
    inputs.prestageRightClosedLoopError =
        RotationsPerSecond.of(rightClosedLoopError.getValueAsDouble());
    inputs.prestageRightPos = rightPos.getValue();
  }

  @Override
  public void setPrestageVoltage(Voltage volts) {
    prestageLeft.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPrestageVelocity(AngularVelocity prestageVelo) {
    prestageLeft.setControl(torqueRequest.withVelocity(prestageVelo));
  }
}
