package frc.robot.subsystems.upperFeeder.io;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.upperFeeder.UpperFeederConstants;

public class UpperFeederIOReal implements UpperFeederIO {

  private static final CANBus CAN_BUS = new CANBus("rio");

  private final TalonFX upperFeederMotor;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVelocityTorqueCurrentFOC torqueRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  // Cached status signals — created once, refreshed in batch each loop
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Temperature> deviceTemp;
  private final StatusSignal<Double> closedLoopReference;
  private final StatusSignal<Double> closedLoopError;
  private final StatusSignal<Angle> pos;

  public UpperFeederIOReal() {
    upperFeederMotor = new TalonFX(HardwareConstants.CanIds.UPPER_FEEDER_MOTOR_ID, CAN_BUS);
    configureFeederMotor();

    // Cache signal references once in the constructor
    velocity = upperFeederMotor.getVelocity();
    statorCurrent = upperFeederMotor.getStatorCurrent();
    supplyCurrent = upperFeederMotor.getSupplyCurrent();
    motorVoltage = upperFeederMotor.getMotorVoltage();
    deviceTemp = upperFeederMotor.getDeviceTemp();
    closedLoopReference = upperFeederMotor.getClosedLoopReference();
    closedLoopError = upperFeederMotor.getClosedLoopError();
    pos = upperFeederMotor.getPosition();

    // 50Hz for signals we need every loop (velocity, voltage, current, closed-loop reference)
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity, statorCurrent, supplyCurrent, motorVoltage, closedLoopReference);

    // 10Hz for diagnostic-only signals (temperature, closed-loop error)
    BaseStatusSignal.setUpdateFrequencyForAll(10.0, deviceTemp, closedLoopError);

    // Stop sending signals we didn't register — reduces CAN bus traffic
    upperFeederMotor.optimizeBusUtilization();
  }

  private void configureFeederMotor() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted =
        UpperFeederConstants.SoftwareConstants.INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = UpperFeederConstants.Mechanical.upperFeederRatio;

    var feederMagic = config.MotionMagic;
    feederMagic.MotionMagicAcceleration =
        UpperFeederConstants.feederMagicConstants.upperFeederAccel;

    // Slot0 PID/FF gains for velocity control
    config.Slot0.kS = UpperFeederConstants.PID.KS;
    config.Slot0.kV = UpperFeederConstants.PID.KV;
    config.Slot0.kP = UpperFeederConstants.PID.KP;

    // Current limits
    var limits = new CurrentLimitsConfigs();
    limits.SupplyCurrentLimit = UpperFeederConstants.CurrentLimits.UPPER_FEEDER_MAIN_SUPPLY_AMP;
    limits.SupplyCurrentLimitEnable = true;
    limits.SupplyCurrentLowerLimit =
        UpperFeederConstants.CurrentLimits.UPPER_FEEDER_MAIN_SUPPLY_TRIGGER_AMP;
    limits.SupplyCurrentLowerTime =
        UpperFeederConstants.CurrentLimits.UPPER_FEEDER_MAIN_SUPPLY_TRIGGER_TIME_SEC.in(Second);
    limits.StatorCurrentLimit = UpperFeederConstants.CurrentLimits.UPPER_FEEDER_MAIN_STATOR_AMP;
    limits.StatorCurrentLimitEnable = true;

    upperFeederMotor.getConfigurator().apply(config);
    upperFeederMotor.getConfigurator().apply(limits);
  }

  @Override
  public void updateInputs(UpperFeederIOInputs inputs) {
    // One batched CAN read for all signals — much faster than individual reads
    BaseStatusSignal.refreshAll(
        velocity,
        statorCurrent,
        supplyCurrent,
        motorVoltage,
        deviceTemp,
        closedLoopReference,
        closedLoopError,
        pos);

    // Read from cache — no additional CAN traffic
    inputs.upperFeederMotorVelocity = velocity.getValue();
    inputs.upperFeederStatorAmps = statorCurrent.getValue();
    inputs.upperFeederSupplyAmps = supplyCurrent.getValue();
    inputs.upperFeederVoltage = motorVoltage.getValue();
    inputs.upperFeederMotorTemperature = deviceTemp.getValue();
    inputs.upperFeederClosedLoopReference =
        RotationsPerSecond.of(closedLoopReference.getValueAsDouble());
    inputs.upperFeederClosedLoopError = RotationsPerSecond.of(closedLoopError.getValueAsDouble());
    inputs.upperFeederPos = pos.getValue();
  }

  @Override
  public void setUpperFeederVoltage(Voltage volts) {
    upperFeederMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setUpperFeederVelocity(AngularVelocity feederVelo) {
    upperFeederMotor.setControl(torqueRequest.withVelocity(feederVelo));
  }
}
