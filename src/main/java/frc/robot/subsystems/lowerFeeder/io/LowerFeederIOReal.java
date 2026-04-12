package frc.robot.subsystems.lowerFeeder.io;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.BaseStatusSignal;
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
import frc.robot.subsystems.lowerFeeder.LowerFeederConstants;

public class LowerFeederIOReal implements LowerFeederIO {

  private final TalonFX lowerFeederMotor;

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

  public LowerFeederIOReal() {
    lowerFeederMotor = new TalonFX(HardwareConstants.CanIds.LOWER_FEEDER_MOTOR_ID, HardwareConstants.CAN_BUS);
    configureFeederMotor();

    // Cache signal references once in the constructor
    velocity = lowerFeederMotor.getVelocity();
    statorCurrent = lowerFeederMotor.getStatorCurrent();
    supplyCurrent = lowerFeederMotor.getSupplyCurrent();
    motorVoltage = lowerFeederMotor.getMotorVoltage();
    deviceTemp = lowerFeederMotor.getDeviceTemp();
    closedLoopReference = lowerFeederMotor.getClosedLoopReference();
    closedLoopError = lowerFeederMotor.getClosedLoopError();
    pos = lowerFeederMotor.getPosition();

    // 50Hz for signals we need every loop (velocity, voltage, current, closed-loop reference)
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity, statorCurrent, supplyCurrent, motorVoltage, closedLoopReference);

    // 10Hz for diagnostic-only signals (temperature, closed-loop error)
    BaseStatusSignal.setUpdateFrequencyForAll(10.0, deviceTemp, closedLoopError);

    // Stop sending signals we didn't register — reduces CAN bus traffic
    lowerFeederMotor.optimizeBusUtilization();
  }

  private void configureFeederMotor() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted =
        LowerFeederConstants.SoftwareConstants.INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = LowerFeederConstants.Mechanical.lowerFeederRatio;

    var feederMagic = config.MotionMagic;
    feederMagic.MotionMagicAcceleration =
        LowerFeederConstants.feederMagicConstants.lowerFeederAccel;

    // Slot0 PID/FF gains for velocity control
    config.Slot0.kS = LowerFeederConstants.PID.KS;
    config.Slot0.kV = LowerFeederConstants.PID.KV;
    config.Slot0.kP = LowerFeederConstants.PID.KP;

    // Current limits
    var limits = new CurrentLimitsConfigs();
    limits.SupplyCurrentLimit = LowerFeederConstants.CurrentLimits.LOWER_FEEDER_MAIN_SUPPLY_AMP;
    limits.SupplyCurrentLimitEnable = true;
    limits.SupplyCurrentLowerLimit =
        LowerFeederConstants.CurrentLimits.LOWER_FEEDER_MAIN_SUPPLY_TRIGGER_AMP;
    limits.SupplyCurrentLowerTime =
        LowerFeederConstants.CurrentLimits.LOWER_FEEDER_MAIN_SUPPLY_TRIGGER_TIME_SEC.in(Second);
    limits.StatorCurrentLimit = LowerFeederConstants.CurrentLimits.LOWER_FEEDER_MAIN_STATOR_AMP;
    limits.StatorCurrentLimitEnable = true;

    lowerFeederMotor.getConfigurator().apply(config);
    lowerFeederMotor.getConfigurator().apply(limits);
  }

  @Override
  public void updateInputs(LowerFeederIOInputs inputs) {
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
    inputs.lowerFeederMotorVelocity = velocity.getValue();
    inputs.lowerFeederStatorAmps = statorCurrent.getValue();
    inputs.lowerFeederSupplyAmps = supplyCurrent.getValue();
    inputs.lowerFeederVoltage = motorVoltage.getValue();
    inputs.lowerFeederMotorTemperature = deviceTemp.getValue();
    inputs.lowerFeederClosedLoopReference =
        RotationsPerSecond.of(closedLoopReference.getValueAsDouble());
    inputs.lowerFeederClosedLoopError = RotationsPerSecond.of(closedLoopError.getValueAsDouble());
    inputs.lowerFeederPos = pos.getValue();
  }

  @Override
  public void setLowerFeederVoltage(Voltage volts) {
    lowerFeederMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setLowerFeederVelocity(AngularVelocity feederVelo) {
    lowerFeederMotor.setControl(torqueRequest.withVelocity(feederVelo));
  }
}
