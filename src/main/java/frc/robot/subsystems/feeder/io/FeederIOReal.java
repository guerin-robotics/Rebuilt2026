package frc.robot.subsystems.feeder.io;

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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.feeder.FeederConstants;

public class FeederIOReal implements FeederIO {

  private static final CANBus CAN_BUS = new CANBus("rio");

  private final TalonFX feederMotor;

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

  public FeederIOReal() {
    feederMotor = new TalonFX(HardwareConstants.CanIds.FEEDER_MOTOR_ID, CAN_BUS);
    configureFeederMotor();

    // Cache signal references once in the constructor
    velocity = feederMotor.getVelocity();
    statorCurrent = feederMotor.getStatorCurrent();
    supplyCurrent = feederMotor.getSupplyCurrent();
    motorVoltage = feederMotor.getMotorVoltage();
    deviceTemp = feederMotor.getDeviceTemp();
    closedLoopReference = feederMotor.getClosedLoopReference();
    closedLoopError = feederMotor.getClosedLoopError();

    // Set update frequency for all signals (50Hz is plenty for non-odometry)
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        velocity,
        statorCurrent,
        supplyCurrent,
        motorVoltage,
        deviceTemp,
        closedLoopReference,
        closedLoopError);

    // Stop sending signals we didn't register — reduces CAN bus traffic
    feederMotor.optimizeBusUtilization();
  }

  private void configureFeederMotor() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted =
        FeederConstants.SoftwareConstants.INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = FeederConstants.Mechanical.feederRatio;

    var feederMagic = config.MotionMagic;
    feederMagic.MotionMagicAcceleration = FeederConstants.feederMagicConstants.feederAccel;

    // Slot0 PID/FF gains for velocity control
    config.Slot0.kS = FeederConstants.PID.KS;
    config.Slot0.kV = FeederConstants.PID.KV;
    config.Slot0.kP = FeederConstants.PID.KP;

    // Current limits
    var limits = new CurrentLimitsConfigs();
    limits.SupplyCurrentLimit = FeederConstants.CurrentLimits.FEEDER_MAIN_SUPPLY_AMP;
    limits.SupplyCurrentLimitEnable = true;
    limits.SupplyCurrentLowerLimit = FeederConstants.CurrentLimits.FEEDER_MAIN_SUPPLY_TRIGGER_AMP;
    limits.SupplyCurrentLowerTime =
        FeederConstants.CurrentLimits.FEEDER_MAIN_SUPPLY_TRIGGER_TIME_SEC.in(Second);
    limits.StatorCurrentLimit = FeederConstants.CurrentLimits.FEEDER_MAIN_STATOR_AMP;
    limits.StatorCurrentLimitEnable = true;

    feederMotor.getConfigurator().apply(config);
    feederMotor.getConfigurator().apply(limits);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    // One batched CAN read for all signals — much faster than individual reads
    BaseStatusSignal.refreshAll(
        velocity,
        statorCurrent,
        supplyCurrent,
        motorVoltage,
        deviceTemp,
        closedLoopReference,
        closedLoopError);

    // Read from cache — no additional CAN traffic
    inputs.feederMotorVelocity = velocity.getValue();
    inputs.feederStatorAmps = statorCurrent.getValue();
    inputs.feederSupplyAmps = supplyCurrent.getValue();
    inputs.feederVoltage = motorVoltage.getValue();
    inputs.feederMotorTemperature = deviceTemp.getValue();
    inputs.feederClosedLoopReference =
        RotationsPerSecond.of(closedLoopReference.getValueAsDouble());
    inputs.feederClosedLoopError = RotationsPerSecond.of(closedLoopError.getValueAsDouble());
  }

  @Override
  public void setFeederVoltage(Voltage volts) {
    feederMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setFeederVelocity(AngularVelocity feederVelo) {
    feederMotor.setControl(torqueRequest.withVelocity(feederVelo));
  }
}
