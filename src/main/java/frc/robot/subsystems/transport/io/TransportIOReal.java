package frc.robot.subsystems.transport.io;

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
import frc.robot.subsystems.transport.TransportConstants;

public class TransportIOReal implements TransportIO {

  private static final CANBus CAN_BUS = new CANBus("rio");

  private final TalonFX transportMotor;

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

  public TransportIOReal() {
    transportMotor = new TalonFX(HardwareConstants.CanIds.TRANSPORT_MOTOR_ID, CAN_BUS);
    configureTransportMotor();

    // Cache signal references once in the constructor
    velocity = transportMotor.getVelocity();
    statorCurrent = transportMotor.getStatorCurrent();
    supplyCurrent = transportMotor.getSupplyCurrent();
    motorVoltage = transportMotor.getMotorVoltage();
    deviceTemp = transportMotor.getDeviceTemp();
    closedLoopReference = transportMotor.getClosedLoopReference();
    closedLoopError = transportMotor.getClosedLoopError();

    // 50Hz for signals we need every loop (velocity, voltage, current)
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity, statorCurrent, supplyCurrent, motorVoltage, closedLoopReference);

    // 10Hz for diagnostic-only signals (temperature, closed-loop error)
    BaseStatusSignal.setUpdateFrequencyForAll(10.0, deviceTemp, closedLoopError);

    // Stop sending signals we didn't register — reduces CAN bus traffic
    transportMotor.optimizeBusUtilization();
  }

  private void configureTransportMotor() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.MotorOutput.Inverted =
        TransportConstants.SoftwareConstants.INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = TransportConstants.Mechanical.transportRatio;

    // Slot0 PID/FF gains for velocity control
    config.Slot0.kS = TransportConstants.PID.KS;
    config.Slot0.kV = TransportConstants.PID.KV;
    config.Slot0.kP = TransportConstants.PID.KP;

    var transportMagic = config.MotionMagic;
    transportMagic.MotionMagicAcceleration =
        TransportConstants.transportMagicConstants.transportAccel;

    // Current limits
    var limits = new CurrentLimitsConfigs();
    limits.SupplyCurrentLimit = TransportConstants.CurrentLimits.TRANSPORT_MAIN_SUPPLY_AMP;
    limits.SupplyCurrentLimitEnable = true;
    limits.SupplyCurrentLowerLimit =
        TransportConstants.CurrentLimits.TRANSPORT_MAIN_SUPPLY_TRIGGER_AMP;
    limits.SupplyCurrentLowerTime =
        TransportConstants.CurrentLimits.TRANSPORT_MAIN_SUPPLY_TRIGGER_TIME_SEC.in(Second);
    limits.StatorCurrentLimit = TransportConstants.CurrentLimits.TRANSPORT_MAIN_STATOR_AMP;
    limits.StatorCurrentLimitEnable = true;

    transportMotor.getConfigurator().apply(config);
    transportMotor.getConfigurator().apply(limits);
  }

  @Override
  public void updateInputs(TransportIOInputs inputs) {
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
    inputs.TransportMotorVelocity = velocity.getValue();
    inputs.TransportStatorAmps = statorCurrent.getValue();
    inputs.TransportSupplyAmps = supplyCurrent.getValue();
    inputs.TransportVoltage = motorVoltage.getValue();
    inputs.TransportMotorTemperature = deviceTemp.getValue();
    inputs.transportClosedLoopReference =
        RotationsPerSecond.of(closedLoopReference.getValueAsDouble());
    inputs.transportClosedLoopError = RotationsPerSecond.of(closedLoopError.getValueAsDouble());
  }

  @Override
  public void setTransportVoltage(Voltage volts) {
    transportMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setTransportVelocity(AngularVelocity transportVelo) {
    transportMotor.setControl(torqueRequest.withVelocity(transportVelo));
  }
}
