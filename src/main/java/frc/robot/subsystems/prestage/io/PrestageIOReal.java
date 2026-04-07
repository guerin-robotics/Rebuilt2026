package frc.robot.subsystems.prestage.io;

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
import frc.robot.subsystems.prestage.PrestageConstants;

public class PrestageIOReal implements PrestageIO {

  private static final CANBus CAN_BUS = new CANBus("rio");

  private final TalonFX prestageLeft;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVelocityTorqueCurrentFOC torqueRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  // Cached status signals for LEFT motor
  private final StatusSignal<AngularVelocity> Velocity;
  private final StatusSignal<Current> StatorCurrent;
  private final StatusSignal<Current> SupplyCurrent;
  private final StatusSignal<Voltage> MotorVoltage;
  private final StatusSignal<Temperature> DeviceTemp;
  private final StatusSignal<Double> ClosedLoopReference;
  private final StatusSignal<Double> ClosedLoopError;
  private final StatusSignal<Angle> Pos;

  public PrestageIOReal() {
    prestageLeft = new TalonFX(HardwareConstants.CanIds.PRESTAGE_LEADER_ID, CAN_BUS);
    configurePrestageMotor();

    // Cache signal references once in the constructor
    Velocity = prestageLeft.getVelocity();
    StatorCurrent = prestageLeft.getStatorCurrent();
    SupplyCurrent = prestageLeft.getSupplyCurrent();
    MotorVoltage = prestageLeft.getMotorVoltage();
    DeviceTemp = prestageLeft.getDeviceTemp();
    ClosedLoopReference = prestageLeft.getClosedLoopReference();
    ClosedLoopError = prestageLeft.getClosedLoopError();
    Pos = prestageLeft.getPosition();

    // 50Hz for signals we need every loop (velocity, voltage, current, closed-loop reference)
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, Velocity, StatorCurrent, SupplyCurrent, MotorVoltage, ClosedLoopReference);

    // 10Hz for diagnostic-only signals (temperature, closed-loop error)
    BaseStatusSignal.setUpdateFrequencyForAll(10.0, DeviceTemp, ClosedLoopError);

    // Stop sending signals we didn't register — reduces CAN bus traffic
    prestageLeft.optimizeBusUtilization();
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
  }

  @Override
  public void updateInputs(PrestageIOInputs inputs) {
    // One batched CAN read for ALL signals on BOTH motors
    BaseStatusSignal.refreshAll(
        Velocity,
        StatorCurrent,
        SupplyCurrent,
        MotorVoltage,
        DeviceTemp,
        ClosedLoopReference,
        ClosedLoopError,
        Pos);

    // Left motor — read from cache
    inputs.prestageVelocity = Velocity.getValue();
    inputs.prestageStatorAmps = StatorCurrent.getValue();
    inputs.prestageSupplyAmps = SupplyCurrent.getValue();
    inputs.prestageVoltage = MotorVoltage.getValue();
    inputs.prestageTemperature = DeviceTemp.getValue();
    inputs.prestageClosedLoopReference =
        RotationsPerSecond.of(ClosedLoopReference.getValueAsDouble());
    inputs.prestageClosedLoopError = RotationsPerSecond.of(ClosedLoopError.getValueAsDouble());
    inputs.prestagePos = Pos.getValue();
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
