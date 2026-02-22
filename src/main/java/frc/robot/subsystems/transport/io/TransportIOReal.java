package frc.robot.subsystems.transport.io;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.transport.TransportConstants;

public class TransportIOReal implements TransportIO {

  private static final CANBus CAN_BUS = new CANBus("rio");

  private final TalonFX transportMotor;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVelocityTorqueCurrentFOC torqueRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  public TransportIOReal() {
    transportMotor = new TalonFX(HardwareConstants.CanIds.TRANSPORT_MOTOR_ID, CAN_BUS);
    // Configure motor
    configureTransportMotor();
  }

  private void configureTransportMotor() {
    // Set current limits, neutral mode, etc. as needed
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.MotorOutput.Inverted =
        TransportConstants.SoftwareConstants.INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

    // Slot0 PID/FF gains for velocity control
    config.Slot0.kS = TransportConstants.PID.KS;
    config.Slot0.kV = TransportConstants.PID.KV;
    config.Slot0.kP = TransportConstants.PID.KP;
    config.Slot0.kI = TransportConstants.PID.KI;
    config.Slot0.kD = TransportConstants.PID.KD;

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
    // Read sensor values and populate inputs object
    inputs.TransportMotorVelocity = transportMotor.getVelocity().getValue();
    inputs.TransportStatorAmps = transportMotor.getStatorCurrent().getValue();
    inputs.TransportSupplyAmps = transportMotor.getSupplyCurrent().getValue();
    inputs.TransportVoltage = transportMotor.getMotorVoltage().getValue();
    inputs.TransportMotorTemperature = transportMotor.getDeviceTemp().getValue();
    inputs.transportClosedLoopReference = RotationsPerSecond.of(transportMotor.getClosedLoopReference().getValueAsDouble());
    inputs.transportClosedLoopError = RotationsPerSecond.of(transportMotor.getClosedLoopError().getValueAsDouble());
  }

  public void setTransportVoltage(Voltage volts) {
    transportMotor.setControl(voltageRequest.withOutput(volts));
  }

  public void setTransportSpeed(AngularVelocity speed) {
    transportMotor.setControl(velocityRequest.withVelocity(speed));
  }

  public void setTransportTorque(AngularVelocity transportVelo) {
    transportMotor.setControl(torqueRequest.withVelocity(transportVelo));
  }
}
