package frc.robot.subsystems.feeder.io;

import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.feeder.FeederConstants;

public class FeederIOReal implements FeederIO {

  private static final CANBus CAN_BUS = new CANBus("rio");

  private final TalonFX feederMotor;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public FeederIOReal() {
    feederMotor = new TalonFX(HardwareConstants.CanIds.FEEDER_MOTOR_ID, CAN_BUS);

    // Configure motor
    configureFeederMotor();
  }

  private void configureFeederMotor() {
    // Set current limits, neutral mode, etc. as needed
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.MotorOutput.Inverted =
        FeederConstants.SoftwareConstants.INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

    // Slot0 PID/FF gains for velocity control
    config.Slot0.kS = FeederConstants.PID.KS;
    config.Slot0.kV = FeederConstants.PID.KV;
    config.Slot0.kP = FeederConstants.PID.KP;
    config.Slot0.kI = FeederConstants.PID.KI;
    config.Slot0.kD = FeederConstants.PID.KD;

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

  public void updateInputs(FeederIOInputs inputs) {
    // Read sensor values and populate inputs object
    inputs.feederMotorVelocity = feederMotor.getVelocity().getValue();
    inputs.feederStatorAmps = feederMotor.getStatorCurrent().getValue();
    inputs.feederSupplyAmps = feederMotor.getSupplyCurrent().getValue();
    inputs.feederVoltage = feederMotor.getMotorVoltage().getValue();
    inputs.feederMotorTemperature = feederMotor.getDeviceTemp().getValue();
  }

  public void setFeederVoltage(Voltage volts) {
    feederMotor.setControl(voltageRequest.withOutput(volts));
  }

  public void setFeederSpeed(AngularVelocity speed) {
    feederMotor.setControl(velocityRequest.withVelocity(speed));
  }
}
