package frc.robot.subsystems.prestage.io;

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
import frc.robot.subsystems.prestage.PrestageConstants;

public class PrestageIOReal implements PrestageIO {

  private static final CANBus CAN_BUS = new CANBus("rio");

  private final TalonFX prestageMotor;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public PrestageIOReal() {
    prestageMotor = new TalonFX(HardwareConstants.CanIds.PRESTAGE_MOTOR_ID, CAN_BUS);

    // Configure motor
    configurePrestageMotor();
  }

  private void configurePrestageMotor() {
    // Set current limits, neutral mode, etc. as needed
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.MotorOutput.Inverted =
        PrestageConstants.SoftwareConstants.INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

    // Slot0 PID/FF gains for velocity control
    config.Slot0.kS = PrestageConstants.PID.KS;
    config.Slot0.kV = PrestageConstants.PID.KV;
    config.Slot0.kP = PrestageConstants.PID.KP;
    config.Slot0.kI = PrestageConstants.PID.KI;
    config.Slot0.kD = PrestageConstants.PID.KD;

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

    prestageMotor.getConfigurator().apply(config);
    prestageMotor.getConfigurator().apply(limits);
  }

  public void updateInputs(PrestageIOInputs inputs) {
    // Read sensor values and populate inputs object
    inputs.prestageMotorVelocity = prestageMotor.getVelocity().getValue();
    inputs.prestageStatorAmps = prestageMotor.getStatorCurrent().getValue();
    inputs.prestageSupplyAmps = prestageMotor.getSupplyCurrent().getValue();
    inputs.prestageVoltage = prestageMotor.getMotorVoltage().getValue();
    inputs.prestageMotorTemperature = prestageMotor.getDeviceTemp().getValue();
  }

  public void setPrestageVoltage(Voltage volts) {
    prestageMotor.setControl(voltageRequest.withOutput(volts));
  }

  public void setPrestageSpeed(AngularVelocity speed) {
    prestageMotor.setControl(velocityRequest.withVelocity(speed));
  }
}
