package frc.robot.subsystems.intakeSlider.io;

import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.intakeSlider.intakeSliderConstants;

public class intakeSliderIOReal implements intakeSliderIO {

  private static final CANBus CAN_BUS = new CANBus("rio");

  private final TalonFX intakeSliderMotor;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrentFOC(0);

  public intakeSliderIOReal() {
    intakeSliderMotor = new TalonFX(HardwareConstants.CanIds.INTAKE_SLIDER_MOTOR_ID, CAN_BUS);

    configureSliderMotor();
  }

  private void configureSliderMotor() {
    // Set current limits, neutral mode, etc. as needed
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.MotorOutput.Inverted =
        intakeSliderConstants.SoftwareConstants.INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

    // Slot0 gains for position control
    config.Slot0.kS = intakeSliderConstants.PID.KS;
    // config.Slot0.kV = intakeSliderConstants.PID.KV;
    config.Slot0.kP = intakeSliderConstants.PID.KP;
    // config.Slot0.kI = intakeSliderConstants.PID.KI;
    config.Slot0.kD = intakeSliderConstants.PID.KD;

    // Current limits
    var limits = new CurrentLimitsConfigs();
    limits.SupplyCurrentLimit = intakeSliderConstants.CurrentLimits.INTAKE_SLIDER_MAIN_SUPPLY_AMP;
    limits.SupplyCurrentLimitEnable = true;
    limits.SupplyCurrentLowerLimit =
        intakeSliderConstants.CurrentLimits.INTAKE_SLIDER_MAIN_SUPPLY_TRIGGER_AMP;
    limits.SupplyCurrentLowerTime =
        intakeSliderConstants.CurrentLimits.INTAKE_SLIDER_MAIN_SUPPLY_TRIGGER_TIME_SEC.in(Second);
    limits.StatorCurrentLimit = intakeSliderConstants.CurrentLimits.INTAKE_SLIDER_MAIN_STATOR_AMP;
    limits.StatorCurrentLimitEnable = true;

    intakeSliderMotor.getConfigurator().apply(config);
    intakeSliderMotor.getConfigurator().apply(limits);
  }

  public void updateInputs(IntakeSliderIOInputs inputs) {
    inputs.intakeSliderVoltage = intakeSliderMotor.getMotorVoltage().getValue();
    inputs.intakeSliderVelocity = intakeSliderMotor.getVelocity().getValue();
    inputs.intakeSliderStatorCurrent = intakeSliderMotor.getStatorCurrent().getValue();
    inputs.intakeSliderSupplyCurrent = intakeSliderMotor.getSupplyCurrent().getValue();
    inputs.intakeSliderTemperature = intakeSliderMotor.getDeviceTemp().getValue();
  }

  public void setIntakeSliderVoltage(Voltage volts) {
    intakeSliderMotor.setControl(voltageRequest.withOutput(volts));
  }

  // Setting position basic
  public void setIntakePos(double setpoint) {
    intakeSliderMotor.setControl(positionRequest.withPosition(setpoint));
  }

  // Setting position for use in pulse sequence. Uses PositionTorqueCurrent control
  public void setIntakePosForPulse(double rotations) {
    double positionSetpoint;
    double currentPoint = intakeSliderMotor.getPosition().getValueAsDouble();
    if (currentPoint > HardwareConstants.intakeRotations) {
      positionSetpoint = currentPoint - rotations;
    } else {
      positionSetpoint = currentPoint + rotations;
    }
    intakeSliderMotor.setControl(positionRequest.withPosition(positionSetpoint));
  }

  // Smarter pulse sequence
  // Logic: Retracts until current draw spikes, then extends +2.0 rotations, repeat
  public void intakeRetract(Voltage retractVolts, double extension) {
    double currentPos = intakeSliderMotor.getPosition().getValueAsDouble();
    if (intakeSliderMotor.getStatorCurrent().getValueAsDouble() < 2.0) {
      intakeSliderMotor.setControl(voltageRequest.withOutput(retractVolts));
    } else {
      intakeSliderMotor.setControl(positionRequest.withPosition(currentPos + extension));
    }
  }

  public void intakeWait(double seconds) {
    Timer intakeTimer = new Timer();
    intakeTimer.start();
    if (intakeTimer.get() > 0.5) {
      intakeTimer.stop();
    }
  }

}
