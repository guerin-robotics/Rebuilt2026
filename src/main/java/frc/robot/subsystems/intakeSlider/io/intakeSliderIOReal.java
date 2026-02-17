package frc.robot.subsystems.intakeSlider.io;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.HardwareConstants;
import frc.robot.subsystems.intakeSlider.intakeSliderConstants;
import static edu.wpi.first.units.Units.Second;
import edu.wpi.first.units.measure.Voltage;

public class intakeSliderIOReal implements intakeSliderIO {
    
    private static final CANBus CAN_BUS = new CANBus("rio");

    private final TalonFX intakeSliderMotor;

    private final VoltageOut voltageRequest = new VoltageOut(0);

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
                ?com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
                :com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

        // Slot0 PID/FF gains for velocity control
        config.Slot0.kS = intakeSliderConstants.PID.KS;
        config.Slot0.kV = intakeSliderConstants.PID.KV;
        config.Slot0.kP = intakeSliderConstants.PID.KP;
        config.Slot0.kI = intakeSliderConstants.PID.KI;
        config.Slot0.kD = intakeSliderConstants.PID.KD;

        // Current limits
        var limits = new CurrentLimitsConfigs();
            limits.SupplyCurrentLimit = intakeSliderConstants.CurrentLimits.INTAKE_SLIDER_MAIN_SUPPLY_AMP;
            limits.SupplyCurrentLimitEnable = true;
            limits.SupplyCurrentLowerLimit = intakeSliderConstants.CurrentLimits.INTAKE_SLIDER_MAIN_SUPPLY_TRIGGER_AMP;
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
}
