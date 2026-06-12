package frc.robot.subsystems.example.io;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.HardwareConstants;
import frc.util.PhoenixUtil;

/**
 * Real hardware implementation of ExampleSubsystemIO.
 *
 * <p>TEMPLATE INSTRUCTIONS: 1. Replace "TODO_MOTOR_CAN_ID" with your CAN ID constant from
 * HardwareConstants 2. Replace "TODO_CAN_BUS" with "rio" or "Canivore" 3. Configure
 * TalonFXConfiguration for your motor (current limits, neutral mode, etc.) 4. Add StatusSignal
 * fields for every signal in ExampleSubsystemIOInputs 5. Add control request objects (VoltageOut,
 * VelocityTorqueCurrentFOC, etc.) 6. Call BaseStatusSignal.refreshAll() at the top of
 * updateInputs()
 */
public class ExampleSubsystemIOReal implements ExampleSubsystemIO {

  private final TalonFX motor;

  // Status signals — one per logged field
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<edu.wpi.first.units.measure.Current> motorStatorAmps;
  private final StatusSignal<edu.wpi.first.units.measure.Current> motorSupplyAmps;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<edu.wpi.first.units.measure.Temperature> motorTemperature;

  // Control requests
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  public ExampleSubsystemIOReal() {
    motor = new TalonFX(HardwareConstants.CanIds.EXAMPLE_MOTOR, "rio"); // TODO: update CAN bus

    TalonFXConfiguration config = new TalonFXConfiguration();
    // TODO: configure current limits, neutral mode, PID gains
    // config.CurrentLimits.SupplyCurrentLimit = 40;
    // config.CurrentLimits.SupplyCurrentLimitEnable = true;
    // config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));

    // Acquire signal handles
    motorVoltage = motor.getMotorVoltage();
    motorStatorAmps = motor.getStatorCurrent();
    motorSupplyAmps = motor.getSupplyCurrent();
    motorVelocity = motor.getVelocity();
    motorTemperature = motor.getDeviceTemp();

    // Set update frequency (50 Hz is standard; use 250 Hz for odometry-critical signals)
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, motorVoltage, motorStatorAmps, motorSupplyAmps, motorVelocity, motorTemperature);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ExampleSubsystemIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        motorVoltage, motorStatorAmps, motorSupplyAmps, motorVelocity, motorTemperature);

    inputs.motorVoltage = motorVoltage.getValue();
    inputs.motorStatorAmps = motorStatorAmps.getValue();
    inputs.motorSupplyAmps = motorSupplyAmps.getValue();
    inputs.motorVelocity = motorVelocity.getValue();
    inputs.motorTemperature = motorTemperature.getValue();
  }

  @Override
  public void setVoltage(Voltage volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    // TODO: replace VoltageOut with VelocityTorqueCurrentFOC if using closed-loop velocity
    // motor.setControl(velocityRequest.withVelocity(velocity.in(RotationsPerSecond)));
  }
}
