package frc.robot.subsystems.intakeSlider.io;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.intakeSlider.intakeSliderConstants;

/**
 * Simulated implementation of {@link intakeSliderIO} using CTRE's TalonFXSimState and
 * CANcoderSimState.
 *
 * <p>Creates a real TalonFX object so the internal closed-loop controller runs in simulation. A
 * CANcoder is also created and its sim state is updated with the position from the physics model,
 * matching the real robot's setup where position is read from the CANcoder.
 */
public class intakeSliderIOSim implements intakeSliderIO {

  // Real TalonFX and CANcoder objects — their internal firmware runs in sim
  private final TalonFX sliderMotor;
  private final CANcoder sliderEncoder;

  // Sim state objects for injecting physics
  private final TalonFXSimState motorSimState;
  private final CANcoderSimState encoderSimState;

  // WPILib physics model for the slider mechanism
  private final DCMotorSim sliderPhysicsSim;

  // Control requests (reused to avoid allocations)
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVelocityTorqueCurrentFOC velocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  public intakeSliderIOSim() {
    sliderMotor = new TalonFX(HardwareConstants.CanIds.INTAKE_SLIDER_MOTOR_ID);
    sliderEncoder = new CANcoder(HardwareConstants.CanIds.INTAKE_SLIDER_ENCODER_ID);

    configureMotor();

    motorSimState = sliderMotor.getSimState();
    encoderSimState = sliderEncoder.getSimState();

    sliderPhysicsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                intakeSliderConstants.Sim.SLIDER_MOTOR,
                intakeSliderConstants.Sim.SLIDER_MOI,
                intakeSliderConstants.Mechanical.sliderRatio),
            intakeSliderConstants.Sim.SLIDER_MOTOR);
  }

  /** Configures the TalonFX with PID gains and mechanical ratios for simulation. */
  private void configureMotor() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.MotorOutput.Inverted =
        intakeSliderConstants.SoftwareConstants.MOTOR_INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = intakeSliderConstants.Mechanical.sliderRatio;

    config.MotionMagic.MotionMagicAcceleration =
        intakeSliderConstants.sliderMagicConstants.sliderAccel;

    // Sim-specific PID gains (Slot0)
    config.Slot0.kS = intakeSliderConstants.Sim.KS;
    config.Slot0.kV = intakeSliderConstants.Sim.KV;
    config.Slot0.kP = intakeSliderConstants.Sim.KP;
    config.Slot0.kI = intakeSliderConstants.Sim.KI;
    config.Slot0.kD = intakeSliderConstants.Sim.KD;

    sliderMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IntakeSliderIOInputs inputs) {
    // 1. Tell the sim what the battery voltage is
    motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    encoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // 2. Get the motor voltage output from the TalonFX's internal controller
    double motorVolts = motorSimState.getMotorVoltageMeasure().in(Volts);

    // 3. Feed the voltage into the physics sim and step it forward
    sliderPhysicsSim.setInputVoltage(motorVolts);
    sliderPhysicsSim.update(0.02);

    // 4. Write the resulting position and velocity back to the TalonFX sim state
    double gearRatio = intakeSliderConstants.Mechanical.sliderRatio;
    motorSimState.setRawRotorPosition(sliderPhysicsSim.getAngularPosition().times(gearRatio));
    motorSimState.setRotorVelocity(sliderPhysicsSim.getAngularVelocity().times(gearRatio));

    // 5. Also update the CANcoder sim state with the mechanism position/velocity
    //    The CANcoder reads mechanism position directly (no gear ratio needed)
    encoderSimState.setRawPosition(sliderPhysicsSim.getAngularPosition());
    encoderSimState.setVelocity(sliderPhysicsSim.getAngularVelocity());

    // 6. Simulate battery voltage sag
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(sliderPhysicsSim.getCurrentDrawAmps()));

    // 7. Read values from the hardware objects (just like on real hardware)
    //    Velocity and position come from the CANcoder (matching real robot IO)
    inputs.intakeSliderVelocity =
        RotationsPerSecond.of(sliderEncoder.getVelocity().getValueAsDouble());
    inputs.intakeSliderPosition = sliderEncoder.getAbsolutePosition().getValueAsDouble();

    // Voltage, current, temperature from the motor
    inputs.intakeSliderVoltage = sliderMotor.getMotorVoltage().getValue();
    inputs.intakeSliderStatorCurrent = sliderMotor.getStatorCurrent().getValueAsDouble();
    inputs.intakeSliderSupplyCurrent = sliderMotor.getSupplyCurrent().getValue();
    inputs.intakeSliderTemperature = sliderMotor.getDeviceTemp().getValue();
    inputs.intakeSliderClosedLoopReference =
        RotationsPerSecond.of(sliderMotor.getClosedLoopReference().getValueAsDouble());
    inputs.intakeSliderClosedLoopError =
        RotationsPerSecond.of(sliderMotor.getClosedLoopError().getValueAsDouble());
  }

  @Override
  public void setSliderVoltage(Voltage volts) {
    sliderMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setSliderVelocity(AngularVelocity velocity) {
    sliderMotor.setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public void zeroSliderEncoder() {
    sliderEncoder.setPosition(0);
  }
}
