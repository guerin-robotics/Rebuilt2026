package frc.robot.subsystems.lowerFeeder.io;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.lowerFeeder.LowerFeederConstants;

/**
 * Simulated implementation of {@link LowerFeederIO} using CTRE's TalonFXSimState.
 *
 * <p>Creates a real TalonFX object so the internal closed-loop controller (PID, Motion Magic) runs
 * in simulation, just like on the real robot. A WPILib {@link DCMotorSim} provides the physics
 * model, and the motor voltage output is fed into the physics sim each loop.
 */
public class LowerFeederIOSim implements LowerFeederIO {

  // Real TalonFX object — its internal firmware runs in sim
  private final TalonFX lowerFeederMotor;

  // Sim state object for injecting physics into the TalonFX
  private final TalonFXSimState lowerFeederSimState;

  // WPILib physics model for the feeder mechanism
  private final DCMotorSim lowerFeederPhysicsSim;

  // Control requests (reused to avoid allocations)
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVelocityTorqueCurrentFOC velocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  public LowerFeederIOSim() {
    // Create the TalonFX motor (uses the same CAN ID as real hardware)
    lowerFeederMotor = new TalonFX(HardwareConstants.CanIds.LOWER_FEEDER_MOTOR_ID);

    // Apply the motor configuration with sim-specific PID gains
    configureMotor();

    // Get the sim state object for this motor
    lowerFeederSimState = lowerFeederMotor.getSimState();

    // Create the physics simulation model
    lowerFeederPhysicsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                LowerFeederConstants.Sim.LOWER_FEEDER_MOTOR,
                LowerFeederConstants.Sim.LOWER_FEEDER_MOI,
                LowerFeederConstants.Mechanical.lowerFeederRatio),
            LowerFeederConstants.Sim.LOWER_FEEDER_MOTOR);
  }

  /** Configures the TalonFX with PID gains and mechanical ratios for simulation. */
  private void configureMotor() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Match the real robot's invert and gear ratio settings
    config.MotorOutput.Inverted =
        LowerFeederConstants.SoftwareConstants.INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = LowerFeederConstants.Mechanical.lowerFeederRatio;

    // Motion Magic acceleration
    config.MotionMagic.MotionMagicAcceleration =
        LowerFeederConstants.feederMagicConstants.lowerFeederAccel;

    // Sim-specific PID gains (tuned for the physics model)
    config.Slot0.kS = LowerFeederConstants.Sim.KS;
    config.Slot0.kV = LowerFeederConstants.Sim.KV;
    config.Slot0.kP = LowerFeederConstants.Sim.KP;
    config.Slot0.kI = LowerFeederConstants.Sim.KI;
    config.Slot0.kD = LowerFeederConstants.Sim.KD;

    lowerFeederMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(LowerFeederIOInputs inputs) {
    // 1. Tell the sim what the battery voltage is
    lowerFeederSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // 2. Get the motor voltage output from the TalonFX's internal controller
    double motorVolts = lowerFeederSimState.getMotorVoltageMeasure().in(Volts);

    // 3. Feed the voltage into the physics sim and step it forward
    lowerFeederPhysicsSim.setInputVoltage(motorVolts);
    lowerFeederPhysicsSim.update(0.02); // 20ms loop

    // 4. Write the resulting position and velocity back to the TalonFX sim state
    //    Note: SimState expects ROTOR values (before gear ratio), so we multiply by gear ratio
    double gearRatio = LowerFeederConstants.Mechanical.lowerFeederRatio;
    lowerFeederSimState.setRawRotorPosition(
        lowerFeederPhysicsSim.getAngularPosition().times(gearRatio));
    lowerFeederSimState.setRotorVelocity(
        lowerFeederPhysicsSim.getAngularVelocity().times(gearRatio));

    // 5. Simulate battery voltage sag based on current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            lowerFeederPhysicsSim.getCurrentDrawAmps()));

    // 6. Read values from the TalonFX status signals (just like on real hardware)
    inputs.lowerFeederMotorVelocity =
        RotationsPerSecond.of(lowerFeederMotor.getVelocity().getValueAsDouble());
    inputs.lowerFeederVoltage = lowerFeederMotor.getMotorVoltage().getValue();
    inputs.lowerFeederStatorAmps = lowerFeederMotor.getStatorCurrent().getValue();
    inputs.lowerFeederSupplyAmps = lowerFeederMotor.getSupplyCurrent().getValue();
    inputs.lowerFeederMotorTemperature = lowerFeederMotor.getDeviceTemp().getValue();
    inputs.lowerFeederClosedLoopReference =
        RotationsPerSecond.of(lowerFeederMotor.getClosedLoopReference().getValueAsDouble());
    inputs.lowerFeederClosedLoopError =
        RotationsPerSecond.of(lowerFeederMotor.getClosedLoopError().getValueAsDouble());
    inputs.lowerFeederPos = lowerFeederMotor.getPosition().getValue();
  }

  @Override
  public void setLowerFeederVoltage(Voltage volts) {
    lowerFeederMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setLowerFeederVelocity(AngularVelocity feederVelo) {
    lowerFeederMotor.setControl(velocityRequest.withVelocity(feederVelo));
  }
}
