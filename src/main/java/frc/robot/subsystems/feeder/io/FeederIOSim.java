package frc.robot.subsystems.feeder.io;

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
import frc.robot.subsystems.feeder.FeederConstants;

/**
 * Simulated implementation of {@link FeederIO} using CTRE's TalonFXSimState.
 *
 * <p>Creates a real TalonFX object so the internal closed-loop controller (PID, Motion Magic) runs
 * in simulation, just like on the real robot. A WPILib {@link DCMotorSim} provides the physics
 * model, and the motor voltage output is fed into the physics sim each loop.
 */
public class FeederIOSim implements FeederIO {

  // Real TalonFX object — its internal firmware runs in sim
  private final TalonFX feederMotor;

  // Sim state object for injecting physics into the TalonFX
  private final TalonFXSimState feederSimState;

  // WPILib physics model for the feeder mechanism
  private final DCMotorSim feederPhysicsSim;

  // Control requests (reused to avoid allocations)
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVelocityTorqueCurrentFOC velocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  public FeederIOSim() {
    // Create the TalonFX motor (uses the same CAN ID as real hardware)
    feederMotor = new TalonFX(HardwareConstants.CanIds.FEEDER_MOTOR_ID);

    // Apply the motor configuration with sim-specific PID gains
    configureMotor();

    // Get the sim state object for this motor
    feederSimState = feederMotor.getSimState();

    // Create the physics simulation model
    feederPhysicsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                FeederConstants.Sim.FEEDER_MOTOR,
                FeederConstants.Sim.FEEDER_MOI,
                FeederConstants.Mechanical.feederRatio),
            FeederConstants.Sim.FEEDER_MOTOR);
  }

  /** Configures the TalonFX with PID gains and mechanical ratios for simulation. */
  private void configureMotor() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Match the real robot's invert and gear ratio settings
    config.MotorOutput.Inverted =
        FeederConstants.SoftwareConstants.INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = FeederConstants.Mechanical.feederRatio;

    // Motion Magic acceleration
    config.MotionMagic.MotionMagicAcceleration = FeederConstants.feederMagicConstants.feederAccel;

    // Sim-specific PID gains (Slot0)
    config.Slot0.kS = FeederConstants.Sim.KS;
    config.Slot0.kV = FeederConstants.Sim.KV;
    config.Slot0.kP = FeederConstants.Sim.KP;
    config.Slot0.kI = FeederConstants.Sim.KI;
    config.Slot0.kD = FeederConstants.Sim.KD;

    feederMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    // 1. Tell the sim what the battery voltage is
    feederSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // 2. Get the motor voltage output from the TalonFX's internal controller
    double motorVolts = feederSimState.getMotorVoltageMeasure().in(Volts);

    // 3. Feed the voltage into the physics sim and step it forward
    feederPhysicsSim.setInputVoltage(motorVolts);
    feederPhysicsSim.update(0.02); // 20ms loop

    // 4. Write the resulting position and velocity back to the TalonFX sim state
    //    Note: SimState expects ROTOR values (before gear ratio), so we multiply by gear ratio
    double gearRatio = FeederConstants.Mechanical.feederRatio;
    feederSimState.setRawRotorPosition(feederPhysicsSim.getAngularPosition().times(gearRatio));
    feederSimState.setRotorVelocity(feederPhysicsSim.getAngularVelocity().times(gearRatio));

    // 5. Simulate battery voltage sag based on current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(feederPhysicsSim.getCurrentDrawAmps()));

    // 6. Read values from the TalonFX status signals (just like on real hardware)
    inputs.feederMotorVelocity =
        RotationsPerSecond.of(feederMotor.getVelocity().getValueAsDouble());
    inputs.feederVoltage = feederMotor.getMotorVoltage().getValue();
    inputs.feederStatorAmps = feederMotor.getStatorCurrent().getValue();
    inputs.feederSupplyAmps = feederMotor.getSupplyCurrent().getValue();
    inputs.feederMotorTemperature = feederMotor.getDeviceTemp().getValue();
    inputs.feederClosedLoopReference =
        RotationsPerSecond.of(feederMotor.getClosedLoopReference().getValueAsDouble());
    inputs.feederClosedLoopError =
        RotationsPerSecond.of(feederMotor.getClosedLoopError().getValueAsDouble());
  }

  @Override
  public void setFeederVoltage(Voltage volts) {
    feederMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setFeederVelocity(AngularVelocity feederVelo) {
    feederMotor.setControl(velocityRequest.withVelocity(feederVelo));
  }
}
