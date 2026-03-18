package frc.robot.subsystems.transport.io;

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
import frc.robot.subsystems.transport.TransportConstants;

/**
 * Simulated implementation of {@link TransportIO} using CTRE's TalonFXSimState.
 *
 * <p>Creates a real TalonFX object so the internal closed-loop controller (PID, Motion Magic) runs
 * in simulation. A WPILib {@link DCMotorSim} provides the physics model.
 */
public class TransportIOSim implements TransportIO {

  // Real TalonFX object — its internal firmware runs in sim
  private final TalonFX transportMotor;

  // Sim state object for injecting physics into the TalonFX
  private final TalonFXSimState transportSimState;

  // WPILib physics model for the transport mechanism
  private final DCMotorSim transportPhysicsSim;

  // Control requests (reused to avoid allocations)
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVelocityTorqueCurrentFOC velocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  public TransportIOSim() {
    transportMotor = new TalonFX(HardwareConstants.CanIds.TRANSPORT_MOTOR_ID);

    configureMotor();

    transportSimState = transportMotor.getSimState();

    transportPhysicsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                TransportConstants.Sim.TRANSPORT_MOTOR,
                TransportConstants.Sim.TRANSPORT_MOI,
                TransportConstants.Mechanical.transportRatio),
            TransportConstants.Sim.TRANSPORT_MOTOR);
  }

  /** Configures the TalonFX with PID gains and mechanical ratios for simulation. */
  private void configureMotor() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.MotorOutput.Inverted =
        TransportConstants.SoftwareConstants.INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = TransportConstants.Mechanical.transportRatio;

    config.MotionMagic.MotionMagicAcceleration =
        TransportConstants.transportMagicConstants.transportAccel;

    // Sim-specific PID gains (tuned for the physics model)
    config.Slot0.kS = TransportConstants.Sim.KS;
    config.Slot0.kV = TransportConstants.Sim.KV;
    config.Slot0.kP = TransportConstants.Sim.KP;
    config.Slot0.kI = TransportConstants.Sim.KI;
    config.Slot0.kD = TransportConstants.Sim.KD;

    transportMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(TransportIOInputs inputs) {
    // 1. Tell the sim what the battery voltage is
    transportSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // 2. Get the motor voltage output from the TalonFX's internal controller
    double motorVolts = transportSimState.getMotorVoltageMeasure().in(Volts);

    // 3. Feed the voltage into the physics sim and step it forward
    transportPhysicsSim.setInputVoltage(motorVolts);
    transportPhysicsSim.update(0.02);

    // 4. Write the resulting position and velocity back to the TalonFX sim state
    double gearRatio = TransportConstants.Mechanical.transportRatio;
    transportSimState.setRawRotorPosition(
        transportPhysicsSim.getAngularPosition().times(gearRatio));
    transportSimState.setRotorVelocity(transportPhysicsSim.getAngularVelocity().times(gearRatio));

    // 5. Simulate battery voltage sag
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(transportPhysicsSim.getCurrentDrawAmps()));

    // 6. Read values from the TalonFX status signals (just like on real hardware)
    inputs.TransportMotorVelocity = transportMotor.getVelocity().getValue();
    inputs.TransportVoltage = transportMotor.getMotorVoltage().getValue();
    inputs.TransportStatorAmps = transportMotor.getStatorCurrent().getValue();
    inputs.TransportSupplyAmps = transportMotor.getSupplyCurrent().getValue();
    inputs.TransportMotorTemperature = transportMotor.getDeviceTemp().getValue();
    inputs.transportClosedLoopReference =
        RotationsPerSecond.of(transportMotor.getClosedLoopReference().getValueAsDouble());
    inputs.transportClosedLoopError =
        RotationsPerSecond.of(transportMotor.getClosedLoopError().getValueAsDouble());
  }

  @Override
  public void setTransportVoltage(Voltage volts) {
    transportMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setTransportVelocity(AngularVelocity transportVelo) {
    transportMotor.setControl(velocityRequest.withVelocity(transportVelo));
  }
}
