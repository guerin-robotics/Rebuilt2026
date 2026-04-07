package frc.robot.subsystems.prestage.io;

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
import frc.robot.subsystems.prestage.PrestageConstants;

/**
 * Simulated implementation of {@link PrestageIO} using CTRE's TalonFXSimState.
 *
 * <p>Creates two independent TalonFX objects (left and right) so each motor's internal closed-loop
 * controller runs in simulation. Each motor has its own DCMotorSim for physics.
 */
public class PrestageIOSim implements PrestageIO {

  // Left prestage motor and its sim state
  private final TalonFX prestageLeft;
  private final TalonFXSimState leftSimState;
  private final DCMotorSim leftPhysicsSim;

  // Control requests (reused to avoid allocations)
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVelocityTorqueCurrentFOC velocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  public PrestageIOSim() {
    prestageLeft = new TalonFX(HardwareConstants.CanIds.PRESTAGE_LEADER_ID);

    configureMotors();

    leftSimState = prestageLeft.getSimState();

    // Create independent physics sims for each motor
    leftPhysicsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                PrestageConstants.Sim.PRESTAGE_MOTOR,
                PrestageConstants.Sim.PRESTAGE_MOI,
                PrestageConstants.Mechanical.prestageRatio),
            PrestageConstants.Sim.PRESTAGE_MOTOR);
  }

  /** Configures both TalonFX motors with their respective PID gains for simulation. */
  private void configureMotors() {
    // Left motor config
    var leftConfig = new TalonFXConfiguration();
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leftConfig.MotorOutput.Inverted =
        PrestageConstants.SoftwareConstants.INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    leftConfig.Feedback.SensorToMechanismRatio = PrestageConstants.Mechanical.prestageRatio;
    leftConfig.MotionMagic.MotionMagicAcceleration =
        PrestageConstants.prestageMagicConstants.prestageAccel;

    // Left motor sim PID gains
    leftConfig.Slot0.kS = PrestageConstants.Sim.LEFT_KS;
    leftConfig.Slot0.kV = PrestageConstants.Sim.LEFT_KV;
    leftConfig.Slot0.kP = PrestageConstants.Sim.LEFT_KP;
    leftConfig.Slot0.kI = PrestageConstants.Sim.LEFT_KI;
    leftConfig.Slot0.kD = PrestageConstants.Sim.LEFT_KD;

    prestageLeft.getConfigurator().apply(leftConfig);
  }

  @Override
  public void updateInputs(PrestageIOInputs inputs) {
    double gearRatio = PrestageConstants.Mechanical.prestageRatio;

    // --- Left motor sim update ---
    leftSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    double leftMotorVolts = leftSimState.getMotorVoltageMeasure().in(Volts);
    leftPhysicsSim.setInputVoltage(leftMotorVolts);
    leftPhysicsSim.update(0.02);
    leftSimState.setRawRotorPosition(leftPhysicsSim.getAngularPosition().times(gearRatio));
    leftSimState.setRotorVelocity(leftPhysicsSim.getAngularVelocity().times(gearRatio));

    // Simulate battery voltage sag from both motors combined
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(leftPhysicsSim.getCurrentDrawAmps()));

    // Read left motor values from TalonFX status signals
    inputs.prestageVelocity = RotationsPerSecond.of(prestageLeft.getVelocity().getValueAsDouble());
    inputs.prestageVoltage = prestageLeft.getMotorVoltage().getValue();
    inputs.prestageStatorAmps = prestageLeft.getStatorCurrent().getValue();
    inputs.prestageSupplyAmps = prestageLeft.getSupplyCurrent().getValue();
    inputs.prestageTemperature = prestageLeft.getDeviceTemp().getValue();
    inputs.prestageClosedLoopReference =
        RotationsPerSecond.of(prestageLeft.getClosedLoopReference().getValueAsDouble());
    inputs.prestageClosedLoopError =
        RotationsPerSecond.of(prestageLeft.getClosedLoopError().getValueAsDouble());
  }

  @Override
  public void setPrestageVoltage(Voltage volts) {
    prestageLeft.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPrestageVelocity(AngularVelocity prestageVelo) {
    prestageLeft.setControl(velocityRequest.withVelocity(prestageVelo));
  }
}
