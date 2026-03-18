package frc.robot.subsystems.intakeRoller.io;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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
import frc.robot.subsystems.intakeRoller.intakeRollerConstants;

/**
 * Simulated implementation of {@link intakeRollerIO} using CTRE's TalonFXSimState.
 *
 * <p>Creates a leader TalonFX + follower TalonFX (matching real hardware). The leader's internal
 * closed-loop controller runs in simulation. A WPILib {@link DCMotorSim} provides the physics model
 * for the combined 2-motor intake roller.
 */
public class intakeRollerIOSim implements intakeRollerIO {

  // Real TalonFX objects — their internal firmware runs in sim
  private final TalonFX intakeRollerLeader;
  private final TalonFX intakeRollerFollower;

  // Sim state for the leader motor
  private final TalonFXSimState leaderSimState;

  // WPILib physics model for the combined 2-motor roller
  private final DCMotorSim rollerPhysicsSim;

  // Control requests (reused to avoid allocations)
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVelocityTorqueCurrentFOC velocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  public intakeRollerIOSim() {
    intakeRollerLeader = new TalonFX(HardwareConstants.CanIds.INTAKE_ROLLER_LEADER_ID);
    intakeRollerFollower = new TalonFX(HardwareConstants.CanIds.INTAKE_ROLLER_FOLLOWER_ID);

    // Set up follower to mirror the leader (matching real robot)
    intakeRollerFollower.setControl(
        new Follower(
            HardwareConstants.CanIds.INTAKE_ROLLER_LEADER_ID, MotorAlignmentValue.Aligned));

    configureMotor();

    leaderSimState = intakeRollerLeader.getSimState();

    // Physics sim uses 2 motors combined (leader + follower)
    rollerPhysicsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                intakeRollerConstants.Sim.ROLLER_MOTOR,
                intakeRollerConstants.Sim.ROLLER_MOI,
                intakeRollerConstants.Mechanical.rollerRatio),
            intakeRollerConstants.Sim.ROLLER_MOTOR);
  }

  /** Configures the leader TalonFX with PID gains and mechanical ratios for simulation. */
  private void configureMotor() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.MotorOutput.Inverted =
        intakeRollerConstants.SoftwareConstants.INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = intakeRollerConstants.Mechanical.rollerRatio;

    config.MotionMagic.MotionMagicAcceleration =
        intakeRollerConstants.rollerMagicConstants.rollerAccel;

    // Sim-specific PID gains (tuned for the physics model)
    config.Slot0.kS = intakeRollerConstants.Sim.KS;
    config.Slot0.kV = intakeRollerConstants.Sim.KV;
    config.Slot0.kP = intakeRollerConstants.Sim.KP;
    config.Slot0.kI = intakeRollerConstants.Sim.KI;
    config.Slot0.kD = intakeRollerConstants.Sim.KD;

    intakeRollerLeader.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(intakeRollerIOInputs inputs) {
    // 1. Tell the sim what the battery voltage is
    leaderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // 2. Get the motor voltage output from the leader's internal controller
    double motorVolts = leaderSimState.getMotorVoltageMeasure().in(Volts);

    // 3. Feed the voltage into the physics sim and step it forward
    rollerPhysicsSim.setInputVoltage(motorVolts);
    rollerPhysicsSim.update(0.02);

    // 4. Write the resulting position and velocity back to the leader's sim state
    double gearRatio = intakeRollerConstants.Mechanical.rollerRatio;
    leaderSimState.setRawRotorPosition(rollerPhysicsSim.getAngularPosition().times(gearRatio));
    leaderSimState.setRotorVelocity(rollerPhysicsSim.getAngularVelocity().times(gearRatio));

    // 5. Simulate battery voltage sag
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(rollerPhysicsSim.getCurrentDrawAmps()));

    // 6. Read values from the TalonFX status signals (just like on real hardware)
    inputs.intakeRollerVelocity =
        RotationsPerSecond.of(intakeRollerLeader.getVelocity().getValueAsDouble());
    inputs.intakeRollerVoltage = intakeRollerLeader.getMotorVoltage().getValue();
    inputs.intakeRollerStatorCurrent = intakeRollerLeader.getStatorCurrent().getValue();
    inputs.intakeRollerSupplyCurrent = intakeRollerLeader.getSupplyCurrent().getValue();
    inputs.intakeRollerTemperature = intakeRollerLeader.getDeviceTemp().getValue();
    inputs.rollerClosedLoopReference =
        RotationsPerSecond.of(intakeRollerLeader.getClosedLoopReference().getValueAsDouble());
    inputs.rollerClosedLoopError =
        RotationsPerSecond.of(intakeRollerLeader.getClosedLoopError().getValueAsDouble());
  }

  @Override
  public void setRollerVoltage(Voltage volts) {
    intakeRollerLeader.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setRollerVelocity(AngularVelocity rollerVelo) {
    intakeRollerLeader.setControl(velocityRequest.withVelocity(rollerVelo));
  }
}
