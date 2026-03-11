package frc.robot.subsystems.flywheel.io;

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
import frc.robot.subsystems.flywheel.FlywheelConstants;

/**
 * Simulated implementation of {@link FlywheelIO} using CTRE's TalonFXSimState.
 *
 * <p>Creates 4 real TalonFX objects (1 leader + 3 followers) matching the real robot hardware. The
 * leader's internal closed-loop controller (PID, Motion Magic) runs in simulation. A WPILib {@link
 * FlywheelSim} provides the physics model for the combined 4-motor flywheel, and {@link BatterySim}
 * models battery voltage sag under load.
 *
 * <p>The followers are configured identically to the real robot — they follow the leader's output.
 * In simulation, only the leader's TalonFXSimState is updated with physics; the followers
 * automatically mirror the leader.
 */
public class FlywheelIOSim implements FlywheelIO {

  // 4 real TalonFX objects — their internal firmware runs in sim
  private final TalonFX leader;
  private final TalonFX follower1;
  private final TalonFX follower2;
  private final TalonFX follower3;

  // Sim state for the leader motor (followers mirror the leader)
  private final TalonFXSimState leaderSimState;

  // WPILib DCMotorSim for physics modeling of the combined 4-motor flywheel
  private final DCMotorSim flywheelPhysicsSim;

  // Control requests (reused to avoid allocations)
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVelocityTorqueCurrentFOC velocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  public FlywheelIOSim() {
    // Create all 4 TalonFX motors using the same CAN IDs as real hardware
    leader = new TalonFX(HardwareConstants.CanIds.MAIN_FLYWHEEL_LEADER_ID);
    follower1 = new TalonFX(HardwareConstants.CanIds.MAIN_FLYWHEEL_FOLLOWER1_ID);
    follower2 = new TalonFX(HardwareConstants.CanIds.MAIN_FLYWHEEL_FOLLOWER2_ID);
    follower3 = new TalonFX(HardwareConstants.CanIds.MAIN_FLYWHEEL_FOLLOWER3_ID);

    // Set up follower relationships (matching real robot configuration)
    int leaderId = leader.getDeviceID();
    follower1.setControl(new Follower(leaderId, MotorAlignmentValue.Opposed));
    follower2.setControl(new Follower(leaderId, MotorAlignmentValue.Aligned));
    follower3.setControl(new Follower(leaderId, MotorAlignmentValue.Opposed));

    configureMotors();

    // Get the leader's sim state — this is where we inject physics
    leaderSimState = leader.getSimState();

    // DCMotorSim models the combined 4-motor flywheel as a single spinning mass
    flywheelPhysicsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                FlywheelConstants.Sim.FLYWHEEL_MOTOR,
                FlywheelConstants.Sim.FLYWHEEL_MOI,
                FlywheelConstants.Mechanical.flywheelRatio),
            FlywheelConstants.Sim.FLYWHEEL_MOTOR);
  }

  /** Configures all TalonFX motors with PID gains and mechanical ratios for simulation. */
  private void configureMotors() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted =
        FlywheelConstants.Mechanical.INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = FlywheelConstants.Mechanical.flywheelRatio;

    config.MotionMagic.MotionMagicAcceleration =
        FlywheelConstants.flywheelMagicConstants.flywheelAccel;

    // Sim-specific PID gains (Slot0)
    config.Slot0.kS = FlywheelConstants.Sim.KS;
    config.Slot0.kV = FlywheelConstants.Sim.KV;
    config.Slot0.kP = FlywheelConstants.Sim.KP;
    config.Slot0.kI = FlywheelConstants.Sim.KI;
    config.Slot0.kD = FlywheelConstants.Sim.KD;

    // Apply config to all motors
    leader.getConfigurator().apply(config);
    follower1.getConfigurator().apply(config);
    follower2.getConfigurator().apply(config);
    follower3.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // 1. Tell the sim what the battery voltage is
    leaderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // 2. Get the motor voltage output from the leader's internal controller
    double motorVolts = leaderSimState.getMotorVoltageMeasure().in(Volts);

    // 3. Feed the voltage into the FlywheelSim and step it forward
    flywheelPhysicsSim.setInputVoltage(motorVolts);
    flywheelPhysicsSim.update(0.02); // 20ms loop

    // 4. Write the resulting position and velocity back to the leader's sim state
    //    Note: SimState expects ROTOR values (before gear ratio), so multiply by gear ratio
    double gearRatio = FlywheelConstants.Mechanical.flywheelRatio;
    leaderSimState.setRawRotorPosition(flywheelPhysicsSim.getAngularPosition().times(gearRatio));
    leaderSimState.setRotorVelocity(flywheelPhysicsSim.getAngularVelocity().times(gearRatio));

    // 5. Simulate battery voltage sag based on total current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelPhysicsSim.getCurrentDrawAmps()));

    // 6. Read values from the TalonFX status signals (just like on real hardware)

    // Combined flywheel velocity
    inputs.flywheelVelocity = RotationsPerSecond.of(leader.getVelocity().getValueAsDouble());
    inputs.closedLoopReference =
        RotationsPerSecond.of(leader.getClosedLoopReference().getValueAsDouble());
    inputs.closedLoopError = RotationsPerSecond.of(leader.getClosedLoopError().getValueAsDouble());

    // Leader motor
    inputs.leaderVelocity = RotationsPerSecond.of(leader.getVelocity().getValueAsDouble());
    inputs.leaderAppliedVolts = leader.getMotorVoltage().getValue();
    inputs.leaderSupplyCurrentAmps = leader.getSupplyCurrent().getValue();
    inputs.leaderStatorCurrentAmps = leader.getStatorCurrent().getValue();
    inputs.leaderTemp = leader.getDeviceTemp().getValue();

    // Follower 1 motor
    inputs.follower1Velocity = follower1.getVelocity().getValue();
    inputs.follower1AppliedVolts = follower1.getMotorVoltage().getValue();
    inputs.follower1SupplyCurrentAmps = follower1.getSupplyCurrent().getValue();
    inputs.follower1StatorCurrentAmps = follower1.getStatorCurrent().getValue();
    inputs.follower1Temp = follower1.getDeviceTemp().getValue();

    // Follower 2 motor
    inputs.follower2Velocity = follower2.getVelocity().getValue();
    inputs.follower2AppliedVolts = follower2.getMotorVoltage().getValue();
    inputs.follower2SupplyCurrentAmps = follower2.getSupplyCurrent().getValue();
    inputs.follower2StatorCurrentAmps = follower2.getStatorCurrent().getValue();
    inputs.follower2Temp = follower2.getDeviceTemp().getValue();

    // Follower 3 motor
    inputs.follower3Velocity = follower3.getVelocity().getValue();
    inputs.follower3AppliedVolts = follower3.getMotorVoltage().getValue();
    inputs.follower3SupplyCurrentAmps = follower3.getSupplyCurrent().getValue();
    inputs.follower3StatorCurrentAmps = follower3.getStatorCurrent().getValue();
    inputs.follower3Temp = follower3.getDeviceTemp().getValue();
  }

  @Override
  public void setFlywheelVoltage(Voltage volts) {
    leader.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setFlywheelVelocity(AngularVelocity velocity) {
    leader.setControl(velocityRequest.withVelocity(velocity));
  }
}
