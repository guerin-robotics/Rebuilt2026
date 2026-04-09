package frc.robot.subsystems.flywheel.io;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.flywheel.FlywheelConstants;
import org.littletonrobotics.junction.Logger;

/**
 * CTRE Phoenix 6 implementation of ShooterIO.
 *
 * <p>Controls 4x TalonFX motors (1 leader + 3 followers) as a single flywheel group. Uses
 * VoltageOut for feedforward control (computed in Flywheel subsystem).
 *
 * <p><b>Hardware:</b>
 *
 * <ul>
 *   <li>Leader: CAN ID 37 (left bay top)
 *   <li>Follower 1: CAN ID 34 (left bay lower, opposed)
 *   <li>Follower 2: CAN ID 36 (right bay top, opposed)
 *   <li>Follower 3: CAN ID 35 (right bay bottom, aligned)
 * </ul>
 */
public class FlywheelIOPhoenix6 implements FlywheelIO {

  private static final CANBus CAN_BUS = new CANBus("rio");

  // 4x TalonFX motors for main flywheel
  private final TalonFX leader;
  private final TalonFX follower1;
  private final TalonFX follower2;
  private final TalonFX follower3;
  private final TalonFX follower4;

  // Control requests (reused to avoid allocations)
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  // Cached status signals for LEADER motor
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderMotorVoltage;
  private final StatusSignal<Current> leaderSupplyCurrent;
  private final StatusSignal<Current> leaderStatorCurrent;
  private final StatusSignal<Temperature> leaderTemp;
  private final StatusSignal<Double> closedLoopReference;
  private final StatusSignal<Double> closedLoopError;
  private final StatusSignal<Angle> leaderPos;

  // Cached status signals for FOLLOWER 1
  private final StatusSignal<AngularVelocity> follower1Velocity;
  private final StatusSignal<Voltage> follower1MotorVoltage;
  private final StatusSignal<Current> follower1SupplyCurrent;
  private final StatusSignal<Current> follower1StatorCurrent;
  private final StatusSignal<Temperature> follower1Temp;

  // Cached status signals for FOLLOWER 2
  private final StatusSignal<AngularVelocity> follower2Velocity;
  private final StatusSignal<Voltage> follower2MotorVoltage;
  private final StatusSignal<Current> follower2SupplyCurrent;
  private final StatusSignal<Current> follower2StatorCurrent;
  private final StatusSignal<Temperature> follower2Temp;

  // Cached status signals for FOLLOWER 3
  private final StatusSignal<AngularVelocity> follower3Velocity;
  private final StatusSignal<Voltage> follower3MotorVoltage;
  private final StatusSignal<Current> follower3SupplyCurrent;
  private final StatusSignal<Current> follower3StatorCurrent;
  private final StatusSignal<Temperature> follower3Temp;

  // Cached status signals for FOLLOWER 4
  private final StatusSignal<AngularVelocity> follower4Velocity;
  private final StatusSignal<Voltage> follower4MotorVoltage;
  private final StatusSignal<Current> follower4SupplyCurrent;
  private final StatusSignal<Current> follower4StatorCurrent;
  private final StatusSignal<Temperature> follower4Temp;

  public FlywheelIOPhoenix6() {
    leader = new TalonFX(HardwareConstants.CanIds.MAIN_FLYWHEEL_LEADER_ID, CAN_BUS);
    follower1 = new TalonFX(HardwareConstants.CanIds.MAIN_FLYWHEEL_FOLLOWER1_ID, CAN_BUS);
    follower2 = new TalonFX(HardwareConstants.CanIds.MAIN_FLYWHEEL_FOLLOWER2_ID, CAN_BUS);
    follower3 = new TalonFX(HardwareConstants.CanIds.MAIN_FLYWHEEL_FOLLOWER3_ID, CAN_BUS);
    follower4 = new TalonFX(HardwareConstants.CanIds.MAIN_FLYWHEEL_FOLLOWER4_ID, CAN_BUS);

    // Configure followers
    int leaderId = leader.getDeviceID();
    follower1.setControl(new Follower(leaderId, MotorAlignmentValue.Aligned));
    follower2.setControl(new Follower(leaderId, MotorAlignmentValue.Aligned));
    follower3.setControl(new Follower(leaderId, MotorAlignmentValue.Opposed));
    follower4.setControl(new Follower(leaderId, MotorAlignmentValue.Opposed));

    configureMotors();

    // Cache signal references once — LEADER
    leaderVelocity = leader.getVelocity();
    leaderMotorVoltage = leader.getMotorVoltage();
    leaderSupplyCurrent = leader.getSupplyCurrent();
    leaderStatorCurrent = leader.getStatorCurrent();
    leaderTemp = leader.getDeviceTemp();
    closedLoopReference = leader.getClosedLoopReference();
    closedLoopError = leader.getClosedLoopError();
    leaderPos = leader.getPosition();

    // Cache signal references once — FOLLOWER 1
    follower1Velocity = follower1.getVelocity();
    follower1MotorVoltage = follower1.getMotorVoltage();
    follower1SupplyCurrent = follower1.getSupplyCurrent();
    follower1StatorCurrent = follower1.getStatorCurrent();
    follower1Temp = follower1.getDeviceTemp();

    // Cache signal references once — FOLLOWER 2
    follower2Velocity = follower2.getVelocity();
    follower2MotorVoltage = follower2.getMotorVoltage();
    follower2SupplyCurrent = follower2.getSupplyCurrent();
    follower2StatorCurrent = follower2.getStatorCurrent();
    follower2Temp = follower2.getDeviceTemp();

    // Cache signal references once — FOLLOWER 3
    follower3Velocity = follower3.getVelocity();
    follower3MotorVoltage = follower3.getMotorVoltage();
    follower3SupplyCurrent = follower3.getSupplyCurrent();
    follower3StatorCurrent = follower3.getStatorCurrent();
    follower3Temp = follower3.getDeviceTemp();

    // Cache signal references once — FOLLOWER 4
    follower4Velocity = follower4.getVelocity();
    follower4MotorVoltage = follower4.getMotorVoltage();
    follower4SupplyCurrent = follower4.getSupplyCurrent();
    follower4StatorCurrent = follower4.getStatorCurrent();
    follower4Temp = follower4.getDeviceTemp();

    // 50Hz for signals we need every loop (velocity, voltage, current, closed-loop reference)
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderVelocity,
        leaderMotorVoltage,
        leaderSupplyCurrent,
        leaderStatorCurrent,
        leaderPos,
        closedLoopReference,
        follower1Velocity,
        follower1MotorVoltage,
        follower1SupplyCurrent,
        follower1StatorCurrent,
        follower2Velocity,
        follower2MotorVoltage,
        follower2SupplyCurrent,
        follower2StatorCurrent,
        follower3Velocity,
        follower3MotorVoltage,
        follower3SupplyCurrent,
        follower3StatorCurrent,
        follower4Velocity,
        follower4MotorVoltage,
        follower4SupplyCurrent,
        follower4StatorCurrent);

    // 10Hz for diagnostic-only signals (temperature, closed-loop error)
    BaseStatusSignal.setUpdateFrequencyForAll(
        10.0,
        leaderTemp,
        closedLoopError,
        follower1Temp,
        follower2Temp,
        follower3Temp,
        follower4Temp);

    // Stop sending signals we didn't register — reduces CAN bus traffic
    leader.optimizeBusUtilization();
    follower1.optimizeBusUtilization();
    follower2.optimizeBusUtilization();
    follower3.optimizeBusUtilization();
    follower4.optimizeBusUtilization();
  }

  private void configureMotors() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted =
        FlywheelConstants.Mechanical.INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = FlywheelConstants.Mechanical.flywheelRatio;

    config.Slot0.kS = FlywheelConstants.TorqueControl.KS;
    config.Slot0.kV = FlywheelConstants.TorqueControl.KV;
    config.Slot0.kP = FlywheelConstants.TorqueControl.KP;
    config.Slot0.kD = FlywheelConstants.TorqueControl.KD;

    var flywheelMotionMagic = config.MotionMagic;
    flywheelMotionMagic.MotionMagicAcceleration =
        FlywheelConstants.flywheelMagicConstants.flywheelAccel;

    // Current limits
    var limits = new CurrentLimitsConfigs();
    limits.SupplyCurrentLimit = FlywheelConstants.CurrentLimits.SHOOTER_MAIN_SUPPLY_AMP;
    limits.SupplyCurrentLimitEnable = false;
    limits.SupplyCurrentLowerLimit =
        FlywheelConstants.CurrentLimits.SHOOTER_MAIN_SUPPLY_TRIGGER_AMP;
    limits.SupplyCurrentLowerTime =
        FlywheelConstants.CurrentLimits.SHOOTER_MAIN_SUPPLY_TRIGGER_TIME_SEC.in(Seconds);
    limits.StatorCurrentLimit = FlywheelConstants.CurrentLimits.SHOOTER_MAIN_STATOR_AMP;
    limits.StatorCurrentLimitEnable = false;

    leader.getConfigurator().apply(config);
    leader.getConfigurator().apply(limits);
    follower1.getConfigurator().apply(config);
    follower1.getConfigurator().apply(limits);
    follower2.getConfigurator().apply(config);
    follower2.getConfigurator().apply(limits);
    follower3.getConfigurator().apply(config);
    follower3.getConfigurator().apply(limits);
    follower4.getConfigurator().apply(config);
    follower4.getConfigurator().apply(limits);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // One batched CAN read for ALL signals on ALL motors
    BaseStatusSignal.refreshAll(
        leaderVelocity,
        leaderMotorVoltage,
        leaderSupplyCurrent,
        leaderStatorCurrent,
        leaderTemp,
        leaderPos,
        closedLoopReference,
        closedLoopError,
        follower1Velocity,
        follower1MotorVoltage,
        follower1SupplyCurrent,
        follower1StatorCurrent,
        follower1Temp,
        follower2Velocity,
        follower2MotorVoltage,
        follower2SupplyCurrent,
        follower2StatorCurrent,
        follower2Temp,
        follower3Velocity,
        follower3MotorVoltage,
        follower3SupplyCurrent,
        follower3StatorCurrent,
        follower3Temp,
        follower4Velocity,
        follower4MotorVoltage,
        follower4SupplyCurrent,
        follower4StatorCurrent,
        follower4Temp);

    // Leader motor — read from cache
    inputs.leaderVelocity = leaderVelocity.getValue();
    inputs.leaderAppliedVolts = leaderMotorVoltage.getValue();
    inputs.leaderSupplyCurrentAmps = leaderSupplyCurrent.getValue();
    inputs.leaderStatorCurrentAmps = leaderStatorCurrent.getValue();
    inputs.leaderTemp = leaderTemp.getValue();
    inputs.leaderAngle = leaderPos.getValue();

    // Follower 1 motor — read from cache
    inputs.follower1Velocity = follower1Velocity.getValue();
    inputs.follower1AppliedVolts = follower1MotorVoltage.getValue();
    inputs.follower1SupplyCurrentAmps = follower1SupplyCurrent.getValue();
    inputs.follower1StatorCurrentAmps = follower1StatorCurrent.getValue();
    inputs.follower1Temp = follower1Temp.getValue();

    // Follower 2 motor — read from cache
    inputs.follower2Velocity = follower2Velocity.getValue();
    inputs.follower2AppliedVolts = follower2MotorVoltage.getValue();
    inputs.follower2SupplyCurrentAmps = follower2SupplyCurrent.getValue();
    inputs.follower2StatorCurrentAmps = follower2StatorCurrent.getValue();
    inputs.follower2Temp = follower2Temp.getValue();

    // Follower 3 motor — read from cache
    inputs.follower3Velocity = follower3Velocity.getValue();
    inputs.follower3AppliedVolts = follower3MotorVoltage.getValue();
    inputs.follower3SupplyCurrentAmps = follower3SupplyCurrent.getValue();
    inputs.follower3StatorCurrentAmps = follower3StatorCurrent.getValue();
    inputs.follower3Temp = follower3Temp.getValue();

    // Follower 4 motor — read from cache
    inputs.follower4Velocity = follower4Velocity.getValue();
    inputs.follower4AppliedVolts = follower4MotorVoltage.getValue();
    inputs.follower4SupplyCurrentAmps = follower4SupplyCurrent.getValue();
    inputs.follower4StatorCurrentAmps = follower4StatorCurrent.getValue();
    inputs.follower4Temp = follower4Temp.getValue();

    // Combined flywheel velocity (use leader velocity as representative)
    inputs.flywheelVelocity = inputs.leaderVelocity;

    // Setpoint and error
    inputs.closedLoopError = RotationsPerSecond.of(closedLoopError.getValueAsDouble());
    inputs.closedLoopReference = RotationsPerSecond.of(closedLoopReference.getValueAsDouble());
  }

  @Override
  public void setFlywheelVoltage(Voltage volts) {
    leader.setControl(voltageRequest.withOutput(volts.in(Volts)));
    // Logger.recordOutput("RobotState/Leader connected", leader.isConnected());
    // Logger.recordOutput("RobotState/Follower1 connected", follower1.isConnected());
    // Logger.recordOutput("RobotState/Follower2 connected", follower2.isConnected());
    // Logger.recordOutput("RobotState/Follower3 connected", follower3.isConnected());
    // Logger.recordOutput("RobotState/Folloer4 connected", follower4.isConnected());
  }

  @Override
  public void setFlywheelVelocity(AngularVelocity velocity) {
    // PID gains are already configured in configureMotors() — no need to re-apply config each call
    leader.setControl(velocityTorqueCurrentRequest.withVelocity(velocity));
    Logger.recordOutput("Flywheel running", velocity);
  }
}
