package frc.robot.subsystems.flywheel.io;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
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
 * <p>Current limits from Constants.CurrentLimits are applied here.
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

  // Control requests (reused to avoid allocations)
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  // Feedforward controller. kV from constants is V/(rps); SimpleMotorFeedforward expects V/(rad/s).
  // Conversion: 1 rps = 2π rad/s => kV_rads = kV_rps / (2π)
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          FlywheelConstants.PID.MAIN_KS, FlywheelConstants.PID.MAIN_KV / (2 * Math.PI));
  private final MotionMagicVelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  public FlywheelIOPhoenix6() {
    leader = new TalonFX(HardwareConstants.CanIds.MAIN_FLYWHEEL_LEADER_ID, CAN_BUS);
    follower1 = new TalonFX(HardwareConstants.CanIds.MAIN_FLYWHEEL_FOLLOWER1_ID, CAN_BUS);
    follower2 = new TalonFX(HardwareConstants.CanIds.MAIN_FLYWHEEL_FOLLOWER2_ID, CAN_BUS);
    follower3 = new TalonFX(HardwareConstants.CanIds.MAIN_FLYWHEEL_FOLLOWER3_ID, CAN_BUS);

    // Configure followers
    int leaderId = leader.getDeviceID();
    follower1.setControl(new Follower(leaderId, MotorAlignmentValue.Opposed));
    follower2.setControl(new Follower(leaderId, MotorAlignmentValue.Aligned));
    follower3.setControl(new Follower(leaderId, MotorAlignmentValue.Opposed));

    configureMotors();
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

    var flywheelMotionMagic = config.MotionMagic;
    flywheelMotionMagic.MotionMagicAcceleration =
        FlywheelConstants.flywheelMagicConstants.flywheelAccel; // 60

    // Current limits
    var limits = new CurrentLimitsConfigs();
    limits.SupplyCurrentLimit = FlywheelConstants.CurrentLimits.SHOOTER_MAIN_SUPPLY_AMP;
    limits.SupplyCurrentLimitEnable = true;
    limits.SupplyCurrentLowerLimit =
        FlywheelConstants.CurrentLimits.SHOOTER_MAIN_SUPPLY_TRIGGER_AMP;
    limits.SupplyCurrentLowerTime =
        FlywheelConstants.CurrentLimits.SHOOTER_MAIN_SUPPLY_TRIGGER_TIME_SEC.in(Seconds);
    limits.StatorCurrentLimit = FlywheelConstants.CurrentLimits.SHOOTER_MAIN_STATOR_AMP;
    limits.StatorCurrentLimitEnable = true;

    leader.getConfigurator().apply(config);
    leader.getConfigurator().apply(limits);
    follower1.getConfigurator().apply(config);
    follower1.getConfigurator().apply(limits);
    follower2.getConfigurator().apply(config);
    follower2.getConfigurator().apply(limits);
    follower3.getConfigurator().apply(config);
    follower3.getConfigurator().apply(limits);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
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

    // Follower 3 motor
    inputs.follower3Velocity = follower3.getVelocity().getValue();
    inputs.follower3AppliedVolts = follower3.getMotorVoltage().getValue();
    inputs.follower3SupplyCurrentAmps = follower3.getSupplyCurrent().getValue();
    inputs.follower3StatorCurrentAmps = follower3.getStatorCurrent().getValue();

    // Combined flywheel velocity (use leader velocity as representative)
    inputs.flywheelVelocity = inputs.leaderVelocity;

    // Setpoint
    inputs.closedLoopError = RotationsPerSecond.of(leader.getClosedLoopError().getValueAsDouble());
    inputs.closedLoopReference =
        RotationsPerSecond.of(leader.getClosedLoopReference().getValueAsDouble());
  }

  public void setFlywheelDutyCycle(double output) {
    leader.setControl(dutyCycleRequest.withOutput(output));
  }

  public void setFlywheelVoltage(Voltage volts) {
    leader.setControl(voltageRequest.withOutput(volts.in(Volts)));
  }

  public void setFlywheelSpeed(AngularVelocity targetSpeed) {
    double velocityRadPerSec = targetSpeed.in(RadiansPerSecond);
    double volts = feedforward.calculate(velocityRadPerSec);
    leader.setControl(voltageRequest.withOutput(volts));
  }

  public void setFlywheelTorque(AngularVelocity velocity) {
    leader.setControl(velocityTorqueCurrentRequest.withVelocity(velocity));
    Logger.recordOutput("Flywheel running", velocity);
  }
}
