package frc.robot.subsystems.flywheel.io;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.flywheel.FlywheelConstants;

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

  public FlywheelIOPhoenix6() {
    leader = new TalonFX(FlywheelConstants.CANIDs.MAIN_FLYWHEEL_LEADER_ID, CAN_BUS);
    follower1 = new TalonFX(FlywheelConstants.CANIDs.MAIN_FLYWHEEL_FOLLOWER1_ID, CAN_BUS);
    follower2 = new TalonFX(FlywheelConstants.CANIDs.MAIN_FLYWHEEL_FOLLOWER2_ID, CAN_BUS);
    follower3 = new TalonFX(FlywheelConstants.CANIDs.MAIN_FLYWHEEL_FOLLOWER3_ID, CAN_BUS);

    // Configure followers
    int leaderId = leader.getDeviceID();
    follower1.setControl(new Follower(leaderId, MotorAlignmentValue.Opposed));
    follower2.setControl(new Follower(leaderId, MotorAlignmentValue.Opposed));
    follower3.setControl(new Follower(leaderId, MotorAlignmentValue.Aligned));

    configureLeader();
    configureFollowers();
  }

  private void configureLeader() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted =
        FlywheelConstants.Mechanical.INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

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
  }

  private void configureFollowers() {
    var coastConfig = new MotorOutputConfigs();
    coastConfig.NeutralMode = NeutralModeValue.Coast;

    var followerLimits = new CurrentLimitsConfigs();
    followerLimits.StatorCurrentLimit = FlywheelConstants.CurrentLimits.SHOOTER_MAIN_STATOR_AMP;
    followerLimits.StatorCurrentLimitEnable = true;

    for (TalonFX f : new TalonFX[] {follower1, follower2, follower3}) {
      f.getConfigurator().apply(coastConfig);
      f.getConfigurator().apply(followerLimits);
    }
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Leader motor
    inputs.leaderVelocity = RevolutionsPerSecond.of(leader.getVelocity().getValueAsDouble());
    inputs.leaderAppliedVolts = leader.getMotorVoltage().getValueAsDouble();
    inputs.leaderSupplyCurrentAmps = leader.getSupplyCurrent().getValueAsDouble();
    inputs.leaderStatorCurrentAmps = leader.getStatorCurrent().getValueAsDouble();

    // Follower 1 motor
    inputs.follower1Velocity = RevolutionsPerSecond.of(follower1.getVelocity().getValueAsDouble());
    inputs.follower1AppliedVolts = follower1.getMotorVoltage().getValueAsDouble();
    inputs.follower1SupplyCurrentAmps = follower1.getSupplyCurrent().getValueAsDouble();
    inputs.follower1StatorCurrentAmps = follower1.getStatorCurrent().getValueAsDouble();

    // Follower 2 motor
    inputs.follower2Velocity = RevolutionsPerSecond.of(follower2.getVelocity().getValueAsDouble());
    inputs.follower2AppliedVolts = follower2.getMotorVoltage().getValueAsDouble();
    inputs.follower2SupplyCurrentAmps = follower2.getSupplyCurrent().getValueAsDouble();
    inputs.follower2StatorCurrentAmps = follower2.getStatorCurrent().getValueAsDouble();

    // Follower 3 motor
    inputs.follower3Velocity = RevolutionsPerSecond.of(follower3.getVelocity().getValueAsDouble());
    inputs.follower3AppliedVolts = follower3.getMotorVoltage().getValueAsDouble();
    inputs.follower3SupplyCurrentAmps = follower3.getSupplyCurrent().getValueAsDouble();
    inputs.follower3StatorCurrentAmps = follower3.getStatorCurrent().getValueAsDouble();

    // Combined flywheel velocity (use leader velocity as representative)
    inputs.flywheelVelocity = inputs.leaderVelocity;
  }

  @Override
  public void setFlywheelDutyCycle(double output) {
    leader.setControl(dutyCycleRequest.withOutput(output));
  }

  @Override
  public void stopFlywheel() {
    leader.setControl(dutyCycleRequest.withOutput(0));
  }

  @Override
  public void setFlywheelVoltage(Voltage volts) {
    leader.setControl(voltageRequest.withOutput(volts.in(Volts)));
  }
}
