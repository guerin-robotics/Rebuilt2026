package frc.robot.subsystems.intakeRoller.io;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.BaseStatusSignal;
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
import frc.robot.subsystems.intakeRoller.intakeRollerConstants;
import frc.robot.subsystems.intakeRoller.intakeRollerConstants.rollerMagicConstants;

public class intakeRollerIOReal implements intakeRollerIO {

  private final TalonFX intakeRollerLeader;
  private final TalonFX intakeRollerFollower;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVelocityTorqueCurrentFOC torqueRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  // Cached status signals — created once, refreshed in batch each loop
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Temperature> deviceTemp;
  private final StatusSignal<Double> closedLoopReference;
  private final StatusSignal<Double> closedLoopError;
  private final StatusSignal<Angle> pos;
  private final StatusSignal<AngularVelocity> Followervelocity;
  private final StatusSignal<Current> FollowerstatorCurrent;
  private final StatusSignal<Current> FollowersupplyCurrent;
  private final StatusSignal<Voltage> FollowermotorVoltage;
  private final StatusSignal<Temperature> FollowerdeviceTemp;
  private final StatusSignal<Double> FollowerclosedLoopReference;
  private final StatusSignal<Double> FollowerclosedLoopError;
  private final StatusSignal<Angle> Followerpos;

  public intakeRollerIOReal() {
    intakeRollerLeader =
        new TalonFX(HardwareConstants.CanIds.INTAKE_ROLLER_LEADER_ID, HardwareConstants.CAN_BUS);
    intakeRollerFollower =
        new TalonFX(HardwareConstants.CanIds.INTAKE_ROLLER_FOLLOWER_ID, HardwareConstants.CAN_BUS);

    intakeRollerFollower.setControl(
        new Follower(
            HardwareConstants.CanIds.INTAKE_ROLLER_LEADER_ID, MotorAlignmentValue.Opposed));

    configureintakeRollerMotor();

    // Cache signal references once in the constructor
    velocity = intakeRollerLeader.getVelocity();
    statorCurrent = intakeRollerLeader.getStatorCurrent();
    supplyCurrent = intakeRollerLeader.getSupplyCurrent();
    motorVoltage = intakeRollerLeader.getMotorVoltage();
    deviceTemp = intakeRollerLeader.getDeviceTemp();
    closedLoopReference = intakeRollerLeader.getClosedLoopReference();
    closedLoopError = intakeRollerLeader.getClosedLoopError();
    pos = intakeRollerLeader.getPosition();
    Followervelocity = intakeRollerFollower.getVelocity();
    FollowerstatorCurrent = intakeRollerFollower.getStatorCurrent();
    FollowersupplyCurrent = intakeRollerFollower.getSupplyCurrent();
    FollowermotorVoltage = intakeRollerFollower.getMotorVoltage();
    FollowerdeviceTemp = intakeRollerFollower.getDeviceTemp();
    FollowerclosedLoopReference = intakeRollerFollower.getClosedLoopReference();
    FollowerclosedLoopError = intakeRollerFollower.getClosedLoopError();
    Followerpos = intakeRollerFollower.getPosition();

    // 50Hz for signals we need every loop (velocity, voltage, current, closed-loop reference)
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity, statorCurrent, supplyCurrent, motorVoltage, closedLoopReference);

    // 10Hz for diagnostic-only signals (temperature, closed-loop error)
    BaseStatusSignal.setUpdateFrequencyForAll(10.0, deviceTemp, closedLoopError);

    // Stop sending signals we didn't register — reduces CAN bus traffic
    intakeRollerLeader.optimizeBusUtilization();
    intakeRollerFollower.optimizeBusUtilization();
  }

  private void configureintakeRollerMotor() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.MotorOutput.Inverted =
        intakeRollerConstants.SoftwareConstants.INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = intakeRollerConstants.Mechanical.rollerRatio;

    var rollerMagic = config.MotionMagic;
    rollerMagic.MotionMagicAcceleration = rollerMagicConstants.rollerAccel;

    // Slot0 PID/FF gains for velocity control
    config.Slot0.kS = intakeRollerConstants.PID.KS;
    config.Slot0.kV = intakeRollerConstants.PID.KV;
    config.Slot0.kP = intakeRollerConstants.PID.KP;

    // Current limits
    var limits = new CurrentLimitsConfigs();
    limits.SupplyCurrentLimit = intakeRollerConstants.CurrentLimits.INTAKE_ROLLER_MAIN_SUPPLY_AMP;
    limits.SupplyCurrentLimitEnable = true;
    limits.SupplyCurrentLowerLimit =
        intakeRollerConstants.CurrentLimits.INTAKE_ROLLER_MAIN_SUPPLY_TRIGGER_AMP;
    limits.SupplyCurrentLowerTime =
        intakeRollerConstants.CurrentLimits.INTAKE_ROLLER_MAIN_SUPPLY_TRIGGER_TIME_SEC.in(Second);
    limits.StatorCurrentLimit = intakeRollerConstants.CurrentLimits.INTAKE_ROLLER_MAIN_STATOR_AMP;
    limits.StatorCurrentLimitEnable = true;

    intakeRollerLeader.getConfigurator().apply(config);
    intakeRollerLeader.getConfigurator().apply(limits);
    intakeRollerFollower.getConfigurator().apply(config);
    intakeRollerFollower.getConfigurator().apply(limits);
  }

  @Override
  public void updateInputs(intakeRollerIOInputs inputs) {
    // One batched CAN read for all signals — much faster than individual reads
    BaseStatusSignal.refreshAll(
        velocity,
        statorCurrent,
        supplyCurrent,
        motorVoltage,
        deviceTemp,
        closedLoopReference,
        closedLoopError,
        pos,
        Followervelocity,
        FollowerstatorCurrent,
        FollowersupplyCurrent,
        FollowermotorVoltage,
        FollowerdeviceTemp,
        FollowerclosedLoopReference,
        FollowerclosedLoopError,
        Followerpos);

    // Read from cache — no additional CAN traffic
    inputs.intakeRollerVelocity = velocity.getValue();
    inputs.intakeRollerStatorCurrent = statorCurrent.getValue();
    inputs.intakeRollerSupplyCurrent = supplyCurrent.getValue();
    inputs.intakeRollerVoltage = motorVoltage.getValue();
    inputs.intakeRollerTemperature = deviceTemp.getValue();
    inputs.rollerClosedLoopReference =
        RotationsPerSecond.of(closedLoopReference.getValueAsDouble());
    inputs.rollerClosedLoopError = RotationsPerSecond.of(closedLoopError.getValueAsDouble());
    inputs.rollerPos = pos.getValue();
    inputs.intakeRollerVelocity = Followervelocity.getValue();
    inputs.intakeRollerFollowerStatorCurrent = FollowerstatorCurrent.getValue();
    inputs.intakeRollerFollowerSupplyCurrent = FollowersupplyCurrent.getValue();
    inputs.intakeRollerFollowerVoltage = FollowermotorVoltage.getValue();
    inputs.intakeRollerFollowerTemperature = FollowerdeviceTemp.getValue();
    inputs.rollerFollowerClosedLoopReference =
        RotationsPerSecond.of(FollowerclosedLoopReference.getValueAsDouble());
    inputs.rollerFollowerClosedLoopError =
        RotationsPerSecond.of(FollowerclosedLoopError.getValueAsDouble());
    inputs.rollerFollowerPos = Followerpos.getValue();
  }

  @Override
  public void setRollerVoltage(Voltage volts) {
    intakeRollerLeader.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setRollerVelocity(AngularVelocity rollerVelo) {
    intakeRollerLeader.setControl(torqueRequest.withVelocity(rollerVelo));
  }
}
