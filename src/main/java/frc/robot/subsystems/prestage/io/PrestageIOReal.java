package frc.robot.subsystems.prestage.io;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.prestage.PrestageConstants;

public class PrestageIOReal implements PrestageIO {

  private static final CANBus CAN_BUS = new CANBus("rio");

  private final TalonFX prestageLeft;
  private final TalonFX prestageRight;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVelocityTorqueCurrentFOC torqueRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  public PrestageIOReal() {
    prestageLeft = new TalonFX(HardwareConstants.CanIds.PRESTAGE_LEADER_ID, CAN_BUS);
    prestageRight = new TalonFX(HardwareConstants.CanIds.PRESTAGE_FOLLOWER_ID, CAN_BUS);

    // Configure motor
    configurePrestageMotor();
  }

  private void configurePrestageMotor() {
    // Set current limits, neutral mode, etc. as needed
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.MotorOutput.Inverted =
        PrestageConstants.SoftwareConstants.INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = PrestageConstants.Mechanical.prestageRatio;

    // Slot0 PID/FF gains for velocity control
    config.Slot0.kS = PrestageConstants.PID.KS;
    config.Slot0.kV = PrestageConstants.PID.KV;
    config.Slot0.kP = PrestageConstants.PID.KP;
    // config.Slot0.kI = PrestageConstants.PID.KI;
    // config.Slot0.kD = PrestageConstants.PID.KD;

    var prestageMagic = config.MotionMagic;
    prestageMagic.MotionMagicAcceleration = PrestageConstants.prestageMagicConstants.prestageAccel;

    var followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    followerConfig.MotorOutput.Inverted =
        PrestageConstants.SoftwareConstants.INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    followerConfig.Feedback.SensorToMechanismRatio = PrestageConstants.Mechanical.prestageRatio;

    // Slot0 PID/FF gains for velocity control
    followerConfig.Slot0.kS = PrestageConstants.PID.followerKS;
    followerConfig.Slot0.kV = PrestageConstants.PID.followerKV;
    followerConfig.Slot0.kP = PrestageConstants.PID.followerKP;
    // config.Slot0.kI = PrestageConstants.PID.KI;
    // config.Slot0.kD = PrestageConstants.PID.KD;

    var prestageRightMagic = followerConfig.MotionMagic;
    prestageRightMagic.MotionMagicAcceleration =
        PrestageConstants.prestageMagicConstants.prestageAccel;

    // Current limits
    var limits = new CurrentLimitsConfigs();
    limits.SupplyCurrentLimit = PrestageConstants.CurrentLimits.PRESTAGE_MAIN_SUPPLY_AMP;
    limits.SupplyCurrentLimitEnable = true;
    limits.SupplyCurrentLowerLimit =
        PrestageConstants.CurrentLimits.PRESTAGE_MAIN_SUPPLY_TRIGGER_AMP;
    limits.SupplyCurrentLowerTime =
        PrestageConstants.CurrentLimits.PRESTAGE_MAIN_SUPPLY_TRIGGER_TIME_SEC.in(Second);
    limits.StatorCurrentLimit = PrestageConstants.CurrentLimits.PRESTAGE_MAIN_STATOR_AMP;
    limits.StatorCurrentLimitEnable = true;

    prestageLeft.getConfigurator().apply(config);
    prestageRight.getConfigurator().apply(followerConfig);
    prestageLeft.getConfigurator().apply(limits);
  }

  @Override
  public void updateInputs(PrestageIOInputs inputs) {
    // Read sensor values and populate inputs object
    inputs.prestageLeftVelocity =
        RotationsPerSecond.of(prestageLeft.getVelocity().getValueAsDouble());
    inputs.prestageRightVelocity =
        RotationsPerSecond.of(prestageRight.getVelocity().getValueAsDouble());
    inputs.prestageLeftStatorAmps = prestageLeft.getStatorCurrent().getValue();
    inputs.prestageLeftSupplyAmps = prestageLeft.getSupplyCurrent().getValue();
    inputs.prestageLeftVoltage = prestageLeft.getMotorVoltage().getValue();
    inputs.prestageLeftTemperature = prestageLeft.getDeviceTemp().getValue();
    inputs.prestageLeftClosedLoopReference =
        RotationsPerSecond.of(prestageRight.getClosedLoopReference().getValueAsDouble());
    inputs.prestageLeftClosedLoopError =
        RotationsPerSecond.of(prestageRight.getClosedLoopError().getValueAsDouble());
    inputs.prestageRightStatorAmps = prestageLeft.getStatorCurrent().getValue();
    inputs.prestageRightSupplyAmps = prestageLeft.getSupplyCurrent().getValue();
    inputs.prestageRightVoltage = prestageLeft.getMotorVoltage().getValue();
    inputs.prestageRightTemperature = prestageLeft.getDeviceTemp().getValue();
    inputs.prestageRightClosedLoopReference =
        RotationsPerSecond.of(prestageRight.getClosedLoopReference().getValueAsDouble());
    inputs.prestageRightClosedLoopError =
        RotationsPerSecond.of(prestageRight.getClosedLoopError().getValueAsDouble());
  }

  public void setPrestageVoltage(Voltage volts) {
    prestageLeft.setControl(voltageRequest.withOutput(volts));
    prestageRight.setControl(voltageRequest.withOutput(volts));
  }

  public void setPrestageVelocity(AngularVelocity prestageVelo) {
    prestageLeft.setControl(torqueRequest.withVelocity(prestageVelo));
    prestageRight.setControl(torqueRequest.withVelocity(prestageVelo));
  }

  public void setOneVelo(AngularVelocity leaderVelo) {
    prestageRight.setControl(torqueRequest.withVelocity(leaderVelo));
  }
}
