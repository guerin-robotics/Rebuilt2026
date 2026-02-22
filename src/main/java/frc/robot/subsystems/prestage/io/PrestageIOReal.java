package frc.robot.subsystems.prestage.io;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.prestage.PrestageConstants;

public class PrestageIOReal implements PrestageIO {

  private static final CANBus CAN_BUS = new CANBus("rio");

  private final TalonFX prestageLeader;
  private final TalonFX prestageFollower;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVelocityTorqueCurrentFOC torqueRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  public PrestageIOReal() {
    prestageLeader = new TalonFX(HardwareConstants.CanIds.PRESTAGE_LEADER_ID, CAN_BUS);
    prestageFollower = new TalonFX(HardwareConstants.CanIds.PRESTAGE_FOLLOWER_ID, CAN_BUS);

    prestageFollower.setControl(
        new Follower(HardwareConstants.CanIds.PRESTAGE_LEADER_ID, MotorAlignmentValue.Aligned));

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

    // Slot0 PID/FF gains for velocity control
    config.Slot0.kS = PrestageConstants.PID.KS;
    config.Slot0.kV = PrestageConstants.PID.KV;
    config.Slot0.kP = PrestageConstants.PID.KP;
    // config.Slot0.kI = PrestageConstants.PID.KI;
    // config.Slot0.kD = PrestageConstants.PID.KD;

    var prestageMagic = config.MotionMagic;
    prestageMagic.MotionMagicAcceleration = PrestageConstants.prestageMagicConstants.prestageAccel;

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

    prestageLeader.getConfigurator().apply(config);
    prestageLeader.getConfigurator().apply(limits);
  }

  @Override
  public void updateInputs(PrestageIOInputs inputs) {
    // Read sensor values and populate inputs object
    inputs.prestageMotorVelocity =
        RotationsPerSecond.of(prestageLeader.getVelocity().getValueAsDouble());
    inputs.prestageStatorAmps = prestageLeader.getStatorCurrent().getValue();
    inputs.prestageSupplyAmps = prestageLeader.getSupplyCurrent().getValue();
    inputs.prestageVoltage = prestageLeader.getMotorVoltage().getValue();
    inputs.prestageMotorTemperature = prestageLeader.getDeviceTemp().getValue();
    inputs.prestageClosedLoopReference =
        RotationsPerSecond.of(prestageLeader.getClosedLoopReference().getValueAsDouble());
    inputs.prestageClosedLoopError =
        RotationsPerSecond.of(prestageLeader.getClosedLoopError().getValueAsDouble());
  }

  public void setPrestageVoltage(Voltage volts) {
    prestageLeader.setControl(voltageRequest.withOutput(volts));
  }

  public void setPrestageSpeed(AngularVelocity speed) {
    prestageLeader.setControl(velocityRequest.withVelocity(speed));
  }

  public void setPrestageTorque(AngularVelocity prestageVelo) {
    prestageLeader.setControl(torqueRequest.withVelocity(prestageVelo));
  }
}
