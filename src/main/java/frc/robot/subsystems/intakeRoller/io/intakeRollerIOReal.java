package frc.robot.subsystems.intakeRoller.io;

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
import frc.robot.subsystems.intakeRoller.intakeRollerConstants;
import frc.robot.subsystems.intakeRoller.intakeRollerConstants.rollerMagicConstants;

public class intakeRollerIOReal implements intakeRollerIO {

  private static final CANBus CAN_BUS = new CANBus("rio");

  private final TalonFX intakeRollerLeader;
  private final TalonFX intakeRollerFollower;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVelocityTorqueCurrentFOC torqueRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  public intakeRollerIOReal() {
    intakeRollerLeader = new TalonFX(HardwareConstants.CanIds.INTAKE_ROLLER_LEADER_ID, CAN_BUS);
    intakeRollerFollower = new TalonFX(HardwareConstants.CanIds.INTAKE_ROLLER_FOLLOWER_ID, CAN_BUS);

    intakeRollerFollower.setControl(
        new Follower(
            HardwareConstants.CanIds.INTAKE_ROLLER_LEADER_ID, MotorAlignmentValue.Aligned));

    // Configure motor
    configureintakeRollerMotor();
  }

  private void configureintakeRollerMotor() {
    // Set current limits, neutral mode, etc. as needed
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.MotorOutput.Inverted =
        intakeRollerConstants.SoftwareConstants.INVERTED
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

    var rollerMagic = config.MotionMagic;
    rollerMagic.MotionMagicAcceleration = rollerMagicConstants.rollerAccel;

    // Slot0 PID/FF gains for velocity control
    config.Slot0.kS = intakeRollerConstants.PID.KS;
    config.Slot0.kV = intakeRollerConstants.PID.KV;
    config.Slot0.kP = intakeRollerConstants.PID.KP;
    config.Slot0.kI = intakeRollerConstants.PID.KI;
    config.Slot0.kD = intakeRollerConstants.PID.KD;

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
  }

  @Override
  public void updateInputs(intakeRollerIOInputs inputs) {
    // Read sensor values and populate inputs object
    inputs.intakeRollerVelocity = intakeRollerLeader.getVelocity().getValue();
    inputs.intakeRollerStatorCurrent = intakeRollerLeader.getStatorCurrent().getValue();
    inputs.intakeRollerSupplyCurrent = intakeRollerLeader.getSupplyCurrent().getValue();
    inputs.intakeRollerVoltage = intakeRollerLeader.getMotorVoltage().getValue();
    inputs.intakeRollerTemperature = intakeRollerLeader.getDeviceTemp().getValue();
  }

  public void setIntakeRollerVoltage(Voltage volts) {
    intakeRollerLeader.setControl(voltageRequest.withOutput(volts));
  }

  public void setIntakeRollerSpeed(AngularVelocity speed) {
    intakeRollerLeader.setControl(velocityRequest.withVelocity(speed));
  }

  public void setRollerTorqueControl(AngularVelocity rollerVelo) {
    intakeRollerLeader.setControl(torqueRequest.withVelocity(rollerVelo));
  }
}
