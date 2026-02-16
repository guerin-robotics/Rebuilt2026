package frc.robot.generated;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.swerve.*;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

/**
 * TunerConstants wrapper that selects between different robot configurations based on the RobotType
 * enum in Constants.java
 */
public class TunerConstants {
  // Delegate to the appropriate constants class based on robot type
  private static final boolean isAlpha = Constants.robotType == Constants.RobotType.ALPHA;
  private static final boolean isComp = Constants.robotType == Constants.RobotType.COMP;

  // CANBus configuration
  public static final CANBus kCANBus =
      isAlpha
          ? ALPHA_TunerConstants.kCANBus
          : isComp
              ? COMP_TunerConstants.kCANBus
              : ALPHA_TunerConstants.kCANBus; // Default to ALPHA if NONE

  // Speed configuration
  public static final LinearVelocity kSpeedAt12Volts =
      isAlpha
          ? ALPHA_TunerConstants.kSpeedAt12Volts
          : isComp ? COMP_TunerConstants.kSpeedAt12Volts : ALPHA_TunerConstants.kSpeedAt12Volts;

  // Drivetrain constants
  public static final SwerveDrivetrainConstants DrivetrainConstants =
      isAlpha
          ? ALPHA_TunerConstants.DrivetrainConstants
          : isComp
              ? COMP_TunerConstants.DrivetrainConstants
              : ALPHA_TunerConstants.DrivetrainConstants;

  // Module constants
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft =
          isAlpha
              ? ALPHA_TunerConstants.FrontLeft
              : isComp ? COMP_TunerConstants.FrontLeft : ALPHA_TunerConstants.FrontLeft;

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight =
          isAlpha
              ? ALPHA_TunerConstants.FrontRight
              : isComp ? COMP_TunerConstants.FrontRight : ALPHA_TunerConstants.FrontRight;

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft =
          isAlpha
              ? ALPHA_TunerConstants.BackLeft
              : isComp ? COMP_TunerConstants.BackLeft : ALPHA_TunerConstants.BackLeft;

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight =
          isAlpha
              ? ALPHA_TunerConstants.BackRight
              : isComp ? COMP_TunerConstants.BackRight : ALPHA_TunerConstants.BackRight;
}
