package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;

/**
 * Constants for the intake pivot mechanism.
 *
 * <p>Organized into inner classes for current limits, software settings, PID gains, mechanical
 * ratios, MotionMagic profiles, simulation parameters, and visualization geometry.
 */
public class IntakePivotConstants {

  /** Current limits for the pivot motor. */
  public static class CurrentLimits {
    public static final int INTAKE_PIVOT_MAIN_SUPPLY_AMP = 40;
    public static final int INTAKE_PIVOT_MAIN_SUPPLY_TRIGGER_AMP = 35;
    public static final Time INTAKE_PIVOT_MAIN_SUPPLY_TRIGGER_TIME_SEC = Seconds.of(1);
    public static final int INTAKE_PIVOT_MAIN_STATOR_AMP = 70;
  }

  /** Software configuration: motor inversion, encoder direction, and soft limits. */
  public static class SoftwareConstants {
    public static final boolean MOTOR_INVERTED = false;
    public static final SensorDirectionValue ENCODER_DIRECTION =
        SensorDirectionValue.Clockwise_Positive;
    /** Upper software limit in rotations of the mechanism. */
    public static final double softwareUpperRotationLimit = 0.25;
    /** Lower software limit in rotations of the mechanism. */
    public static final double softwareLowerRotationLimit = 0.0;
  }

  /** PID gains for the real robot's position controller (NOT tuned yet — placeholder values). */
  public static class PID {
    public static final double KG = 10.0;
    public static final double KV = 0.0;
    public static final double KP = 200;
    public static final double KI = 0.0;
    public static final double KD = 20;
  }

  /** Mechanical properties of the pivot. */
  public static class Mechanical {
    /** Total gear ratio from motor to pivot output. */
    public static final double pivotRatio = 45;
    /** Current threshold (stator amps) used in the jostle routine. */
    public static final double pivotJostleCurrentLimit = 70;
    /** CANcoder magnet offset in rotations. */
    public static final double magnetOffset = 0.35;
    /** CANcoder absolute sensor discontinuity point. */
    public static final double magnetSensorDiscontinuityPoint = 0.625;

    public static final double pivotJostleDegreesUp = 0.25;
    public static final double pivotDegreesDown = 0.0;
  }

  /** MotionMagic profile constraints. */
  public static class PivotMagicConstants {
    /** MotionMagic acceleration in rotations/s². */
    public static final double pivotAccel = 5.0;
    /** MotionMagic cruise velocity in rotations/s. */
    public static final double pivotVelo = 1;
  }

  /** Simulation constants for the intake pivot motor and arm physics. */
  public static class Sim {
    /** One Kraken X60 FOC motor drives the pivot. */
    public static final DCMotor PIVOT_MOTOR = DCMotor.getKrakenX60Foc(1);

    /** Moment of inertia of the pivot arm (kg·m²). Approximate value — tune in sim. */
    public static final double PIVOT_MOI = 0.01;

    /** Length of the pivot arm in meters (used by SingleJointedArmSim). */
    public static final double ARM_LENGTH_METERS = 0.3;

    /** Minimum angle the pivot can reach (radians). 0 = horizontal/stowed. */
    public static final double MIN_ANGLE_RAD =
        Units.rotationsToRadians(SoftwareConstants.softwareLowerRotationLimit);

    /** Maximum angle the pivot can reach (radians). */
    public static final double MAX_ANGLE_RAD =
        Units.rotationsToRadians(SoftwareConstants.softwareUpperRotationLimit);

    /** Whether to simulate the effect of gravity on the arm. */
    public static final boolean SIMULATE_GRAVITY = true;

    /** Starting angle of the arm in radians (at the retracted/stowed position). */
    public static final double STARTING_ANGLE_RAD = MIN_ANGLE_RAD;

    // Sim PID gains for TalonFX closed-loop in simulation
    public static final double KS = 0.0;
    public static final double KV = 0.12;
    public static final double KP = 1.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
  }

  /** Visualization constants for the Mechanism2d and 3D pose output. */
  public static class Visualization {
    /** Length of the pivot arm for Mechanism2d display (meters). */
    public static final double ARM_LENGTH_DISPLAY = 0.3;

    /**
     * Offset from the robot origin to the pivot joint in the robot coordinate frame (X = forward, Y
     * = left, Z = up).
     */
    public static final Translation3d PIVOT_BASE_OFFSET = new Translation3d(-0.2, 0.0, 0.15);

    /**
     * Tolerance in rotations — when the measured position is this close to goal, "atGoal" = true.
     */
    public static final double POSITION_TOLERANCE_ROTATIONS = 0.005;
  }
}
