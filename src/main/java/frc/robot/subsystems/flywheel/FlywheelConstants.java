package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

/**
 * Constants for the shooter subsystem.
 *
 * <p>Organized by category: mechanical, limits, PID, current limits, software tuning, distance map,
 * and FuelSim visualization.
 *
 * <p>Our robot uses CTRE TalonFX (Phoenix 6) motors, so PID/FF values here are for the TalonFX
 * Slot0 closed-loop controller (not REV SparkMax).
 */
public class FlywheelConstants {

  /** Velocity and acceleration limits for the flywheel. */
  public static class Limits {
    public static final AngularVelocity MIN_SPEED = RPM.of(100);
    public static final AngularVelocity MAX_SPEED = RPM.of(5600);
    public static final AngularAcceleration MAX_ACCEL =
        RevolutionsPerSecond.per(Second).of(5600.0 / 60.0);
  }

  /** Current limits for the flywheel motors. */
  public static class CurrentLimits {
    // Main flywheel (4x TalonFX)
    public static final int SHOOTER_MAIN_SUPPLY_AMP = 40;
    public static final int SHOOTER_MAIN_SUPPLY_TRIGGER_AMP = 35;
    public static final Time SHOOTER_MAIN_SUPPLY_TRIGGER_TIME_SEC = Seconds.of(1);
    public static final int SHOOTER_MAIN_STATOR_AMP = 60;
  }

  public static class TorqueControl {
    public static final double KS = 35;
    public static final double KV = 0.0;
    public static final double KP = 5.0;
  }

  public static class flywheelMagicConstants {
    public static final double flywheelAccel = 60;
  }

  /**
   * Feedforward constants for the main flywheel. Used by SimpleMotorFeedforward (kS, kV). Prestage
   * and Kicker use TalonFX Slot0 closed-loop; main flywheel uses feedforward-only. In progress:
   * adding combined feedforward and feedback; adding motion magic
   */
  public static class PID {
    // Main flywheel (4x TalonFX) - feedforward only
    public static final double MAIN_KS = 0.35;
    public static final double MAIN_KV = 0.12;
    public static final double kP = 0.3;
  }

  /** Software tuning settings. */
  public static class Software {
    /** Tolerance as a percentage (0.05 = 5%) for determining if flywheel is at target speed. */
    public static final double PID_TOLERANCE = 0.05;

    /** Idle duty cycle when shooter is not actively shooting. */
    public static final double IDLE_DUTY_CYCLE = 0.0;
  }

  public static class Mechanical {
    /** Hood angle for trajectory visualization (degrees). */
    public static final boolean INVERTED = false;
  }

  /**
   * Distance-to-speed lookup table for automatic shooting.
   *
   * <p>Maps distance to target (meters) -> required shooter speed (RPM). Characterize by shooting
   * from various distances and recording the RPM needed.
   */
  public static class DistanceMap {
    public static final InterpolatingDoubleTreeMap SPEED_MAP = new InterpolatingDoubleTreeMap();

    static {
      // Characterize these values on the real robot
      SPEED_MAP.put(1.0, 1000.0); // 1 meter -> 1000 RPM
      SPEED_MAP.put(3.0, 1500.0); // 3 meters -> 1500 RPM
    }
  }
}
