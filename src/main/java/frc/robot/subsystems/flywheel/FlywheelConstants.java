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

  public static class CANIDs {
    public static final int MAIN_FLYWHEEL_LEADER_ID = 37;
    public static final int MAIN_FLYWHEEL_FOLLOWER1_ID = 34;
    public static final int MAIN_FLYWHEEL_FOLLOWER2_ID = 36;
    public static final int MAIN_FLYWHEEL_FOLLOWER3_ID = 35;

    public static final int PRESTAGE_LEADER_ID = 31;
    public static final int PRESTAGE_FOLLOWER_ID = 32;

    public static final int KICKER_MOTOR_ID = 33;
  }

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

    // Prestage (2x TalonFX)
    public static final int PRESTAGE_SUPPLY_AMP = 30;
    public static final int PRESTAGE_SUPPLY_TRIGGER_AMP = 25;
    public static final Time PRESTAGE_SUPPLY_TRIGGER_TIME_SEC = Seconds.of(1);
    public static final int PRESTAGE_STATOR_AMP = 40;

    // Kicker/feeder (1x TalonFX)
    public static final int KICKER_SUPPLY_AMP = 20;
    public static final int KICKER_SUPPLY_TRIGGER_AMP = 15;
    public static final Time KICKER_SUPPLY_TRIGGER_TIME_SEC = Seconds.of(1);
    public static final int KICKER_STATOR_AMP = 30;
  }

  /**
   * PID and feedforward constants for the TalonFX Slot0 controller.
   *
   * <p>These map to Phoenix 6 Slot0Configs (kS, kV, kP, kI, kD). The values below are the defaults
   * already tuned in ShooterIOPhoenix6.
   */
  public static class PID {
    // Main flywheel (4x TalonFX)
    public static final double MAIN_KS = 0.35;
    public static final double MAIN_KV = 0.12;
    public static final double MAIN_KP = 0.15;
    public static final double MAIN_KI = 0.0;
    public static final double MAIN_KD = 0.0;

    // Prestage (2x TalonFX)
    public static final double PRESTAGE_KS = 0.45;
    public static final double PRESTAGE_KV = 0.10;
    public static final double PRESTAGE_KP = 0.12;
    public static final double PRESTAGE_KI = 0.0;
    public static final double PRESTAGE_KD = 0.0;

    // Kicker/feeder (1x TalonFX)
    public static final double KICKER_KS = 0.7;
    public static final double KICKER_KV = 0.4;
    public static final double KICKER_KP = 0.1;
    public static final double KICKER_KI = 0.0;
    public static final double KICKER_KD = 0.0;
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
      // TODO: Characterize these values on the real robot
      SPEED_MAP.put(1.0, 1000.0); // 1 meter -> 1000 RPM
      SPEED_MAP.put(3.0, 1500.0); // 3 meters -> 1500 RPM
    }
  }
}
