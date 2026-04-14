package frc.robot.subsystems.flywheel;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants;

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
    public static final AngularAcceleration MAX_ACCEL = RevolutionsPerSecond.per(Second).of(160.0);
    public static final AngularVelocity velocityThreshold = RPM.of(500);
    public static final AngularVelocity MAX_VELOCITY = RPM.of(185);
  }

  /** Current limits for the flywheel motors. */
  public static class CurrentLimits {
    // Main flywheel (4x TalonFX)
    public static final int SHOOTER_MAIN_SUPPLY_AMP = 40;
    public static final int SHOOTER_MAIN_SUPPLY_TRIGGER_AMP = 35;
    public static final Time SHOOTER_MAIN_SUPPLY_TRIGGER_TIME_SEC = Seconds.of(1);
    public static final int SHOOTER_MAIN_STATOR_AMP = 60;
  }

  /**
   * Torque-current PID/FF gains used on the real robot.
   *
   * <p>These are tuned for MotionMagicVelocityTorqueCurrentFOC on real hardware.
   */
  public static class TorqueControl {
    // private static LoggedNetworkNumber tunableKS = new LoggedNetworkNumber("Tune/flywheel/KS",
    // 20.0);
    // private static LoggedNetworkNumber tunableKV = new LoggedNetworkNumber("Tune/flywheel/KV",
    // 0.12);
    // private static LoggedNetworkNumber tunableKP = new LoggedNetworkNumber("Tune/flywheel/KP",
    // 30.0);

    // public static double KS = tunableKS.get();
    // public static double KV = tunableKV.get();
    // public static double KP = tunableKP.get();

    // For MotionMagicVelocityTorqueCurrentFOC (torque-current output):
    //   kS = Amps to overcome static friction
    //   kV = Amps per rps of target velocity (feedforward)
    //   kP = Amps per rps of velocity error (feedback)
    //   kD = Amps per rps/s of velocity error derivative
    //
    // With kV=0 and kP=0 the motor only applies static friction amps — it never
    // actually drives toward the velocity setpoint. Start with small values and
    // tune up from there.
    public static double KS = 10.0; // 4.5
    public static double KV = 0.12; // feedforward: Amps per rps — start ~0.12, tune to taste
    public static double KP = 7.0; // feedback: Amps per rps of error — start ~2.0, tune to taste
    public static double KD = 0; // 0
  }

  public static class flywheelMagicConstants {
    public static final double flywheelAccel = 100;
  }

  /**
   * Feedforward constants for the main flywheel. Used by SimpleMotorFeedforward (kS, kV). Prestage
   * and Kicker use TalonFX Slot0 closed-loop; main flywheel uses feedforward-only. In progress:
   * adding combined feedforward and feedback; adding motion magic
   */
  public static class PID {
    // Main flywheel (4x TalonFX) - feedforward only
    public static final double MAIN_KS = 0.35;
    public static final double MAIN_KV = 0.0;
    public static final double kP = 0.0;
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
    public static final boolean INVERTED = true;

    public static final boolean FOLLOWER_INVERTED = true;

    public static final double flywheelRatio =
        36.0 / 24.0; // Sensor rotations to mechanism rotations
    public static final double flywheelMetersPerRotation = inchesToMeters(Math.PI * 4);
    public static final double flywheelRotationsPerMeter = 1 / flywheelMetersPerRotation;
  }

  /**
   * Distance-to-speed lookup table for automatic shooting.
   *
   * <p>Maps distance to target (meters) -> required shooter speed (RPM). Characterize by shooting
   * from various distances and recording the RPM needed.
   */
  public static class DistanceMap {
    public static final InterpolatingDoubleTreeMap SPEED_MAP = new InterpolatingDoubleTreeMap();

    // Key is distance from center of hub (converted to meters)
    // Value is angular velocity (RPM)
    static {
      // SPEED_MAP.put(inchesToMeters(70), 1700.0);
      // SPEED_MAP.put(inchesToMeters(83), 1950.0);
      // SPEED_MAP.put(inchesToMeters(90), 1950.0);
      // SPEED_MAP.put(inchesToMeters(100), 2000.0);
      // SPEED_MAP.put(inchesToMeters(120), 2100.0);
      // SPEED_MAP.put(inchesToMeters(130), 2500.0);
      // SPEED_MAP.put(inchesToMeters(150), 2500.0);
      // SPEED_MAP.put(inchesToMeters(163), 2800.0);
      // SPEED_MAP.put(inchesToMeters(205), 2800.0);
      SPEED_MAP.put(inchesToMeters(126.0), 1600.0);
      SPEED_MAP.put(inchesToMeters(81.0), 1500.0);
    }
  }

  /** Simulation constants for the flywheel. */
  public static class Sim {
    /** Five Kraken X60 FOC motors drive the flywheel (1 leader + 4 followers). */
    public static final DCMotor FLYWHEEL_MOTOR = DCMotor.getKrakenX60Foc(5);

    /** Number of motors driving the flywheel. */
    public static final int NUM_MOTORS = 5;

    /** Moment of inertia of the flywheel (kg·m²). Larger than small rollers due to mass. */
    public static final double FLYWHEEL_MOI = 0.01;

    // Sim-specific torque-current PID gains.
    // The TalonFX firmware still runs TorqueCurrentFOC in sim and produces
    // a motor voltage output that we feed into DCMotorSim.
    // These gains are tuned for the physics model, not real hardware.
    public static final double KS = 4.0;
    public static final double KV = 0.005;
    public static final double KP = 8.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
  }

  /** Constants for the trajectory visualization in AdvantageScope. */
  public static class TrajectoryVisualization {
    /** Number of sample points along the trajectory arc. More = smoother curve. */
    public static final int TRAJECTORY_POINTS = 50;

    /** Total time span (seconds) to project the trajectory forward. */
    public static final double TRAJECTORY_TIME_SPAN = 2.0;

    /** Height above the ground where fuel leaves the shooter (meters). */
    public static final double LAUNCH_HEIGHT_METERS = inchesToMeters(20);

    /** Drum radius used to convert flywheel angular velocity to linear launch velocity. */
    public static final double DRUM_RADIUS_METERS = inchesToMeters(1.5);

    /**
     * Fudge factor applied to the calculated launch velocity. Tune this to make the trajectory
     * preview match real-world behavior. 1.0 = no adjustment.
     */
    public static final double VELOCITY_FUDGE_FACTOR = 0.8;

    /** Minimum flywheel RPM before showing a trajectory. Below this the preview is hidden. */
    public static final double MIN_RPM_FOR_TRAJECTORY = 50.0;

    /**
     * Offset from the robot origin to the shooter exit point in the robot frame (X = forward, Y =
     * left, Z = up). Adjust to match your actual shooter placement on the robot.
     */
    public static final double SHOOTER_EXIT_X_METERS = inchesToMeters(-6); // behind center

    public static final double SHOOTER_EXIT_Y_METERS = 0.0; // centered left-right
  }

  /**
   * Helper to get the correct Slot0 gains for the current robot mode.
   *
   * <p>Real robot uses hardware-tuned torque-current gains; sim uses gains tuned for the physics
   * model. Both use the same MotionMagicVelocityTorqueCurrentFOC control request.
   */
  public static double getKS() {
    return Constants.currentMode == Constants.Mode.SIM ? Sim.KS : TorqueControl.KS;
  }

  public static double getKV() {
    return Constants.currentMode == Constants.Mode.SIM ? Sim.KV : TorqueControl.KV;
  }

  public static double getKP() {
    return Constants.currentMode == Constants.Mode.SIM ? Sim.KP : TorqueControl.KP;
  }

  public static double getKD() {
    return Constants.currentMode == Constants.Mode.SIM ? Sim.KD : TorqueControl.KD;
  }
}
