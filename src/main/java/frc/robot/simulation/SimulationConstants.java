package frc.robot.simulation;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

/**
 * Physical constants describing the robot to the MapleSim rigid-body physics engine.
 *
 * <p>These are <b>simulation-only</b>. Nothing here is read on a real robot, so changing a value in
 * this file cannot affect match behavior — it only changes what the sim predicts.
 *
 * <p>Several values intentionally mirror numbers that already live elsewhere in the codebase. They
 * are duplicated rather than referenced because the originals are {@code private}. If you change
 * one of the originals, change it here too:
 *
 * <ul>
 *   <li>{@link #ROBOT_MASS}, {@link #WHEEL_COF} — mirror {@code Drive.ROBOT_MASS_KG} and {@code
 *       Drive.WHEEL_COF} (the PathPlanner {@code RobotConfig})
 *   <li>Module gear ratios / wheel radius — mirror {@code COMP_TunerConstants}
 * </ul>
 */
public final class SimulationConstants {

  private SimulationConstants() {}

  // ─── Chassis ────────────────────────────────────────────────────────────────

  /** Mirrors {@code Drive.ROBOT_MASS_KG}. Drives acceleration and collision response. */
  public static final Mass ROBOT_MASS = Kilograms.of(63.503);

  /**
   * Bumper footprint — measured on the physical robot. MapleSim uses this rectangle as the robot's
   * collision body, so it determines when the robot hits field walls and other robots.
   */
  public static final Distance BUMPER_LENGTH_X = Inches.of(33.4);

  public static final Distance BUMPER_WIDTH_Y = Inches.of(33.4);

  /** Mirrors {@code Drive.WHEEL_COF}. Sets the traction limit before the wheels slip. */
  public static final double WHEEL_COF = 2.225;

  // ─── Swerve module ──────────────────────────────────────────────────────────
  // Values mirror COMP_TunerConstants. Motor choices match the existing ModuleIOSim.

  public static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60Foc(1);
  public static final DCMotor STEER_MOTOR = DCMotor.getKrakenX44Foc(1);
  public static final double DRIVE_GEAR_RATIO = 7.03125;
  public static final double STEER_GEAR_RATIO = 26.09090909090909;
  public static final Distance WHEEL_RADIUS = Inches.of(2);
  public static final MomentOfInertia STEER_MOI = KilogramSquareMeters.of(0.01);
  public static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);
  public static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.2);

  /** Mirrors {@code kSlipCurrent}. Caps the propelling force MapleSim applies per module. */
  public static final Current DRIVE_CURRENT_LIMIT = Amps.of(80);

  // ─── Intake ─────────────────────────────────────────────────────────────────

  /**
   * Width of the intake across the front face, and how far it reaches past the front bumper when
   * deployed. Together these define the rectangle that grabs Fuel on contact.
   */
  public static final Distance INTAKE_WIDTH = Inches.of(25.5);

  public static final Distance INTAKE_EXTENSION = Inches.of(8.5);

  /**
   * How many Fuel the robot can hold before the intake stops accepting more.
   *
   * <p>ASSUMPTION — not derived from anything in the codebase. Adjust to match the real hopper.
   */
  public static final int INTAKE_CAPACITY = 24;

  /**
   * The intake only grabs Fuel when the pivot is at or below this angle. The pivot reads 0 when
   * deployed and rises toward {@code softwareUpperRotationLimit} (0.4 rot) when stowed.
   */
  public static final Angle PIVOT_DEPLOYED_THRESHOLD = Rotations.of(0.1);

  /** The intake only grabs Fuel when the rollers are spinning at least this fast. */
  public static final AngularVelocity ROLLER_RUNNING_THRESHOLD = RPM.of(200);

  // ─── Shooting ───────────────────────────────────────────────────────────────

  /**
   * A shot is simulated only when the upper feeder is moving at least this fast — the feeder is
   * what physically pushes Fuel into the flywheel.
   *
   * <p>Compared against absolute value: {@code feederVelocity} is negative (-3000 RPM) on this
   * robot.
   */
  public static final AngularVelocity FEEDER_FEEDING_THRESHOLD = RPM.of(500);

  /**
   * Minimum time between simulated shots while the feeder is running — the robot's cycle rate.
   *
   * <p>ASSUMPTION — tune this until the sim's shots-per-second matches match video.
   */
  public static final double SHOT_INTERVAL_SECONDS = 0.15;

  /** Below this flywheel speed no projectile is launched, matching the trajectory preview. */
  public static final AngularVelocity MIN_SHOT_VELOCITY = RPM.of(300);

  // ─── Field ──────────────────────────────────────────────────────────────────

  /**
   * Where the simulated robot is placed on startup and whenever the sim field is reset. Roughly the
   * blue-alliance side of the field; the auto chooser overwrites this via {@code Drive.setPose()}.
   */
  public static final Pose2d DEFAULT_STARTING_POSE = new Pose2d(3.0, 3.0, Rotation2d.kZero);

  /** NetworkTables/AdvantageKit key prefix for everything this package logs. */
  public static final String LOG_ROOT = "MapleSim";
}
