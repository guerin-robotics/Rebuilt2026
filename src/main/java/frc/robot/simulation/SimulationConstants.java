package frc.robot.simulation;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
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

  /** How many Fuel the robot can hold before the intake stops accepting more. */
  public static final int INTAKE_CAPACITY = 50;

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
   * Time between simulated shots while the feeder is running — 20 Fuel per second.
   *
   * <p>This is faster than the 20 ms robot loop, so {@code MapleSimWorld} fires however many shots
   * are due each loop rather than at most one. Without that, the rate would silently clamp to 50/s
   * and in practice land near 16/s from loop-boundary rounding.
   */
  public static final double SHOT_INTERVAL_SECONDS = 0.05;

  /** Safety cap on shots launched in a single loop, so a stalled clock cannot flood the field. */
  public static final int MAX_SHOTS_PER_LOOP = 10;

  /** Below this flywheel speed no projectile is launched, matching the trajectory preview. */
  public static final AngularVelocity MIN_SHOT_VELOCITY = RPM.of(300);

  // ─── Launch model ───────────────────────────────────────────────────────────

  /**
   * How far the Fuel leaves the robot from its center, measured <i>along the direction the shooter
   * fires</i>. The shooter is on the back of the robot, and it fires backward, so this is positive.
   *
   * <p><b>Sign convention — read before changing:</b> MapleSim positions a projectile as {@code
   * robotPosition + offset.rotateBy(shooterFacing)}. It rotates by the <i>shooter facing</i>, not
   * the robot heading. So this offset is expressed in the shooter's frame: positive means "out the
   * muzzle". Passing the robot-frame value (negative X, since the shooter sits at −X) puts the Fuel
   * out the <i>front</i> of the robot, which is the bug this constant replaced.
   *
   * <p>Started from the CAD shooter axis (0.2762 m) and pulled 3 in toward robot center after
   * watching where Fuel actually left the robot in simulation.
   */
  public static final Distance SHOOTER_EXIT_DISTANCE = Meters.of(0.2000);

  /**
   * Height the Fuel leaves the shooter at. Also from the CAD-exported {@code RobotModelVisualizer}
   * shooter axis.
   *
   * <p>Note this disagrees with {@code TrajectoryVisualization.LAUNCH_HEIGHT_METERS} (20 in = 0.508
   * m), as does the exit offset above (−6 in there vs −10.9 in from CAD). The CAD pair is used here
   * because the two values are self-consistent and came from the exported model; the visualizer's
   * pair appear to be hand-entered estimates. Worth reconciling on the real robot.
   */
  public static final Distance LAUNCH_HEIGHT = Meters.of(0.415);

  /**
   * Converts hood position to the Fuel's actual launch elevation above horizontal.
   *
   * <p><b>Why this exists:</b> {@code Hood.getPosition()} reports a <i>mechanism</i> angle — the
   * CANcoder reading after the 122:12 reduction. It is not the angle the ball leaves at. The hub
   * shot range asks for hood values of roughly 1°–12°, and feeding those in as launch elevation
   * produces an almost flat shot that cannot reach the Hub at any distance.
   *
   * <p><b>Where these numbers came from:</b> fitted from the robot's own characterization tables.
   * {@code FlywheelConstants.SPEED_MAP} and {@code HoodConstants.ANGLE_MAP} were tuned until shots
   * actually scored, so each (distance, RPM, hood) triple is a known-good shot. Solving projectile
   * motion for the elevation that carries Fuel from the exit point into the Hub (1.5748 m, from
   * MapleSim's {@code RebuiltHub}) gives 43°–54°, fitting this line with R² = 0.88 and a worst
   * vertical miss of 3 in.
   *
   * <p>The fit additionally requires every shot to be <i>descending</i> when it reaches the Hub. A
   * shot that arrives still rising clips the front of the goal structure instead of dropping in.
   *
   * <p><b>The fit uses MapleSim's gravity, not Earth's.</b> {@code GamePieceProjectile.GRAVITY} is
   * 11.0 m/s², not 9.81 — the library inflates gravity as a rough stand-in for the air drag it does
   * not simulate. Fitting against 9.81 lands shots 5–15 in low depending on distance, which is
   * exactly how this was found.
   *
   * <p>The negative slope is physically sensible: farther shots use more speed and a flatter arc.
   *
   * <p><b>These are derived, not measured.</b> The fit assumes a shot through the center of the
   * Hub. If you measure the real launch angle, replace them — {@code LaunchModelTest} will say
   * immediately if the new values stop scoring.
   */
  public static final double LAUNCH_ANGLE_OFFSET_DEGREES = 54.45;

  public static final double LAUNCH_ANGLE_PER_HOOD_DEGREE = -0.899;

  /**
   * Scales flywheel surface speed to Fuel exit speed (v = ω · r · factor).
   *
   * <p>{@code FlywheelConstants.TrajectoryVisualization.VELOCITY_FUDGE_FACTOR} is 0.8, which makes
   * the Fuel physically unable to reach the Hub at any angle from the mapped distances — the shot
   * falls short even on an ideal 45° arc.
   */
  public static final double LAUNCH_VELOCITY_FACTOR = 1.05;

  // ─── Field ──────────────────────────────────────────────────────────────────

  /**
   * Whether the robot can drive across the field bumps in simulation.
   *
   * <p>MapleSim's physics engine is 2D, so it cannot model driving <i>over</i> anything — every
   * obstacle is a wall of infinite height. Left in place, the bumps stop an auto dead and most
   * routines become impossible to run.
   *
   * <p>{@code true} removes them, which is wrong in the opposite direction: crossings become free,
   * with none of the real time loss, traction loss, or deflection. Trust sim autos for <i>path
   * geometry</i> across a bump, not for <i>timing</i>.
   *
   * <p>Set to {@code false} to restore the stock arena and treat the bumps as walls.
   */
  public static final boolean BUMPS_ARE_PASSABLE = true;

  /**
   * Where the simulated robot is placed on startup and whenever the sim field is reset. Roughly the
   * blue-alliance side of the field; the auto chooser overwrites this via {@code Drive.setPose()}.
   */
  public static final Pose2d DEFAULT_STARTING_POSE = new Pose2d(3.0, 3.0, Rotation2d.kZero);

  /** NetworkTables/AdvantageKit key prefix for everything this package logs. */
  public static final String LOG_ROOT = "MapleSim";
}
