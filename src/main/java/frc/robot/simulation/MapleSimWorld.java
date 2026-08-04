package frc.robot.simulation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.AllianceFlipUtil;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.FlywheelConstants.TrajectoryVisualization;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

/**
 * Owns the MapleSim physics world: the rigid-body drivetrain, the 2026 Rebuilt field, and the
 * robot's interaction with Fuel.
 *
 * <p><b>This class must never be constructed outside simulation.</b> MapleSim itself throws if its
 * arena is touched on a real robot. Every entry point here is guarded by {@link #isActive()}, and
 * {@code RobotContainer} only builds it inside the {@code SIM} branch.
 *
 * <p><b>What this replaces:</b> the stock {@code ModuleIOSim} spins four independent {@code
 * DCMotorSim}s and integrates the resulting wheel speeds through kinematics — it has no concept of
 * robot mass, traction limits, or walls. MapleSim instead simulates the chassis as a rigid body
 * pushed by per-module friction forces, so wheel slip, collisions, and realistic acceleration all
 * emerge from the physics.
 *
 * <p><b>Ground truth vs. estimate:</b> {@link #getGroundTruthPose()} is where the robot <i>actually
 * is</i> in the physics world. {@code Drive.getPose()} is where the pose estimator <i>thinks</i> it
 * is. Feeding vision the ground truth (rather than the estimator's own output) is what makes vision
 * simulation meaningful — the two poses diverging is exactly the error vision exists to correct.
 */
public class MapleSimWorld {

  private static MapleSimWorld instance;

  private final SwerveDriveSimulation driveSimulation;
  private final IntakeSimulation intakeSimulation;

  // ── Mechanism state, wired from RobotContainer ──────────────────────────────
  // Suppliers rather than subsystem references: this class is not a subsystem and must not
  // participate in the command scheduler or hold subsystems (see .claude/rules/01-architecture.md).
  private Supplier<Angle> intakePivotAngle = () -> Rotations.of(0);
  private Supplier<AngularVelocity> intakeRollerVelocity = () -> RPM.of(0);
  private Supplier<AngularVelocity> flywheelVelocity = () -> RPM.of(0);
  private Supplier<AngularVelocity> upperFeederVelocity = () -> RPM.of(0);
  private Supplier<Angle> hoodAngle = () -> Radians.of(0);
  private BooleanSupplier flywheelSpunUp = () -> false;

  /** Hub center height, from MapleSim's {@code RebuiltHub.blueHubPose}. */
  private static final double HUB_HEIGHT_METERS = 1.5748;

  /** Timestamp of the last simulated shot, used to rate-limit the shot cadence. */
  private double lastShotTimestamp = 0.0;

  /** Timestamp of the last Fuel fed back in by the simulated human player. */
  private double lastFuelReturnTimestamp = 0.0;

  /** Scatter for returned Fuel. Seeded for repeatable runs. */
  private final java.util.Random random = new java.util.Random(2026);

  private MapleSimWorld() {
    // Install our arena BEFORE anything calls SimulatedArena.getInstance(), which would otherwise
    // lazily create the stock Arena2026Rebuilt and leave the bumps in place as solid walls.
    SimulatedArena.overrideInstance(new RebuiltArena());

    // One shared module config — MapleSim calls the supplier once per corner.
    SwerveModuleSimulationConfig moduleConfig =
        new SwerveModuleSimulationConfig(
            SimulationConstants.DRIVE_MOTOR,
            SimulationConstants.STEER_MOTOR,
            SimulationConstants.DRIVE_GEAR_RATIO,
            SimulationConstants.STEER_GEAR_RATIO,
            SimulationConstants.DRIVE_FRICTION_VOLTAGE,
            SimulationConstants.STEER_FRICTION_VOLTAGE,
            SimulationConstants.WHEEL_RADIUS,
            SimulationConstants.STEER_MOI,
            SimulationConstants.WHEEL_COF);

    DriveTrainSimulationConfig driveConfig =
        DriveTrainSimulationConfig.Default()
            .withRobotMass(SimulationConstants.ROBOT_MASS)
            .withBumperSize(SimulationConstants.BUMPER_LENGTH_X, SimulationConstants.BUMPER_WIDTH_Y)
            // Reuse Drive's module translations so the sim geometry can never drift from the
            // kinematics the real robot uses.
            .withCustomModuleTranslations(Drive.getModuleTranslations())
            .withGyro(org.ironmaple.simulation.drivesims.COTS.ofPigeon2())
            .withSwerveModule(moduleConfig);

    driveSimulation =
        new SwerveDriveSimulation(driveConfig, SimulationConstants.DEFAULT_STARTING_POSE);

    // Over-the-bumper: the grab zone extends past the front bumper, matching the pivoting
    // intake that swings out when deployed.
    intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            driveSimulation,
            SimulationConstants.INTAKE_WIDTH,
            SimulationConstants.INTAKE_EXTENSION,
            IntakeSide.FRONT,
            SimulationConstants.INTAKE_CAPACITY);

    SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
    intakeSimulation.register();
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  /**
   * True when the MapleSim world should be used at all. Guards every static entry point so callers
   * never need their own mode check.
   */
  public static boolean isActive() {
    return Constants.currentMode == Constants.Mode.SIM && Constants.useMapleSim;
  }

  /**
   * Returns the singleton, creating the physics world on first call.
   *
   * @throws IllegalStateException if called when {@link #isActive()} is false — that would mean
   *     constructing a physics world on a real robot.
   */
  public static MapleSimWorld getInstance() {
    if (!isActive()) {
      throw new IllegalStateException(
          "MapleSimWorld accessed outside simulation. Guard the call with MapleSimWorld.isActive().");
    }
    if (instance == null) {
      instance = new MapleSimWorld();
    }
    return instance;
  }

  // ── Accessors used to build the IO layer ────────────────────────────────────

  public SwerveModuleSimulation[] getModules() {
    return driveSimulation.getModules();
  }

  public GyroSimulation getGyroSimulation() {
    return driveSimulation.getGyroSimulation();
  }

  /** The robot's true pose in the physics world — not the pose estimator's guess. */
  public Pose2d getGroundTruthPose() {
    return driveSimulation.getSimulatedDriveTrainPose();
  }

  /**
   * Teleports the simulated robot. Called when {@code Drive.setPose()} resets odometry (auto start
   * pose, driver reset) so the physics body and the estimator do not silently disagree.
   */
  public void resetPose(Pose2d pose) {
    driveSimulation.setSimulationWorldPose(pose);
  }

  /**
   * Wires the mechanism state MapleSim needs to decide when Fuel is picked up and when a shot
   * leaves the shooter. Called once from {@code RobotContainer} — this is wiring, not logic.
   */
  public void configureMechanisms(
      Supplier<Angle> intakePivotAngle,
      Supplier<AngularVelocity> intakeRollerVelocity,
      Supplier<AngularVelocity> flywheelVelocity,
      Supplier<AngularVelocity> upperFeederVelocity,
      Supplier<Angle> hoodAngle,
      BooleanSupplier flywheelSpunUp) {
    this.intakePivotAngle = intakePivotAngle;
    this.intakeRollerVelocity = intakeRollerVelocity;
    this.flywheelVelocity = flywheelVelocity;
    this.upperFeederVelocity = upperFeederVelocity;
    this.hoodAngle = hoodAngle;
    this.flywheelSpunUp = flywheelSpunUp;
  }

  // ── Per-loop update ─────────────────────────────────────────────────────────

  /** Steps the physics world one robot period. Called from {@code Robot.simulationPeriodic()}. */
  public void update() {
    updateIntake();
    updateShooter();
    returnScoredFuel();

    SimulatedArena.getInstance().simulationPeriodic();

    logState();
  }

  /**
   * Puts Fuel back into play to model the human player feeding it in from the depot.
   *
   * <p>MapleSim consumes Fuel scored in the Hub and never returns it, so without this the field
   * drains as you shoot and eventually there is nothing left to intake.
   *
   * <p>Works against Fuel <i>in circulation</i> — on the field, held by the robot, and in flight —
   * rather than what is merely visible on the field. A robot carrying a full 50-Fuel hopper is not
   * missing Fuel, and treating it as missing would flood the field with replacements.
   *
   * <p>Rate-limited, so a large deficit trickles back in the way a human player would feed it
   * rather than appearing all at once.
   */
  private void returnScoredFuel() {
    if (!SimulationConstants.FUEL_RETURN_ENABLED) {
      return;
    }

    double now = Timer.getFPGATimestamp();
    if (now - lastFuelReturnTimestamp < SimulationConstants.FUEL_RETURN_INTERVAL_SECONDS) {
      return;
    }

    if (fuelInCirculation() >= SimulationConstants.TARGET_FUEL_IN_CIRCULATION) {
      // Nothing owed. Hold the clock at now so the next genuine deficit is served promptly.
      lastFuelReturnTimestamp = now;
      return;
    }
    lastFuelReturnTimestamp = now;

    Translation2d depot = SimulationConstants.FUEL_RETURN_POSITION;
    if (AllianceFlipUtil.shouldFlip()) {
      depot = AllianceFlipUtil.apply(depot);
    }

    double scatter = SimulationConstants.FUEL_RETURN_SCATTER_METERS;
    Translation2d spawn =
        depot.plus(
            new Translation2d(
                (random.nextDouble() - 0.5) * scatter, (random.nextDouble() - 0.5) * scatter));

    SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(spawn));
  }

  /** Fuel still in play: lying on the field, held by the robot, or airborne. */
  private int fuelInCirculation() {
    return SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel").length
        + intakeSimulation.getGamePiecesAmount()
        + SimulatedArena.getInstance().gamePieceLaunched().size();
  }

  /**
   * The intake grabs Fuel only when the pivot is deployed and the rollers are actually spinning —
   * driving over Fuel with the intake stowed does nothing, same as the real robot.
   */
  private void updateIntake() {
    boolean deployed = intakePivotAngle.get().lte(SimulationConstants.PIVOT_DEPLOYED_THRESHOLD);
    boolean rollersRunning =
        Math.abs(intakeRollerVelocity.get().in(RPM))
            >= SimulationConstants.ROLLER_RUNNING_THRESHOLD.in(RPM);

    if (deployed && rollersRunning) {
      intakeSimulation.startIntake();
    } else {
      intakeSimulation.stopIntake();
    }
  }

  /**
   * Launches a Fuel projectile when the robot is genuinely shooting: flywheel at speed, upper
   * feeder pushing Fuel into it, and at least one Fuel actually held.
   *
   * <p>Shots are rate-limited to {@link SimulationConstants#SHOT_INTERVAL_SECONDS} because the
   * feeder runs continuously while the real mechanism releases discrete balls.
   *
   * <p>The launch model deliberately reuses {@link TrajectoryVisualization} — the same drum radius,
   * fudge factor, launch height and exit offset that drive the AdvantageScope trajectory preview.
   * Sim projectiles therefore follow the arc the preview already draws.
   */
  private void updateShooter() {
    AngularVelocity flywheelSpeed = flywheelVelocity.get();

    boolean feeding =
        Math.abs(upperFeederVelocity.get().in(RPM))
            >= SimulationConstants.FEEDER_FEEDING_THRESHOLD.in(RPM);

    // Require the flywheel to have actually reached its commanded speed, not merely to be
    // spinning. ShootSequences waits on isFlywheelSpunUp but with a timeout, so the feeder can
    // run while the flywheel is still ramping; firing then launches Fuel far below the RPM the
    // characterization tables assume, and every such shot falls short.
    boolean spunUp =
        flywheelSpunUp.getAsBoolean()
            && Math.abs(flywheelSpeed.in(RPM)) >= SimulationConstants.MIN_SHOT_VELOCITY.in(RPM);

    Logger.recordOutput(SimulationConstants.LOG_ROOT + "/Shot/Feeding", feeding);
    Logger.recordOutput(SimulationConstants.LOG_ROOT + "/Shot/SpunUp", spunUp);
    Logger.recordOutput(SimulationConstants.LOG_ROOT + "/Shot/FlywheelRPM", flywheelSpeed.in(RPM));

    double now = Timer.getFPGATimestamp();

    if (!(feeding && spunUp)) {
      // Not shooting: hold the cadence clock at "now" so the first shot after the feeder spins
      // up fires immediately instead of dumping a backlog of missed intervals.
      lastShotTimestamp = now;
      return;
    }

    // The shot interval (50 ms) is shorter than the robot loop (20 ms per call), so fire every
    // shot that came due since the last loop rather than capping the rate at one per loop.
    int fired = 0;
    while (now - lastShotTimestamp >= SimulationConstants.SHOT_INTERVAL_SECONDS
        && fired < SimulationConstants.MAX_SHOTS_PER_LOOP) {
      if (!launchOneFuel(flywheelSpeed)) {
        // Robot is empty — stop trying and resync the clock so it does not build up credit.
        lastShotTimestamp = now;
        break;
      }
      lastShotTimestamp += SimulationConstants.SHOT_INTERVAL_SECONDS;
      fired++;
    }

    if (fired > 0) {
      Logger.recordOutput(SimulationConstants.LOG_ROOT + "/ShotsThisLoop", fired);
    }
  }

  /**
   * Consumes one held Fuel and adds it to the field as a projectile.
   *
   * @return false if the robot had no Fuel to shoot
   */
  private boolean launchOneFuel(AngularVelocity flywheelSpeed) {
    if (!intakeSimulation.obtainGamePieceFromIntake()) {
      return false;
    }

    // v = ω · r · factor. Uses the sim's own factor rather than the visualizer's 0.8, which
    // cannot reach the Hub from any mapped distance — see SimulationConstants.
    double launchSpeedMetersPerSec =
        Math.abs(flywheelSpeed.in(RadiansPerSecond))
            * TrajectoryVisualization.DRUM_RADIUS_METERS
            * SimulationConstants.LAUNCH_VELOCITY_FACTOR;

    Pose2d robotPose = getGroundTruthPose();

    // The shooter fires out the back of the robot, so it faces 180° from the robot heading.
    Rotation2d shooterFacing = robotPose.getRotation().plus(Rotation2d.k180deg);

    logShotPrediction(robotPose, launchSpeedMetersPerSec, getLaunchElevation().in(Degrees));

    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new RebuiltFuelOnFly(
                robotPose.getTranslation(),
                // MapleSim computes the spawn point as
                //   robotPosition + offset.rotateBy(shooterFacing)
                // rotating by the SHOOTER facing, not the robot heading. So this offset lives in
                // the shooter's frame and is positive "out the muzzle". Passing the robot-frame
                // value (negative X) spawns the Fuel out the front of the robot instead.
                new Translation2d(SimulationConstants.SHOOTER_EXIT_DISTANCE.in(Meters), 0.0),
                driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                shooterFacing,
                SimulationConstants.LAUNCH_HEIGHT,
                MetersPerSecond.of(launchSpeedMetersPerSec),
                getLaunchElevation()));
    return true;
  }

  /**
   * Publishes what this shot is predicted to do, so a miss can be diagnosed without guessing.
   *
   * <p>{@code ArrivalHeight} is the key signal. Compare it against the Hub at 1.575 m:
   *
   * <ul>
   *   <li><b>Arrival height is right but Fuel still misses</b> — the launch model is fine and the
   *       problem is aim, robot pose, or the shot firing at the wrong distance.
   *   <li><b>Arrival height is low</b> — the model itself is off; check {@code FlywheelRPM} against
   *       {@code Flywheel/targetRPM} first, since a flywheel below its commanded speed makes every
   *       shot fall short no matter how good the angle is.
   * </ul>
   *
   * <p>Uses MapleSim's inflated gravity, not Earth's — see {@link SimulationConstants}.
   */
  private void logShotPrediction(Pose2d robotPose, double speed, double elevationDegrees) {
    double distanceToHub =
        robotPose
            .getTranslation()
            .getDistance(RobotState.getInstance().getAllianceHubTarget().toTranslation2d());
    double flightDistance = distanceToHub - SimulationConstants.SHOOTER_EXIT_DISTANCE.in(Meters);
    double theta = Math.toRadians(elevationDegrees);

    double flightTime = flightDistance / (speed * Math.cos(theta));
    double arrivalHeight =
        SimulationConstants.LAUNCH_HEIGHT.in(Meters)
            + speed * Math.sin(theta) * flightTime
            - 0.5 * GamePieceProjectile.GRAVITY * flightTime * flightTime;
    double arrivalVerticalSpeed =
        speed * Math.sin(theta) - GamePieceProjectile.GRAVITY * flightTime;

    String root = SimulationConstants.LOG_ROOT + "/Shot/";
    Logger.recordOutput(root + "DistanceToHubMeters", distanceToHub);
    Logger.recordOutput(root + "ExitSpeedMPS", speed);
    Logger.recordOutput(root + "ArrivalHeightMeters", arrivalHeight);
    Logger.recordOutput(root + "ArrivalErrorMeters", arrivalHeight - HUB_HEIGHT_METERS);
    // Negative means descending into the goal; positive means still rising into the front of it.
    Logger.recordOutput(root + "ArrivalVerticalSpeedMPS", arrivalVerticalSpeed);
  }

  /**
   * Converts the hood's mechanism angle into the Fuel's actual launch elevation.
   *
   * <p>Feeding the raw hood reading in as elevation gives a 1°–12° shot that cannot reach the Hub.
   * See {@code SimulationConstants.LAUNCH_ANGLE_OFFSET_DEGREES} for where this mapping came from.
   */
  private Angle getLaunchElevation() {
    double elevationDegrees =
        SimulationConstants.LAUNCH_ANGLE_OFFSET_DEGREES
            + SimulationConstants.LAUNCH_ANGLE_PER_HOOD_DEGREE * hoodAngle.get().in(Degrees);
    Logger.recordOutput(SimulationConstants.LOG_ROOT + "/LaunchAngleDeg", elevationDegrees);
    return Degrees.of(elevationDegrees);
  }

  /** Publishes sim state for AdvantageScope. Ground truth is the key signal for judging drift. */
  private void logState() {
    Logger.recordOutput(SimulationConstants.LOG_ROOT + "/GroundTruthPose", getGroundTruthPose());
    Logger.recordOutput(
        SimulationConstants.LOG_ROOT + "/FuelHeld", intakeSimulation.getGamePiecesAmount());
    Logger.recordOutput(
        SimulationConstants.LOG_ROOT + "/IntakeRunning", intakeSimulation.isRunning());
    Logger.recordOutput(
        SimulationConstants.LOG_ROOT + "/FlywheelSpunUp", flywheelSpunUp.getAsBoolean());
    // Every Fuel the arena draws, for the AdvantageScope 3D view. Drag this onto the field as a
    // "Fuel" game piece to watch shots fly and land.
    Pose3d[] fuelPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");
    Logger.recordOutput(SimulationConstants.LOG_ROOT + "/Fuel", fuelPoses);

    // Fuel accounting. The field starts with 192; if fewer are visible, they are somewhere in
    // this breakdown rather than lost. The robot alone can hold INTAKE_CAPACITY (50) of them,
    // which is a quarter of the field, and Fuel scored in the Hub stays there until the Outpost
    // releases it.
    int held = intakeSimulation.getGamePiecesAmount();
    int inFlight = SimulatedArena.getInstance().gamePieceLaunched().size();
    Logger.recordOutput(SimulationConstants.LOG_ROOT + "/Fuel/Drawn", fuelPoses.length);
    Logger.recordOutput(SimulationConstants.LOG_ROOT + "/Fuel/HeldByRobot", held);
    Logger.recordOutput(SimulationConstants.LOG_ROOT + "/Fuel/InFlight", inFlight);
    int circulating = fuelPoses.length + held + inFlight;
    Logger.recordOutput(SimulationConstants.LOG_ROOT + "/Fuel/InCirculation", circulating);
    // How much Fuel the Hub has swallowed that the human player has not fed back in yet.
    Logger.recordOutput(
        SimulationConstants.LOG_ROOT + "/Fuel/AwaitingReturn",
        Math.max(0, SimulationConstants.TARGET_FUEL_IN_CIRCULATION - circulating));
  }
}
