package frc.robot.simulation;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
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

  /** Timestamp of the last simulated shot, used to rate-limit the shot cadence. */
  private double lastShotTimestamp = 0.0;

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

    SimulatedArena.getInstance().simulationPeriodic();

    logState();
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
    boolean fastEnough =
        Math.abs(flywheelSpeed.in(RPM)) >= SimulationConstants.MIN_SHOT_VELOCITY.in(RPM);
    boolean cadenceElapsed =
        Timer.getFPGATimestamp() - lastShotTimestamp >= SimulationConstants.SHOT_INTERVAL_SECONDS;

    if (!(feeding && fastEnough && cadenceElapsed)) {
      return;
    }

    // Consume a held Fuel. Returns false when the robot is empty — nothing to shoot.
    if (!intakeSimulation.obtainGamePieceFromIntake()) {
      return;
    }
    lastShotTimestamp = Timer.getFPGATimestamp();

    // v = ω × r, matching FlywheelVisualizer.toLinearVelocity()
    double launchSpeedMetersPerSec =
        Math.abs(flywheelSpeed.in(RadiansPerSecond))
            * TrajectoryVisualization.DRUM_RADIUS_METERS
            * TrajectoryVisualization.VELOCITY_FUDGE_FACTOR;

    Pose2d robotPose = getGroundTruthPose();

    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new RebuiltFuelOnFly(
                robotPose.getTranslation(),
                new Translation2d(
                    TrajectoryVisualization.SHOOTER_EXIT_X_METERS,
                    TrajectoryVisualization.SHOOTER_EXIT_Y_METERS),
                driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                // The shooter fires out the back of the robot (-X), so it faces 180° from heading.
                robotPose.getRotation().plus(Rotation2d.k180deg),
                Meters.of(TrajectoryVisualization.LAUNCH_HEIGHT_METERS),
                MetersPerSecond.of(launchSpeedMetersPerSec),
                hoodAngle.get()));
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
    // Every Fuel on the field, for the AdvantageScope 3D view. Drag this onto the field as a
    // "Fuel" game piece to watch shots fly and land.
    Logger.recordOutput(
        SimulationConstants.LOG_ROOT + "/Fuel",
        SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }
}
