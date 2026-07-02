// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.metersToInches;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.AllianceFlipUtil;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeederCommands;
import frc.robot.commands.FlywheelCommands;
import frc.robot.commands.HoodCommands;
import frc.robot.commands.IntakePivotCommands;
import frc.robot.commands.PrestageCommands;
import frc.robot.commands.ShootSequences;
import frc.robot.commands.TransportCommands;
import frc.robot.commands.intakeRollerCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.io.FlywheelIO;
import frc.robot.subsystems.flywheel.io.FlywheelIOPhoenix6;
import frc.robot.subsystems.flywheel.io.FlywheelIOSim;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodPosCalculator;
import frc.robot.subsystems.hood.io.HoodIO;
import frc.robot.subsystems.hood.io.HoodIOReal;
import frc.robot.subsystems.hood.io.HoodIOSim;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakePivot.io.IntakePivotIO;
import frc.robot.subsystems.intakePivot.io.IntakePivotIOReal;
import frc.robot.subsystems.intakePivot.io.IntakePivotIOSim;
import frc.robot.subsystems.intakeRoller.intakeRoller;
import frc.robot.subsystems.intakeRoller.io.intakeRollerIO;
import frc.robot.subsystems.intakeRoller.io.intakeRollerIOReal;
import frc.robot.subsystems.intakeRoller.io.intakeRollerIOSim;
import frc.robot.subsystems.lowerFeeder.LowerFeeder;
import frc.robot.subsystems.lowerFeeder.io.LowerFeederIO;
import frc.robot.subsystems.lowerFeeder.io.LowerFeederIOReal;
import frc.robot.subsystems.lowerFeeder.io.LowerFeederIOSim;
import frc.robot.subsystems.prestage.Prestage;
import frc.robot.subsystems.prestage.io.PrestageIO;
import frc.robot.subsystems.prestage.io.PrestageIOReal;
import frc.robot.subsystems.prestage.io.PrestageIOSim;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.transport.io.TransportIO;
import frc.robot.subsystems.transport.io.TransportIOReal;
import frc.robot.subsystems.transport.io.TransportIOSim;
import frc.robot.subsystems.upperFeeder.UpperFeeder;
import frc.robot.subsystems.upperFeeder.io.UpperFeederIO;
import frc.robot.subsystems.upperFeeder.io.UpperFeederIOReal;
import frc.robot.subsystems.upperFeeder.io.UpperFeederIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.io.VisionIO;
import frc.robot.subsystems.vision.io.VisionIOPhotonVision;
import frc.robot.subsystems.vision.io.VisionIOPhotonVisionSim;
import frc.robot.util.HubShiftUtil;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Flywheel flywheel;
  private final UpperFeeder upperFeeder;
  private final LowerFeeder lowerFeeder;
  private final Hood hood;
  private final Prestage prestage;
  private final IntakePivot intakePivot;
  private final intakeRoller intakeRoller;
  private final Transport transport;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // ── Auto Preview & Starting Pose Check ──────────────────────────────────────
  // Field2d widget to show the selected auto's path and the robot's current position.
  // Used during disabled/pre-match to verify the robot is placed correctly.
  public final Field2d autoPreviewField = new Field2d();

  // ── Compress Cancellation Tracking ──────────────────────────────────────────
  // When the driver overrides the automatic compress (by pressing intake in/out/compress),
  // this flag is set to true so the auto-compress won't reschedule until the shoot button
  // is released. Reset to false when the shoot button is released.
  private boolean compressCancelled = false;

  // Similar cancellation flag for the auto-x
  // Set to true when driver hits x-override button
  // Resets to false when shoot button is released
  private boolean xCancelled = false;

  // Similar latch for double compress
  // Sets to true when driver hits double compress override
  // Resets to false when shoot button is released
  public boolean doubleCompress = false;

  // Stores the starting pose of the currently selected auto.
  // Updated when the auto chooser selection changes.
  private Pose2d autoStartPose = new Pose2d();
  private final double defaultAutoDelay = 0.0;
  private final String autoDelayKey = "Auto Delay";

  // ── Starting Pose Tolerances ────────────────────────────────────────────────
  // How close (in inches) the robot needs to be to the auto's starting position
  // for us to consider it "close enough" to start the match.
  private static final Distance STARTING_POSE_DRIVE_TOLERANCE = Inches.of(6.0);

  // How close (in degrees) the robot's heading needs to be to the auto's starting heading.
  private static final double STARTING_POSE_ROT_TOLERANCE_DEGREES = 5.0;

  // Controllers
  private final CommandXboxController controller =
      new CommandXboxController(HardwareConstants.ControllerConstants.XboxControllerPort);
  private final CommandXboxController simController =
      new CommandXboxController(HardwareConstants.ControllerConstants.SimControllerPort);

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1),
                new VisionIOPhotonVision(
                    VisionConstants.camera2Name, VisionConstants.robotToCamera2),
                new VisionIOPhotonVision(
                    VisionConstants.camera3Name, VisionConstants.robotToCamera3));
        flywheel = new Flywheel(new FlywheelIOPhoenix6());
        upperFeeder = new UpperFeeder(new UpperFeederIOReal());
        lowerFeeder = new LowerFeeder(new LowerFeederIOReal());
        hood = new Hood(new HoodIOReal());
        prestage = new Prestage(new PrestageIOReal());
        transport = new Transport(new TransportIOReal());
        intakePivot = new IntakePivot(new IntakePivotIOReal());
        intakeRoller = new intakeRoller(new intakeRollerIOReal());
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera2Name, VisionConstants.robotToCamera2, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera3Name, VisionConstants.robotToCamera3, drive::getPose));
        flywheel = new Flywheel(new FlywheelIOSim());
        upperFeeder = new UpperFeeder(new UpperFeederIOSim());
        lowerFeeder = new LowerFeeder(new LowerFeederIOSim());
        prestage = new Prestage(new PrestageIOSim());
        hood = new Hood(new HoodIOSim());
        transport = new Transport(new TransportIOSim());
        intakePivot = new IntakePivot(new IntakePivotIOSim());
        intakeRoller = new intakeRoller(new intakeRollerIOSim());
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // Replay requires one VisionIO per camera so all four cameras' logged inputs are replayed
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
        upperFeeder = new UpperFeeder(new UpperFeederIO() {});
        lowerFeeder = new LowerFeeder(new LowerFeederIO() {});
        prestage = new Prestage(new PrestageIO() {});
        hood = new Hood(new HoodIO() {});
        transport = new Transport(new TransportIO() {});
        intakePivot = new IntakePivot(new IntakePivotIO() {});
        intakeRoller = new intakeRoller(new intakeRollerIO() {});
        break;
    }

    // Wire the hood angle supplier into the flywheel's trajectory visualizer.
    // This keeps Flywheel and Hood decoupled — the supplier is the only link.
    flywheel.setHoodAngleSupplier(hood::getPosition);

    // IMPORTANT: Register named commands and event triggers BEFORE building the auto chooser.
    // AutoBuilder.buildAutoChooser() parses the .auto files and resolves named commands at
    // build time. If commands aren't registered yet, they resolve to Commands.none().
    registerNamedCommands();
    registerEventTriggers();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addDefaultOption(
        HardwareConstants.CompConstants.Autos.DefaultAutoName,
        new PathPlannerAuto(HardwareConstants.CompConstants.Autos.DefaultAutoName));

    // Publish the auto preview field to the dashboard so we can see the selected path
    SmartDashboard.putData("Auto Preview", autoPreviewField);
    SmartDashboard.putNumber(autoDelayKey, defaultAutoDelay);

    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    if (Robot.isReal()) {
      configureStateBindings();
    } else if (Robot.isSimulation()) {
      configureSimBindings();
    }
  }

  private double deadband(double value) {
    return MathUtil.applyDeadband(value, HardwareConstants.ControllerConstants.DEADBAND);
  }

  // Drive-axis inputs are source-gated in Triggers on the "Use Xbox Drive" flag, so these
  // return the Thrustmaster or the Xbox drive controller depending on the live selection.
  private double getThrustX() {
    return Triggers.getInstance().thrustmaster.getX(); // strafe
  }

  private double getThrustY() {
    return Triggers.getInstance().thrustmaster.getY(); // forward
  }

  private double getThrustRot() {
    return Triggers.getInstance().thrustmaster.getTwist(); // twist
  }

  // NamedCommands
  private void registerNamedCommands() {
    // Auto deploy intake command
    NamedCommands.registerCommand(
        "DeployIntake",
        IntakePivotCommands.setPivotPosition(
            intakePivot, HardwareConstants.CompConstants.Positions.pivotDownPos));

    // Auto retract intake command
    NamedCommands.registerCommand(
        "RetractIntake",
        IntakePivotCommands.setPivotPosition(
            intakePivot, HardwareConstants.CompConstants.Positions.pivotUpPos));

    // Auto run intake command
    NamedCommands.registerCommand(
        "RunIntake",
        intakeRollerCommands
            .setRollerVoltage(
                intakeRoller, HardwareConstants.CompConstants.Voltages.intakeRollerVoltage)
            .alongWith(
                TransportCommands.setTransportVoltage(
                    transport, HardwareConstants.CompConstants.Voltages.transportVoltage)));

    // Auto shoot command
    NamedCommands.registerCommand(
        "Shoot",
        DriveCommands.joystickDriveAtAngle(
                drive, () -> 0, () -> 0, () -> RobotState.getInstance().getAngleToAllianceHub())
            .alongWith(
                ShootSequences.autoShootToHub(
                    flywheel,
                    prestage,
                    hood,
                    upperFeeder,
                    lowerFeeder,
                    transport,
                    intakeRoller,
                    intakePivot)));

    // Stop all subsystems after shooting
    NamedCommands.registerCommand(
        "stopAll",
        ShootSequences.stopAll(
            flywheel, prestage, hood, upperFeeder, lowerFeeder, transport, intakeRoller));

    // Puts the hood down.
    NamedCommands.registerCommand(
        "HoodDownNamed",
        HoodCommands.setHoodPos(hood, HardwareConstants.CompConstants.Positions.hoodDownPos));
  }
  // EventTriggers
  //
  // IMPORTANT: Event trigger commands must NOT declare subsystem requirements that overlap
  // with ANY command in the auto command group. A SequentialCommandGroup's requirements are
  // the UNION of all its sub-commands' requirements. So even though "DeployIntake" fires
  // during the path-following step, the auto group also includes the "Shoot" named command
  // which requires intakePivot (via ShootSequences.shootToHub → jostlePivotByPos).
  //
  // If an EventTrigger schedules a command that shares a requirement with the auto group,
  // WPILib's scheduler will INTERRUPT the entire auto group to resolve the conflict.
  //
  // Fix: Use Commands.runOnce() WITHOUT passing the subsystem as a requirement.
  // The subsystem method is still called (so the hardware moves), but the command
  // doesn't "require" the subsystem, avoiding the scheduling conflict.
  private void registerEventTriggers() {
    // Event marker for intake deploy command (no subsystem requirement to avoid auto interruption)
    new EventTrigger("DeployIntake")
        .onTrue(
            Commands.runOnce(
                () ->
                    intakePivot.setPivotPosition(
                        HardwareConstants.CompConstants.Positions.pivotDownPos)));

    // Event marker for intake retract command (no subsystem requirement to avoid auto interruption)
    new EventTrigger("RetractIntake")
        .onTrue(
            Commands.runOnce(
                () ->
                    intakePivot.setPivotPosition(
                        HardwareConstants.CompConstants.Positions.pivotUpPos)));

    // Event marker for running intake rollers while in a zoned area
    // Uses whileTrue because this is a zoned event marker (has start AND end positions in the path)
    // No subsystem requirements declared to avoid interrupting the auto command group
    new EventTrigger("RunIntake")
        .whileTrue(
            Commands.startEnd(
                () -> {
                  intakeRoller.setRollerVoltage(
                      HardwareConstants.CompConstants.Voltages.intakeRollerVoltage);
                },
                () -> {
                  intakeRoller.setRollerVoltage(Volts.of(0));
                }));

    // Event marker for setting the hood position to down
    new EventTrigger("HoodDown")
        .onTrue(
            HoodCommands.setHoodPos(hood, HardwareConstants.CompConstants.Positions.hoodDownPos));
  }

  private void configureStateBindings() {

    // DEFAULT COMMANDS
    // Drivetrain - joystick drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> MathUtil.clamp(-getThrustY(), -1.0, 1.0),
            () -> MathUtil.clamp(-getThrustX(), -1.0, 1.0),
            () -> MathUtil.clamp(-getThrustRot(), -1.0, 1.0)));
    // Flywheel - idle
    flywheel.setDefaultCommand(FlywheelCommands.flywheelIdle(flywheel));
    // // Prestage - idle
    // prestage.setDefaultCommand(PrestageCommands.prestageIdle(prestage));
    // Hood - stop motor when no command is running (prevents stale closed-loop reference)
    hood.setDefaultCommand(HoodCommands.hoodIdle(hood));

    // Intake Roller - Idle
    intakeRoller.setDefaultCommand(intakeRollerCommands.intakeRollerIdle(intakeRoller));
    // OVERRIDES
    // Flip alliance winner
    Triggers.getInstance().allianceWinFlipper().onTrue(HubShiftUtil.flipWinner());
    // Disable hub shift logic
    Triggers.getInstance().allianceWinDisabler().onTrue(HubShiftUtil.disableHubShiftUtil());
    // Manually cancel auto-x
    Triggers.getInstance().autoXOverride().onTrue(Commands.runOnce(() -> xCancelled = true));
    // Compress for half hopper
    Triggers.getInstance()
        .doubleCompressOverride()
        .onTrue(Commands.runOnce(() -> doubleCompress = !doubleCompress));

    // DRIVETRAIN
    // Align for shoot when shoot button is pressed and we're in our alliance zone and hub is
    // active, or if tower shoot button is pressed
    // X while aligned
    // This command does not run if the robot is approaching a hardstop spot (bump or trench) -
    // instead it runs
    // the hit hardstop and align command below
    ((Triggers.getInstance().shootButton().and(Triggers.getInstance().isShootSafeZone))
            .and(
                () ->
                    !(frc.robot.RobotState.getInstance()
                            .getApproachingZoneX(
                                frc.robot.RobotState.getInstance().getEstimatedPose())
                        == HardwareConstants.Zones.approachingZoneX.APPROACHING_ALLIANCE_TRENCH)))
        .or(Triggers.getInstance().shootFromTowerButton())
        .whileTrue(
            DriveCommands.alignOrXForShoot(
                drive,
                () -> -Triggers.getInstance().thrustmaster.getY() * .5,
                () -> -Triggers.getInstance().thrustmaster.getX() * .5,
                () -> RobotState.getInstance().getAngleToAllianceHub()));

    // If close to a hardstop spot (bump or trench), or if hardstop shoot button pressed,
    // run into the hardstop and align to shoot
    // Eventually add tower
    (Triggers.getInstance()
            .shootButton()
            .and(Triggers.getInstance().isShootClear)
            .and(
                () ->
                    frc.robot.RobotState.getInstance()
                            .getApproachingZoneX(
                                frc.robot.RobotState.getInstance().getEstimatedPose())
                        == HardwareConstants.Zones.approachingZoneX.APPROACHING_ALLIANCE_TRENCH))
        .or(Triggers.getInstance().hardstopShootButton())
        .whileTrue(
            Commands.sequence(
                DriveCommands.alignForDefenseShot(drive),
                DriveCommands.alignOrXForShoot(
                    drive,
                    () -> Triggers.getInstance().thrustmaster.getX(),
                    () -> Triggers.getInstance().thrustmaster.getY(),
                    () -> new Rotation2d(Triggers.getInstance().thrustmaster.getTwist()))));

    // Align for pass if shoot button is pressed but we're not in our alliance zone, or if pass
    // button is pressed
    // Requires demo mode to be false (if demo mode is on we don't want to align to pass)
    (Triggers.getInstance()
            .shootButton()
            .and(() -> !Triggers.getInstance().isShootSafeZone.getAsBoolean()))
        .or(Triggers.getInstance().passButton())
        .and(() -> !HardwareConstants.TuningConstants.DEMO_MODE)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -Triggers.getInstance().thrustmaster.getX() * .5,
                () -> -Triggers.getInstance().thrustmaster.getY() * .5,
                () ->
                    RobotState.getInstance()
                        .getAngleToTarget(
                            new Translation2d(
                                RobotState.getInstance().getPassTarget().getX(),
                                RobotState.getInstance().getPassTarget().getY()))));

    // Align for trench when trench button pressed - zone logic temporarily disabled
    Triggers.getInstance()
        .trenchAlignButton()
        // .and(Triggers.getInstance().isRobotInTrench())
        // .or(Triggers.getInstance().isRobotApproachingTrench())
        .whileTrue(
            DriveCommands.joystickDriveAlignForTrench(
                drive,
                () -> -Triggers.getInstance().thrustmaster.getY() * .5,
                () -> -Triggers.getInstance().thrustmaster.getX() * .5));

    // Align for bump when bump button pressed - zone logic temporarily disabled
    Triggers.getInstance()
        .bumpAlignButton()
        // .and(Triggers.getInstance().isRobotOnBump())
        // .or(Triggers.getInstance().isRobotApproachingBump())
        .whileTrue(
            DriveCommands.joystickDriveAlignForBump(
                drive,
                () -> -Triggers.getInstance().thrustmaster.getY() * .5,
                () -> -Triggers.getInstance().thrustmaster.getX() * .5));

    // UPPER SHOOTER
    // Set shooting velocity if shoot button pressed, we're in our alliance zone, hub is active, and
    // tuning false
    Triggers.getInstance()
        .shootButton()
        .and(Triggers.getInstance().isShootClear)
        .and(() -> !HardwareConstants.TuningConstants.TUNING_MODE)
        .whileTrue(
            FlywheelCommands.setVelocityForHub(flywheel)
                .alongWith(
                    PrestageCommands.setPrestageVelocity(
                        prestage, HardwareConstants.CompConstants.Velocities.prestageVelocity)))
        .onFalse(FlywheelCommands.stop(flywheel))
        .onFalse(PrestageCommands.stop(prestage));

    // Set passing velocity if shoot button is pressed but we're not in our alliance zone and tuning
    // false,
    // or if pass button is pressed
    (Triggers.getInstance()
            .shootButton()
            .and(() -> !Triggers.getInstance().isShootSafeZone.getAsBoolean())
            .and(() -> !HardwareConstants.TuningConstants.TUNING_MODE))
        .or(Triggers.getInstance().passButton())
        .whileTrue(
            FlywheelCommands.setVelocityForPassing(flywheel)
                .alongWith(
                    PrestageCommands.setPrestageVelocity(
                        prestage, HardwareConstants.CompConstants.Velocities.prestageVelocity)))
        .onFalse(FlywheelCommands.stop(flywheel))
        .onFalse(PrestageCommands.stop(prestage));

    // LOWER SHOOTER
    // Set shooting velocity if:
    //   - shoot button is pressed
    //   - tuning is false
    //   - drivetrain is aligned
    //   - robot is not in our alliance zone with the hub inactive
    //   - upper shooter is spun up
    Triggers.getInstance()
        .shootButton()
        .and(() -> !HardwareConstants.TuningConstants.TUNING_MODE)
        .and(
            () ->
                !(Triggers.getInstance().isShootSafeZone.getAsBoolean()
                    && !Triggers.getInstance().isShootSafeTime.getAsBoolean()))
        .and(Triggers.getInstance().isAlignedLooser)
        .whileTrue(
            Commands.sequence(
                Commands.waitUntil(
                        flywheel.isFlywheelSpunUp.and(Triggers.getInstance().isAlignedLooser))
                    .withTimeout(HardwareConstants.CompConstants.Waits.spinUpTimeOut),
                FeederCommands.setUpperFeederVelocity(
                        upperFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity)
                    .alongWith(
                        FeederCommands.setLowerFeederVelocity(
                            lowerFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity))
                    .alongWith(
                        TransportCommands.setTransportVelocity(
                            transport,
                            HardwareConstants.CompConstants.Velocities.transportVelocity))))
        .onFalse(FeederCommands.stopLower(lowerFeeder))
        .onFalse(FeederCommands.stopUpper(upperFeeder))
        .onFalse(TransportCommands.stop(transport));

    // FULL SHOOTER - hard-coded shots, tuning shots, demo shots
    // These use constant waits instead of waiting for alignment or spinup
    // Hard-coded tower shot
    Triggers.getInstance()
        .shootFromTowerButton()
        .whileTrue(
            FlywheelCommands.setFlywheelVelocity(
                    flywheel, HardwareConstants.TowerConstants.FlywheelTowerVelocity)
                .alongWith(
                    PrestageCommands.setPrestageVelocity(
                        prestage, HardwareConstants.CompConstants.Velocities.prestageVelocity))
                .alongWith(
                    FeederCommands.setUpperVelocityAfterWait(
                        upperFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity))
                .alongWith(
                    FeederCommands.setLowerVelocityAfterWait(
                        lowerFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity))
                .alongWith(
                    TransportCommands.setVelocityAfterWait(
                        transport, HardwareConstants.CompConstants.Velocities.transportVelocity)))
        .onFalse(FlywheelCommands.stop(flywheel))
        .onFalse(PrestageCommands.stop(prestage))
        .onFalse(FeederCommands.stopLower(lowerFeeder))
        .onFalse(FeederCommands.stopUpper(upperFeeder))
        .onFalse(TransportCommands.stop(transport));

    // Distance map shot if tuning mode true
    Triggers.getInstance()
        .shootButton()
        .and(() -> HardwareConstants.TuningConstants.TUNING_MODE)
        .whileTrue(
            FlywheelCommands.setFlywheelVelocity(
                    flywheel, HardwareConstants.TuningConstants.FlywheelTuningVelocity)
                .alongWith(
                    PrestageCommands.setPrestageVelocity(
                        prestage, HardwareConstants.CompConstants.Velocities.prestageVelocity))
                .alongWith(
                    FeederCommands.setUpperVelocityAfterWait(
                        upperFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity))
                .alongWith(
                    FeederCommands.setLowerVelocityAfterWait(
                        lowerFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity))
                .alongWith(
                    TransportCommands.setVelocityAfterWait(
                        transport, HardwareConstants.CompConstants.Velocities.transportVelocity)))
        .onFalse(FlywheelCommands.stop(flywheel))
        .onFalse(PrestageCommands.stop(prestage))
        .onFalse(FeederCommands.stopLower(lowerFeeder))
        .onFalse(FeederCommands.stopUpper(upperFeeder))
        .onFalse(TransportCommands.stop(transport));

    // Demo shot if demo mode true and demo shoot button pressed
    Triggers.getInstance()
        .demoDistanceShot()
        .and(() -> HardwareConstants.TuningConstants.DEMO_MODE)
        .whileTrue(
            FlywheelCommands.setFlywheelVelocity(
                    flywheel, HardwareConstants.TuningConstants.FlywheelTuningVelocity)
                .alongWith(
                    PrestageCommands.setPrestageVelocity(
                        prestage, HardwareConstants.CompConstants.Velocities.prestageVelocity))
                .alongWith(
                    FeederCommands.setUpperVelocityAfterWait(
                        upperFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity))
                .alongWith(
                    FeederCommands.setLowerVelocityAfterWait(
                        lowerFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity))
                .alongWith(
                    TransportCommands.setVelocityAfterWait(
                        transport, HardwareConstants.CompConstants.Velocities.transportVelocity)))
        .onFalse(FlywheelCommands.stop(flywheel))
        .onFalse(PrestageCommands.stop(prestage))
        .onFalse(FeederCommands.stopLower(lowerFeeder))
        .onFalse(FeederCommands.stopUpper(upperFeeder))
        .onFalse(TransportCommands.stop(transport));

    // INTAKE ROLLER
    // Set to intaking voltage when intake button is pressed
    Triggers.getInstance()
        .intakeRollerButton()
        .whileTrue(
            intakeRollerCommands.setRollerVoltage(
                intakeRoller, HardwareConstants.CompConstants.Voltages.intakeRollerVoltage))
        .onFalse(intakeRollerCommands.stopIntakeRoller(intakeRoller));

    // INTAKE PIVOT
    // Retract on retract button — also cancels automatic compression for this shoot press
    Triggers.getInstance()
        .intakeInButton()
        .onTrue(Commands.runOnce(() -> compressCancelled = true))
        .whileTrue(
            IntakePivotCommands.setPivotPosition(
                intakePivot, HardwareConstants.CompConstants.Positions.pivotUpPos));

    // Deploy on deploy button — also cancels automatic compression for this shoot press
    Triggers.getInstance()
        .intakeOutButton()
        .onTrue(Commands.runOnce(() -> compressCancelled = true))
        .whileTrue(
            IntakePivotCommands.setPivotPosition(
                intakePivot, HardwareConstants.CompConstants.Positions.pivotDownPos));

    // Manual compress button — cancels auto-compress and runs compress with no initial wait
    // Runs single or double compress based on override - subject to change
    Triggers.getInstance()
        .intakeCompressButton()
        .onTrue(Commands.runOnce(() -> compressCancelled = true))
        .whileTrue(IntakePivotCommands.compressPivot(intakePivot, () -> doubleCompress))
        .onFalse(
            IntakePivotCommands.setPivotPosition(
                intakePivot, HardwareConstants.CompConstants.Positions.pivotDownPos));

    // Automatic compress on shoot button when:
    //   - neither intake deploy nor retract button is currently pressed
    //   - compression has not been cancelled by a prior intake override this shoot press
    //   - we're not in our alliance zone with the hub inactive
    // Runs double compress if operator has activated override, otherwise single
    Triggers.getInstance()
        .shootButton()
        .and(() -> !compressCancelled)
        .and(
            () ->
                !(Triggers.getInstance().isShootSafeZone.getAsBoolean()
                    && !Triggers.getInstance().isShootSafeTime.getAsBoolean()))
        .and(Triggers.getInstance().isAlignedLooser)
        .whileTrue(
            Commands.sequence(
                Commands.waitUntil(flywheel.isFlywheelSpunUp)
                    .withTimeout(HardwareConstants.CompConstants.Waits.spinUpTimeOut),
                IntakePivotCommands.compressPivot(intakePivot, () -> doubleCompress)))
        .onFalse(
            IntakePivotCommands.setPivotPosition(
                intakePivot, HardwareConstants.CompConstants.Positions.pivotDownPos));

    // HOOD
    // Set pos for hub if shoot to hub button or shoot to tower button is pressed, and we're in our
    // alliance zone and the hub is active, and tuning mode is false
    (Triggers.getInstance().shootButton().or(Triggers.getInstance().shootFromTowerButton()))
        .and(Triggers.getInstance().isShootClear)
        .and(() -> !HardwareConstants.TuningConstants.TUNING_MODE)
        .whileTrue(HoodCommands.setHoodPosForHub(hood));

    // Set pos for passing if shoot to hub button is pressed but we're not in our alliance zone, or
    // if pass button is presssed
    (Triggers.getInstance()
            .shootButton()
            .and(() -> !Triggers.getInstance().isShootSafeZone.getAsBoolean())
            .and(() -> !HardwareConstants.TuningConstants.TUNING_MODE))
        .or(Triggers.getInstance().passButton())
        .whileTrue(HoodCommands.setPosForPassing(hood));

    // Set pos to defined tuning pos when shoot button is pressed and tuning mode is on
    Triggers.getInstance()
        .shootButton()
        .and(() -> HardwareConstants.TuningConstants.TUNING_MODE)
        .whileTrue(HoodCommands.setHoodPos(hood, HardwareConstants.TuningConstants.HoodTuningPos));

    // Set pos to demo pose when demo button is pressed and demo mode is on
    Triggers.getInstance()
        .demoDistanceShot()
        .and(() -> HardwareConstants.TuningConstants.DEMO_MODE)
        .whileTrue(HoodCommands.setHoodPos(hood, HardwareConstants.TuningConstants.HoodDemoPos));

    // CANCELLATIONS
    // Auto-compress, auto-x, and double compress cancellations
    Triggers.getInstance()
        .shootButton()
        .onFalse(Commands.runOnce(() -> compressCancelled = false))
        .onFalse(Commands.runOnce(() -> xCancelled = false))
        .onFalse(Commands.runOnce(() -> doubleCompress = false));
  }

  private void configureSimBindings() {

    // In sim there is no FMS game-specific message, so the hub shift schedule defaults
    // to "inactive" after the first 10-second TRANSITION window.  Disabling the hub
    // shift logic lets isShootSafeTime always return true so the hood (and other
    // shoot-gated commands) work at any point during a sim session.
    // if (!HubShiftUtil.disabled) {
    //   HubShiftUtil.toggleDisable();
    // }
    // DEFAULT COMMANDS
    // Drivetrain - joystick drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> MathUtil.clamp(Triggers.getInstance().simXSupplier(), -1.0, 1.0),
            () -> MathUtil.clamp(Triggers.getInstance().simYSupplier(), -1.0, 1.0),
            () -> MathUtil.clamp(Triggers.getInstance().simRotationSupplier(), -1.0, 1.0)));
    // Flywheel - idle
    flywheel.setDefaultCommand(FlywheelCommands.flywheelIdle(flywheel));
    // // Prestage - idle
    // prestage.setDefaultCommand(PrestageCommands.prestageIdle(prestage));
    // Hood - stop motor when no command is running (prevents stale closed-loop reference)
    hood.setDefaultCommand(HoodCommands.hoodIdle(hood));

    // OVERRIDES
    // Flip alliance winner
    Triggers.getInstance().simAllianceWinFlipper().onTrue(HubShiftUtil.flipWinner());
    // Disable hub shift logic
    Triggers.getInstance().allianceWinDisabler().onTrue(HubShiftUtil.disableHubShiftUtil());
    // Manually cancel auto-x
    Triggers.getInstance().autoXOverride().onTrue(Commands.runOnce(() -> xCancelled = true));

    // DRIVETRAIN
    // Align for shoot when shoot button is pressed and we're in our alliance zone and hub is
    // active, or if tower shoot button is pressed
    // Modified to not run if the align to hard stop should be running instead
    (Triggers.getInstance().simShootButton())
        .and(Triggers.getInstance().isShootClear)
        .and(
            () ->
                !(frc.robot.RobotState.getInstance()
                        .getApproachingZoneX(frc.robot.RobotState.getInstance().getEstimatedPose())
                    == HardwareConstants.Zones.approachingZoneX.APPROACHING_ALLIANCE_TRENCH))
        .or(Triggers.getInstance().simShootFromTowerButton())
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> Triggers.getInstance().simXSupplier(),
                () -> Triggers.getInstance().simYSupplier(),
                () -> RobotState.getInstance().getAngleToAllianceHub()));

    // If shooting when near a hardstop spot (trench and eventually tower), go to the hardstop spot
    // and shoot
    Triggers.getInstance()
        .simShootButton()
        .and(Triggers.getInstance().isShootClear)
        .and(
            () ->
                frc.robot.RobotState.getInstance()
                        .getApproachingZoneX(frc.robot.RobotState.getInstance().getEstimatedPose())
                    == HardwareConstants.Zones.approachingZoneX.APPROACHING_ALLIANCE_TRENCH)
        .whileTrue(DriveCommands.alignForDefenseShot(drive));

    // Align for pass if shoot button is pressed but we're not in our alliance zone, or if pass
    // button is pressed
    (Triggers.getInstance()
            .simShootButton()
            .and(() -> !Triggers.getInstance().isShootSafeZone.getAsBoolean()))
        .or(Triggers.getInstance().simPassButton())
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> Triggers.getInstance().simXSupplier(),
                () -> Triggers.getInstance().simYSupplier(),
                () ->
                    RobotState.getInstance()
                        .getAngleToTarget(
                            new Translation2d(
                                RobotState.getInstance().getPassTarget().getX(),
                                RobotState.getInstance().getPassTarget().getY()))));

    // X wheels when shoot button is pressed and we're shooting and we're lined up and
    // auto-x hasn't been manually overriden, or when x button is pressed
    (Triggers.getInstance()
            .simShootButton()
            .and(Triggers.getInstance().isShootClear)
            .and(Triggers.getInstance().isAlignedForCurrentShot)
            .and(() -> !xCancelled))
        .or(Triggers.getInstance().xWheels())
        .whileTrue(DriveCommands.stopWithX(drive));

    // Align for trench when trench button pressed - zone logic temporarily disabled
    Triggers.getInstance()
        .simTrenchAlignButton()
        // .and(Triggers.getInstance().isRobotInTrench())
        // .or(Triggers.getInstance().isRobotApproachingTrench())
        .whileTrue(
            DriveCommands.joystickDriveAlignForTrench(
                drive,
                () -> Triggers.getInstance().simXSupplier(),
                () -> Triggers.getInstance().simYSupplier()));

    // Align for bump when bump button pressed - zone logic temporarily disabled
    Triggers.getInstance()
        .simBumpAlignButton()
        // .and(Triggers.getInstance().isRobotOnBump())
        // .or(Triggers.getInstance().isRobotApproachingBump())
        .whileTrue(
            DriveCommands.joystickDriveAlignForBump(
                drive,
                () -> Triggers.getInstance().simXSupplier(),
                () -> Triggers.getInstance().simYSupplier()));

    // UPPER SHOOTER
    // Set shooting velocity if shoot button pressed, we're in our alliance zone, hub is active, and
    // tuning false
    Triggers.getInstance()
        .simShootButton()
        .and(Triggers.getInstance().isShootClear)
        .and(() -> !HardwareConstants.TuningConstants.TUNING_MODE)
        .whileTrue(
            FlywheelCommands.shootOnTheMove(
                    flywheel, HoodPosCalculator.getInstance().getHoodPosForHub().magnitude())
                .alongWith(
                    PrestageCommands.setPrestageVelocity(
                        prestage, HardwareConstants.CompConstants.Velocities.prestageVelocity)))
        .onFalse(FlywheelCommands.stop(flywheel))
        .onFalse(PrestageCommands.stop(prestage));

    // // Run dynamic shoot sequence - same constraints as normal shoot, but different trigger
    // Triggers.getInstance()
    //     .simShootOnTheMove()
    //     .and(Triggers.getInstance().isShootClear)
    //     .and(() -> !HardwareConstants.TuningConstants.TUNING_MODE)
    //     .whileTrue(
    //         FlywheelCommands.shootOnTheMove(
    //             flywheel, HoodPosCalculator.getInstance().getHoodPosForHub().magnitude()));

    // Set passing velocity if shoot button is pressed but we're not in our alliance zone and tuning
    // false,
    // or if pass button is pressed
    (Triggers.getInstance()
            .simShootButton()
            .and(() -> !Triggers.getInstance().isShootSafeZone.getAsBoolean())
            .and(() -> !HardwareConstants.TuningConstants.TUNING_MODE))
        .or(Triggers.getInstance().simPassButton())
        .whileTrue(
            FlywheelCommands.setVelocityForPassing(flywheel)
                // FlywheelCommands.setFlywheelVelocity(
                //         flywheel, HardwareConstants.PassConstants.FlywheelPassVelocity)
                .alongWith(
                    PrestageCommands.setPrestageVelocity(
                        prestage, HardwareConstants.CompConstants.Velocities.prestageVelocity)))
        .onFalse(FlywheelCommands.stop(flywheel))
        .onFalse(PrestageCommands.stop(prestage));

    // LOWER SHOOTER
    // Set shooting velocity if shoot button pressed, we're in our alliance zone, hub is active,
    // tuning is false, flywheel is spun up, and we're aligned to shoot
    Triggers.getInstance()
        .simShootButton()
        .and(Triggers.getInstance().isShootClear)
        .and(() -> !HardwareConstants.TuningConstants.TUNING_MODE)
        .and(Triggers.getInstance().isAlignedForCurrentShot)
        .whileTrue(
            FeederCommands.setUpperFeederVelocity(
                    upperFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity)
                .alongWith(
                    FeederCommands.setLowerFeederVelocity(
                        lowerFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity))
                .alongWith(
                    TransportCommands.setTransportVelocity(
                        transport, HardwareConstants.CompConstants.Velocities.transportVelocity)))
        .onFalse(FeederCommands.stopLower(lowerFeeder))
        .onFalse(FeederCommands.stopUpper(upperFeeder))
        .onFalse(TransportCommands.stop(transport));

    // Set passing velocity if shoot button is pressed but we're not in our alliance zone and tuning
    // false,
    // or if pass button is pressed
    Triggers.getInstance()
        .simShootButton()
        .and(() -> !Triggers.getInstance().isShootSafeZone.getAsBoolean())
        .and(() -> !HardwareConstants.TuningConstants.TUNING_MODE)
        .and(Triggers.getInstance().isAlignedForCurrentShot)
        .whileTrue(
            FeederCommands.setUpperFeederVelocity(
                    upperFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity)
                .alongWith(
                    FeederCommands.setLowerFeederVelocity(
                        lowerFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity))
                .alongWith(
                    TransportCommands.setTransportVelocity(
                        transport, HardwareConstants.CompConstants.Velocities.transportVelocity)))
        .onFalse(FeederCommands.stopLower(lowerFeeder))
        .onFalse(FeederCommands.stopUpper(upperFeeder))
        .onFalse(TransportCommands.stop(transport));

    // FULL SHOOTER - hard-coded shots and tuning shots
    // These use constant waits instead of waiting for alignment or spinup
    // Hard-coded tower shot
    Triggers.getInstance()
        .simShootFromTowerButton()
        .whileTrue(
            FlywheelCommands.setFlywheelVelocity(
                    flywheel, HardwareConstants.TowerConstants.FlywheelTowerVelocity)
                .alongWith(
                    PrestageCommands.setPrestageVelocity(
                        prestage, HardwareConstants.CompConstants.Velocities.prestageVelocity))
                .alongWith(
                    FeederCommands.setUpperVelocityAfterWait(
                        upperFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity))
                .alongWith(
                    FeederCommands.setLowerVelocityAfterWait(
                        lowerFeeder, HardwareConstants.CompConstants.Velocities.feederVelocity))
                .alongWith(
                    TransportCommands.setVelocityAfterWait(
                        transport, HardwareConstants.CompConstants.Velocities.transportVelocity)))
        .onFalse(FlywheelCommands.stop(flywheel))
        .onFalse(PrestageCommands.stop(prestage))
        .onFalse(FeederCommands.stopLower(lowerFeeder))
        .onFalse(FeederCommands.stopUpper(upperFeeder))
        .onFalse(TransportCommands.stop(transport));

    // INTAKE ROLLER
    // Set to intaking voltage when intake button is pressed
    Triggers.getInstance()
        .simIntakeRollerButton()
        .whileTrue(
            intakeRollerCommands.setRollerVoltage(
                intakeRoller, HardwareConstants.CompConstants.Voltages.intakeRollerVoltage))
        .onFalse(intakeRollerCommands.stopIntakeRoller(intakeRoller));

    // Set to agitate voltage (after a wait) when any shooting sequence is started (shoot to hub,
    // pass, shoot from tower). Agitate is also delayed until aligned.
    Triggers.getInstance()
        .simShootButton()
        .or(Triggers.getInstance().passButton())
        .or(Triggers.getInstance().shootFromTowerButton())
        .whileTrue(
            intakeRollerCommands.setVoltageAfterWait(
                intakeRoller,
                HardwareConstants.CompConstants.Voltages.intakeRollerAgitateVoltage,
                Triggers.getInstance().isAlignedForCurrentShot))
        .onFalse(intakeRollerCommands.stopIntakeRoller(intakeRoller));

    // INTAKE PIVOT
    // Retract on retract button — also cancels automatic compression for this shoot press
    Triggers.getInstance()
        .simIntakeInButton()
        .onTrue(Commands.runOnce(() -> compressCancelled = true))
        .whileTrue(
            IntakePivotCommands.setPivotPosition(
                intakePivot, HardwareConstants.CompConstants.Positions.pivotUpPos));

    // Deploy on deploy button — also cancels automatic compression for this shoot press
    Triggers.getInstance()
        .simIntakeOutButton()
        .onTrue(Commands.runOnce(() -> compressCancelled = true))
        .whileTrue(
            IntakePivotCommands.setPivotPosition(
                intakePivot, HardwareConstants.CompConstants.Positions.pivotDownPos));

    // Manual compress button — cancels auto-compress and runs compress with no initial wait
    Triggers.getInstance()
        .simIntakeCompressButton()
        .onTrue(Commands.runOnce(() -> compressCancelled = true))
        .whileTrue(IntakePivotCommands.manualPivotCompress(intakePivot))
        .onFalse(
            IntakePivotCommands.setPivotPosition(
                intakePivot, HardwareConstants.CompConstants.Positions.pivotDownPos));

    // Automatic compress on shoot button when:
    //   - neither intake deploy nor retract button is currently pressed
    //   - compression has not been cancelled by a prior intake override this shoot press
    //   - we're not in our alliance zone with the hub inactive
    Triggers.getInstance()
        .simShootButton()
        .and(() -> !compressCancelled)
        .and(
            () ->
                !(Triggers.getInstance().isShootSafeZone.getAsBoolean()
                    && !Triggers.getInstance().isShootSafeTime.getAsBoolean()))
        .and(Triggers.getInstance().isAlignedForCurrentShot)
        .whileTrue(IntakePivotCommands.compressPivot(intakePivot))
        .onFalse(
            IntakePivotCommands.setPivotPosition(
                intakePivot, HardwareConstants.CompConstants.Positions.pivotDownPos));

    // HOOD
    // Set pos for hub if shoot to hub button or shoot to tower button is pressed, and we're in our
    // alliance zone and the hub is active, and tuning mode is false
    (Triggers.getInstance().simShootButton().or(Triggers.getInstance().simShootFromTowerButton()))
        .and(Triggers.getInstance().isShootClear)
        .and(() -> !HardwareConstants.TuningConstants.TUNING_MODE)
        .whileTrue(HoodCommands.setHoodPosForHub(hood));

    // Set pos for passing if shoot to hub button is pressed but we're not in our alliance zone, or
    // if pass button is presssed
    (Triggers.getInstance()
            .simShootButton()
            .and(() -> !Triggers.getInstance().isShootSafeZone.getAsBoolean()))
        .and(() -> !HardwareConstants.TuningConstants.TUNING_MODE)
        .or(Triggers.getInstance().simPassButton())
        .whileTrue(HoodCommands.setPosForPassing(hood))
    // .whileTrue(HoodCommands.setHoodPos(hood, HardwareConstants.PassConstants.hoodPassPos))
    ;

    // Auto-x and auto-compress cancellations
    Triggers.getInstance()
        .shootButton()
        .onFalse(Commands.runOnce(() -> compressCancelled = false))
        .onFalse(Commands.runOnce(() -> xCancelled = false));
  }

  public Command getAutonomousCommand() {
    double delay = SmartDashboard.getNumber(autoDelayKey, defaultAutoDelay);
    return Commands.sequence(Commands.waitSeconds(delay), autoChooser.get().asProxy());
  }

  // ==================== AUTO PREVIEW & STARTING POSE CHECK ====================

  /** Tracks the last auto name so we only reload paths when the selection changes. */
  private String lastAutoName = "";

  /**
   * Updates the auto path preview on the Field2d when the selected auto changes.
   *
   * <p>Call this periodically (e.g., from {@code disabledPeriodic}). It reads the currently
   * selected auto command's name, loads all paths from that auto file, and draws them on the "Auto
   * Preview" Field2d widget.
   */
  public void updateAutoPreview() {
    // Get the currently selected auto command
    Command selectedAuto = autoChooser.get();
    if (selectedAuto == null) return;

    String autoName = selectedAuto.getName();

    // Only reload if the selection changed
    if (autoName.equals(lastAutoName)) return;
    lastAutoName = autoName;

    Logger.recordOutput("Auto/SelectedAuto", autoName);

    // Clear any previously drawn paths
    autoPreviewField.getObject("path").setPoses();

    try {
      // Load all paths from the selected auto and draw them on the field
      List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);

      if (paths.isEmpty()) {
        Logger.recordOutput("Auto/PreviewStatus", "No paths found for: " + autoName);
        autoStartPose = new Pose2d();
        return;
      }

      // Collect all path poses for visualization
      List<Pose2d> allPoses = new java.util.ArrayList<>();
      for (PathPlannerPath path : paths) {
        // Flip the path for red alliance if needed
        PathPlannerPath flippedPath = path.flipPath();
        boolean shouldFlip = AllianceFlipUtil.shouldFlip();

        PathPlannerPath displayPath = shouldFlip ? flippedPath : path;
        allPoses.addAll(displayPath.getPathPoses());
      }

      // Draw all path poses on the Field2d
      autoPreviewField.getObject("path").setPoses(allPoses);

      // Store the starting pose (first point of the first path)
      autoStartPose = paths.get(0).getStartingHolonomicPose().orElse(new Pose2d());
      if (AllianceFlipUtil.shouldFlip()) {
        autoStartPose = AllianceFlipUtil.apply(autoStartPose);
      }

      Logger.recordOutput("Auto/PreviewStatus", "Loaded " + paths.size() + " paths");
      Logger.recordOutput("Auto/StartPose", autoStartPose);

    } catch (Exception e) {
      Logger.recordOutput("Auto/PreviewStatus", "Error loading: " + e.getMessage());
      autoStartPose = new Pose2d();
    }
  }

  /**
   * Checks and displays the robot's starting pose accuracy relative to the selected autonomous
   * path.
   *
   * <p>This method should be called periodically while the robot is disabled so the drive team can
   * verify the robot is placed correctly before a match. It publishes:
   *
   * <ul>
   *   <li>The robot's current pose on the auto preview Field2d
   *   <li>Distance (in inches) from the auto's starting position
   *   <li>Rotation difference (in degrees) from the auto's starting heading
   *   <li>Boolean flags indicating if position and rotation are within tolerance
   * </ul>
   */
  public void checkStartPose() {
    // Show the robot's current pose on the auto preview field
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    autoPreviewField.setRobotPose(currentPose);

    // Skip checks if we don't have a valid start pose
    if (autoStartPose.equals(new Pose2d())) {
      Logger.recordOutput("Auto/StartCheck/PositionOK", false);
      Logger.recordOutput("Auto/StartCheck/RotationOK", false);
      return;
    }

    // Calculate distance from current pose to auto start pose (in inches)
    // getDistance() returns meters, so convert meters → inches
    double distanceInches =
        metersToInches(currentPose.getTranslation().getDistance(autoStartPose.getTranslation()));

    // Calculate rotation difference (in degrees)
    double rotationDifferenceDegrees =
        Math.abs(currentPose.getRotation().minus(autoStartPose.getRotation()).getDegrees());

    // Check if within tolerance
    boolean positionOK = distanceInches <= STARTING_POSE_DRIVE_TOLERANCE.in(Inches);
    boolean rotationOK = rotationDifferenceDegrees <= STARTING_POSE_ROT_TOLERANCE_DEGREES;

    // Log results to dashboard
    Logger.recordOutput("Auto/StartCheck/DistanceInches", distanceInches);
    Logger.recordOutput("Auto/StartCheck/RotationDiffDegrees", rotationDifferenceDegrees);
    Logger.recordOutput("Auto/StartCheck/PositionOK", positionOK);
    Logger.recordOutput("Auto/StartCheck/RotationOK", rotationOK);
  }

  public Command getAutoStopCommand() {
    return ShootSequences.stopAll(
        flywheel, prestage, hood, upperFeeder, lowerFeeder, transport, intakeRoller);
  }

  public Command getIntakeRollerCommand() {
    return intakeRollerCommands.setRollerVoltage(
        intakeRoller, HardwareConstants.CompConstants.Voltages.intakeRollerVoltage);
  }

  public Command getIntakePivotCommand() {
    return IntakePivotCommands.setPivotPosition(
        intakePivot, HardwareConstants.CompConstants.Positions.pivotDownPos);
  }

  public boolean isFlywheelSpunUp() {
    return flywheel.isSpunUp();
  }

  public boolean isDriveXed() {
    return drive.areWheelsXed;
  }

  public boolean aligningDefensively() {
    return drive.aligningDefensively;
  }
}
