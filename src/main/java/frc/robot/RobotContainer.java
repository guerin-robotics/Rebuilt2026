// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.math.util.Units.metersToInches;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.AllianceFlipUtil;
import frc.lib.FieldConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeederCommands;
import frc.robot.commands.FlywheelCommands;
import frc.robot.commands.HoodCommands;
import frc.robot.commands.IntakePivotCommands;
import frc.robot.commands.PrestageCommands;
import frc.robot.commands.ShootSequences;
import frc.robot.commands.SpitSequences;
import frc.robot.commands.TransportCommands;
import frc.robot.commands.intakeRollerCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.io.FeederIO;
import frc.robot.subsystems.feeder.io.FeederIOReal;
import frc.robot.subsystems.feeder.io.FeederIOSim;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.io.FlywheelIO;
import frc.robot.subsystems.flywheel.io.FlywheelIOPhoenix6;
import frc.robot.subsystems.flywheel.io.FlywheelIOSim;
import frc.robot.subsystems.hood.Hood;
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
import frc.robot.subsystems.prestage.Prestage;
import frc.robot.subsystems.prestage.io.PrestageIO;
import frc.robot.subsystems.prestage.io.PrestageIOReal;
import frc.robot.subsystems.prestage.io.PrestageIOSim;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.transport.io.TransportIO;
import frc.robot.subsystems.transport.io.TransportIOReal;
import frc.robot.subsystems.transport.io.TransportIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.io.VisionIO;
import frc.robot.subsystems.vision.io.VisionIOPhotonVision;
import frc.robot.subsystems.vision.io.VisionIOPhotonVisionSim;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Flywheel flywheel;
  private final Feeder feeder;
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

  // Stores the starting pose of the currently selected auto.
  // Updated when the auto chooser selection changes.
  private Pose2d autoStartPose = new Pose2d();

  // ── Starting Pose Tolerances ────────────────────────────────────────────────
  // How close (in inches) the robot needs to be to the auto's starting position
  // for us to consider it "close enough" to start the match.
  private static final Distance STARTING_POSE_DRIVE_TOLERANCE = Inches.of(6.0);

  // How close (in degrees) the robot's heading needs to be to the auto's starting heading.
  private static final double STARTING_POSE_ROT_TOLERANCE_DEGREES = 5.0;

  // Controllers
  private final CommandXboxController controller =
      new CommandXboxController(HardwareConstants.ControllerConstants.XboxControllerPort);
  private final CommandJoystick thrustmaster =
      new CommandJoystick(HardwareConstants.ControllerConstants.JoystickControllerPort);

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
        feeder = new Feeder(new FeederIOReal());
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
        feeder = new Feeder(new FeederIOSim());
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
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
        feeder = new Feeder(new FeederIO() {});
        prestage = new Prestage(new PrestageIO() {});
        hood = new Hood(new HoodIO() {});
        transport = new Transport(new TransportIO() {});
        intakePivot = new IntakePivot(new IntakePivotIO() {});
        intakeRoller = new intakeRoller(new intakeRollerIO() {});
        break;
    }

    // IMPORTANT: Register named commands and event triggers BEFORE building the auto chooser.
    // AutoBuilder.buildAutoChooser() parses the .auto files and resolves named commands at
    // build time. If commands aren't registered yet, they resolve to Commands.none().
    registerNamedCommands();
    registerEventTriggers();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Publish the auto preview field to the dashboard so we can see the selected path
    SmartDashboard.putData("Auto Preview", autoPreviewField);

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
      configureButtonBindings();
    } else if (Robot.isSimulation()) {
      configureSimBindings();
    }
  }

  private double deadband(double value) {
    return MathUtil.applyDeadband(value, HardwareConstants.ControllerConstants.DEADBAND);
  }

  private double getThrustX() {
    return thrustmaster.getRawAxis(0); // strafe
  }

  private double getThrustY() {
    return thrustmaster.getRawAxis(1); // forward
  }

  private double getThrustRot() {
    return thrustmaster.getRawAxis(2); // twist
  }

  // NamedCommands
  private void registerNamedCommands() {
    // Auto deploy intake command
    NamedCommands.registerCommand(
        "DeployIntake",
        IntakePivotCommands.setPivotRotations(
            intakePivot, HardwareConstants.CompConstants.Positions.pivotDownPos));

    // Auto retract intake command
    NamedCommands.registerCommand(
        "RetractIntake",
        IntakePivotCommands.setPivotRotations(
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
                new WaitCommand(0.1)
                    .andThen(
                        ShootSequences.shootToHub(
                            flywheel,
                            prestage,
                            hood,
                            feeder,
                            transport,
                            intakeRoller,
                            intakePivot))));

    // Stop all subsystems after shooting
    NamedCommands.registerCommand(
        "stopAll",
        ShootSequences.stopAll(flywheel, prestage, hood, feeder, transport, intakeRoller));
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

    // Event marker for running intake rollers + transport while in a zoned area
    // Uses whileTrue because this is a zoned event marker (has start AND end positions in the path)
    // No subsystem requirements declared to avoid interrupting the auto command group
    new EventTrigger("RunIntake")
        .whileTrue(
            Commands.startEnd(
                () -> {
                  intakeRoller.setRollerVoltage(
                      HardwareConstants.CompConstants.Voltages.intakeRollerVoltage);
                  transport.setTransportVoltage(
                      HardwareConstants.CompConstants.Voltages.transportVoltage);
                },
                () -> {
                  intakeRoller.setRollerVoltage(Volts.of(0));
                  transport.setTransportVoltage(Volts.of(0));
                }));

    // Event marker for setting the hood position to down
    new EventTrigger("HoodDown")
        .onTrue(
            HoodCommands.setHoodPos(hood, HardwareConstants.CompConstants.Positions.hoodDownPos));
  }

  private void configureButtonBindings() {
    // ==================== DRIVE CONTROLS (DO NOT MODIFY) ====================
    // Default command: Xbox + Thrustmaster combined
    drive.setDefaultCommand(
        DriveCommands.driveLucasProof(
            drive,
            () -> MathUtil.clamp(-controller.getLeftY() - getThrustY(), -1.0, 1.0),
            () -> MathUtil.clamp(-controller.getLeftX() - getThrustX(), -1.0, 1.0),
            () -> MathUtil.clamp(-controller.getRightX() - getThrustRot(), -1.0, 1.0),
            controller.y()));
    // Lock to 0° when A button is held (Xbox still controls angle)
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> MathUtil.clamp(controller.getLeftY() + getThrustY(), -1.0, 1.0),
                () -> MathUtil.clamp(controller.getLeftX() + getThrustX(), -1.0, 1.0),
                () -> Rotation2d.kZero));
    // X pattern
    controller.a().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    controller
        .rightBumper()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> MathUtil.clamp(controller.getLeftY() + getThrustY(), -1.0, 1.0),
                () -> MathUtil.clamp(controller.getLeftX() + getThrustX(), -1.0, 1.0),
                () -> RobotState.getInstance().getAngleToAllianceHub()));

    // ==================== SUBSYSTEM CONTROLS ====================

    // REVISED SUBSYSTEM CONTROLS

    // Default commands
    if (!HardwareConstants.TuningConstants.TUNING_MODE) {
      // Flywheel (5 rps)
      flywheel.setDefaultCommand(FlywheelCommands.flywheelIdle(flywheel));
      // Hood (down)
      hood.setDefaultCommand(
          HoodCommands.setHoodPos(hood, HardwareConstants.CompConstants.Positions.hoodDownPos));
      // Prestage, by voltage
      prestage.setDefaultCommand(PrestageCommands.setPrestageVoltage(prestage, Volts.of(-1)));
      // Feeder, by voltage
      feeder.setDefaultCommand(FeederCommands.setFeederVoltage(feeder, Volts.of(-1)));
    }

    if (!HardwareConstants.TuningConstants.TUNING_MODE) {
      // Distance-based shooting
      thrustmaster
          .button(1)
          .whileTrue(
              DriveCommands.joystickDriveAtAngle(
                      drive,
                      () -> -thrustmaster.getY(),
                      () -> -thrustmaster.getX(),
                      () -> flywheel.getShootAngleForZone())
                  .alongWith(
                      ShootSequences.zonePassOrShoot(
                          flywheel, prestage, hood, feeder, transport, intakeRoller, intakePivot)))
          .onFalse(
              ShootSequences.shootEndBehavior(
                  flywheel, prestage, hood, feeder, transport, intakeRoller, intakePivot));
    } else {
      // Shoot for map tuning
      thrustmaster
          .button(1)
          .whileTrue(
              DriveCommands.joystickDriveAtAngle(
                      drive,
                      () -> -thrustmaster.getX(),
                      () -> -thrustmaster.getY(),
                      () -> RobotState.getInstance().getAngleToAllianceHub())
                  .alongWith(
                      ShootSequences.mapTuningShoot(
                          flywheel, prestage, hood, intakePivot, feeder, transport, intakeRoller)));
    }

    // Align for sweep
    thrustmaster
        .button(2)
        .whileTrue(
            DriveCommands.joystickDriveAlignForSweepToAllianceZone(
                drive, () -> -thrustmaster.getX(), () -> -thrustmaster.getY()));

    // Intake up
    thrustmaster
        .button(3)
        .whileTrue(
            IntakePivotCommands.setPivotRotations(
                intakePivot, HardwareConstants.CompConstants.Positions.pivotUpPos));

    // Intake down
    thrustmaster
        .button(4)
        .whileTrue(
            IntakePivotCommands.setPivotRotations(
                intakePivot, HardwareConstants.CompConstants.Positions.pivotDownPos));

    // Run intake roller
    thrustmaster
        .button(5)
        .whileTrue(
            intakeRollerCommands.setRollerVoltage(
                intakeRoller, HardwareConstants.CompConstants.Voltages.intakeRollerVoltage)
            // .alongWith(
            //     TransportCommands.setTransportVoltage(
            //         transport, HardwareConstants.TestVoltages.TransportTestVoltage))
            );

    // Intake jostle
    // thrustmaster.button(6).whileTrue(IntakePivotCommands.jostlePivotByPos(intakePivot));
    thrustmaster.button(6).onTrue(IntakePivotCommands.setPivotRotations(intakePivot, 0.0));

    // Spit sequence
    thrustmaster.button(7).whileTrue(SpitSequences.clearShooter(flywheel, prestage, feeder));

    // Shoot from tower
    thrustmaster
        .button(10)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -thrustmaster.getX(),
                    () -> -thrustmaster.getY(),
                    () -> RobotState.getInstance().getAngleToAllianceHub())
                .alongWith(
                    new WaitCommand(0.5)
                        .andThen(
                            ShootSequences.shootForTower(
                                flywheel, prestage, hood, feeder, transport, intakeRoller))))
        .onFalse(
            ShootSequences.shootEndBehavior(
                flywheel, prestage, hood, feeder, transport, intakeRoller, intakePivot));

    // Pass
    thrustmaster
        .button(11)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -thrustmaster.getX(),
                    () -> -thrustmaster.getY(),
                    () ->
                        RobotState.getInstance()
                            .getAngleToTarget(
                                new Translation2d(
                                    flywheel.getPassTarget().getX(),
                                    flywheel.getPassTarget().getY())))
                .alongWith(
                    new WaitCommand(0.5)
                        .andThen(
                            ShootSequences.pass(
                                flywheel,
                                prestage,
                                hood,
                                feeder,
                                transport,
                                intakePivot,
                                intakeRoller))))
        .onFalse(
            ShootSequences.shootEndBehavior(
                flywheel, prestage, hood, feeder, transport, intakeRoller, intakePivot));

    // Align for bump
    thrustmaster
        .button(12)
        .whileTrue(
            DriveCommands.joystickDriveAlignForBump(
                drive, () -> -thrustmaster.getX(), () -> -thrustmaster.getY()));

    // Drop hood
    // controller
    //     .y()
    //     .onTrue(HoodCommands.setHoodPos(hood,
    // HardwareConstants.CompConstants.Positions.hoodDownPos));

    // Bump hood pos up
    controller.leftBumper().onTrue(HoodCommands.incrementHoodPos(hood));

    // controller
    //     .y()
    //     .whileTrue(
    //         intakeRollerCommands.setRollerVelocity(
    //             intakeRoller, HardwareConstants.TestConstants.TestVelocities.rollerVelocity));
  }

  private void configureSimBindings() {
    // ==================== DRIVE (SIM) ====================
    drive.setDefaultCommand(
        DriveCommands.driveLucasProof(
            drive,
            () -> MathUtil.clamp(-controller.getLeftY(), -1.0, 1.0),
            () -> MathUtil.clamp(-controller.getLeftX(), -1.0, 1.0),
            () -> MathUtil.clamp(-controller.getRightTriggerAxis(), -1.0, 1.0),
            controller.y()));

    // ==================== FLYWHEEL TEST (SIM) ====================
    // Set flywheel idle as default command so it's always spinning slowly
    flywheel.setDefaultCommand(FlywheelCommands.flywheelIdle(flywheel));

    // A button: full shoot sequence (drive-at-angle + shoot to hub)
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                    drive, () -> 0.0, () -> 0.0, () -> flywheel.getShootAngleForZone())
                .alongWith(
                    new WaitCommand(0.5)
                        .andThen(
                            ShootSequences.shootToHub(
                                flywheel,
                                prestage,
                                hood,
                                feeder,
                                transport,
                                intakeRoller,
                                intakePivot))))
        .onFalse(SpitSequences.spitAfterShoot(flywheel, prestage, feeder, transport, intakeRoller));

    // B button maps correctly
    controller
        .b()
        .whileTrue(
            IntakePivotCommands.setPivotRotations(
                intakePivot, HardwareConstants.CompConstants.Positions.pivotDownPos));

    // Left bumper: intake down
    controller
        .leftBumper()
        .whileTrue(
            IntakePivotCommands.setPivotRotations(
                intakePivot, HardwareConstants.CompConstants.Positions.pivotDownPos));

    // Right bumper: intake up
    controller
        .rightBumper()
        .whileTrue(
            IntakePivotCommands.setPivotRotations(
                intakePivot, HardwareConstants.CompConstants.Positions.pivotUpPos));

    // ==================== INTAKE ROLLER + TRANSPORT TEST (SIM) ====================
    // X button: run intake roller + transport together (simulates intaking a ball)
    controller
        .x()
        .whileTrue(
            intakeRollerCommands
                .setRollerVoltage(
                    intakeRoller, HardwareConstants.CompConstants.Voltages.intakeRollerVoltage)
                .alongWith(
                    TransportCommands.setTransportVoltage(
                        transport, HardwareConstants.CompConstants.Voltages.transportVoltage)));

    // ==================== PRESTAGE + FEEDER TEST (SIM) ====================
    // Y button: run prestage and feeder at velocity setpoints
    controller
        .y()
        .whileTrue(
            PrestageCommands.setPrestageVelocity(
                    prestage, HardwareConstants.CompConstants.Velocities.prestageVelocity)
                .alongWith(
                    FeederCommands.setFeederVelocity(
                        feeder, HardwareConstants.CompConstants.Velocities.feederVelocity)))
        .onFalse(
            PrestageCommands.stop(prestage)
                .alongWith(FeederCommands.setFeederVelocity(feeder, RotationsPerSecond.of(0))));

    // ==================== SPIT TEST (SIM) ====================
    // Start button: spit all
    controller
        .start()
        .whileTrue(SpitSequences.spitAll(flywheel, prestage, feeder, transport, intakeRoller));

    // ==================== RESET POSE (SIM) ====================
    // Back button: reset pose to field corner
    controller
        .back()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            AllianceFlipUtil.apply(
                                new Pose2d(
                                    new Translation2d(
                                        (inchesToMeters(33 / 2)),
                                        (FieldConstants.fieldWidth - inchesToMeters(33 / 2))),
                                    drive.getRotation()))))
                .ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
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
    return ShootSequences.stopAll(flywheel, prestage, hood, feeder, transport, intakeRoller);
  }
}
