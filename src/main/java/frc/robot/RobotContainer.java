// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeederCommands;
import frc.robot.commands.FlywheelCommands;
import frc.robot.commands.HoodCommands;
import frc.robot.commands.PrestageCommands;
import frc.robot.commands.TransportCommands;
import frc.robot.commands.intakeRollerCommands;
import frc.robot.commands.intakeSliderCommands;
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
import frc.robot.subsystems.intakeRoller.intakeRoller;
import frc.robot.subsystems.intakeRoller.io.intakeRollerIO;
import frc.robot.subsystems.intakeRoller.io.intakeRollerIOReal;
import frc.robot.subsystems.intakeRoller.io.intakeRollerIOSim;
import frc.robot.subsystems.intakeSlider.intakeSlider;
import frc.robot.subsystems.intakeSlider.io.intakeSliderIO;
import frc.robot.subsystems.intakeSlider.io.intakeSliderIOReal;
import frc.robot.subsystems.intakeSlider.io.intakeSliderIOSim;
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
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Flywheel flywheel;
  private final Feeder feeder;
  private final Hood hood;
  private final Prestage prestage;
  private final intakeSlider intakeSlider;
  private final intakeRoller intakeRoller;
  private final Transport transport;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Controllers
  private final CommandXboxController controller =
      new CommandXboxController(HardwareConstants.ControllerConstants.XboxControllerPort);
  private final CommandJoystick thrustmaster =
      new CommandJoystick(HardwareConstants.ControllerConstants.JoystickControllerPort);
  private final CommandJoystick buttonPanel =
      new CommandJoystick(HardwareConstants.ControllerConstants.ButtonPanelPort);

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
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1));
        flywheel = new Flywheel(new FlywheelIOPhoenix6());
        feeder = new Feeder(new FeederIOReal());
        hood = new Hood(new HoodIOReal());
        prestage = new Prestage(new PrestageIOReal());
        transport = new Transport(new TransportIOReal());
        intakeSlider = new intakeSlider(new intakeSliderIOReal());
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
        intakeSlider = new intakeSlider(new intakeSliderIOSim());
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
        intakeSlider = new intakeSlider(new intakeSliderIO() {});
        intakeRoller = new intakeRoller(new intakeRollerIO() {});
        break;
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
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

    configureButtonBindings();
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

  private void configureButtonBindings() {
    // ==================== DRIVE CONTROLS (DO NOT MODIFY) ====================
    // Default command: Xbox + Thrustmaster combined
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> MathUtil.clamp(controller.getLeftY() + getThrustY(), -1.0, 1.0),
            () -> MathUtil.clamp(controller.getLeftX() + getThrustX(), -1.0, 1.0),
            () -> MathUtil.clamp(-controller.getRightX() - getThrustRot(), -1.0, 1.0)));
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
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

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

    // Manually set the odometry from starting (0,0) to in front of the hub
    // (Used for developing distance-based shooting without vision)
    controller
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(new Translation2d(3, 4), drive.getPose().getRotation())),
                    drive)
                .ignoringDisable(true));

    // ==================== SUBSYSTEM CONTROLS ====================

    // REVISED SUBSYSTEM CONTROLS

    // Set idle command (run at 10 rps) as default
    // flywheel.setDefaultCommand(FlywheelCommands.flywheelIdle(flywheel));

    // Single-button shoot sequence.
    // Joystick trigger button.
    // Wide-range (non-dynamic) shooting and hood positioning w/ flywheel/prestage linked and
    // feeder/transport waiting for velocity checker, intake jostle command currently disabled
    // Out of commission until distance-RPM and distance-angle maps exist
    // thrustmaster
    //     .button(1)
    //     .onTrue(
    //         FlywheelCommands.setVelocityForHub(flywheel)
    //         .alongWith(PrestageCommands.setPrestageVelocity(prestage,
    //             HardwareConstants.TestVelocities.prestageVelocity))
    //         .alongWith(HoodCommands.setHoodPosForHub(hood))
    //         .alongWith(FeederCommands.setVelocityAtRPM(feeder,
    //             HardwareConstants.TestVelocities.feederVelocity,
    //
    // flywheel.isFlywheelAtVelocity(ShotCalculator.getInstance().getFlywheelSpeedForAllianceHub())))
    //         .alongWith(TransportCommands.setVelocityAtRPM(transport,
    //             HardwareConstants.TestVelocities.transportVelocity,
    //
    // flywheel.isFlywheelAtVelocity(ShotCalculator.getInstance().getFlywheelSpeedForAllianceHub())))
    //         // .alongWith(intakeSliderCommands.jostleSliderByCurrent(intakeSlider,
    //         //     HardwareConstants.TestVelocities.sliderUpVelocity,
    //         //     HardwareConstants.TestVelocities.sliderDownVelocity,
    //         //     HardwareConstants.TestPositions.intakeDegreesDownTest,
    //         //     HardwareConstants.TestPositions.pulseSeconds))
    //     );

    // Shoot from tower
    thrustmaster
        .button(1)
        .onTrue(
            FlywheelCommands.setFlywheelVelocity(
                    flywheel, HardwareConstants.TowerConstants.FlywheelTowerVelocity)
                .alongWith(
                    HoodCommands.setHoodPos(hood, HardwareConstants.TowerConstants.hoodTowerPos))
                .alongWith(
                    PrestageCommands.setPrestageVelocity(
                        prestage, HardwareConstants.TestVelocities.prestageVelocity))
                .alongWith(
                    FeederCommands.setVelocityAtRPM(
                        feeder,
                        HardwareConstants.TestVelocities.feederVelocity,
                        flywheel.isFlywheelAtVelocity(
                            HardwareConstants.TowerConstants.FlywheelTowerVelocity)))
                .alongWith(
                    TransportCommands.setVelocityAtRPM(
                        transport,
                        HardwareConstants.TestVelocities.transportVelocity,
                        flywheel.isFlywheelAtVelocity(
                            HardwareConstants.TowerConstants.FlywheelTowerVelocity)))
            // .alongWith(
            //     intakeSliderCommands.jostleSliderByCurrent(
            //         intakeSlider,
            //         HardwareConstants.TestVelocities.sliderUpVelocity,
            //         HardwareConstants.TestVelocities.sliderDownVelocity,
            //         HardwareConstants.TestPositions.intakeDegreesDownTest,
            //         HardwareConstants.TestPositions.pulseSeconds))
            );

    // Intake up
    thrustmaster
        .button(3)
        .whileTrue(
            intakeSliderCommands.setSliderRotations(
                intakeSlider, HardwareConstants.TestPositions.intakeDegreesUpTest));

    // Intake down
    thrustmaster
        .button(4)
        .whileTrue(
            intakeSliderCommands.setSliderRotations(
                intakeSlider, HardwareConstants.TestPositions.intakeDegreesDownTest));

    // Run intake roller
    thrustmaster
        .button(5)
        .whileTrue(
            intakeRollerCommands.setRollerVoltage(
                intakeRoller, HardwareConstants.TestVoltages.intakeRollerTestVoltage));

    // Intake jostle
    // thrustmaster
    //     .button(6)
    //     .whileTrue(
    //         intakeSliderCommands.jostleSliderByCurrent(
    //             intakeSlider,
    //             HardwareConstants.TestVelocities.sliderUpVelocity,
    //             HardwareConstants.TestVelocities.sliderDownVelocity,
    //             HardwareConstants.TestPositions.intakeDegreesDownTest,
    //             HardwareConstants.TestPositions.pulseSeconds));

    // Spit sequence
    thrustmaster
        .button(7)
        .whileTrue(
            FlywheelCommands.setFlywheelVelocity(
                    flywheel, HardwareConstants.SpitVelocities.FlywheelSpitVelocity)
                .alongWith(
                    PrestageCommands.setPrestageVelocity(
                        prestage, HardwareConstants.SpitVelocities.prestageSpitVelocity))
                .alongWith(
                    FeederCommands.setFeederVelocity(
                        feeder, HardwareConstants.SpitVelocities.feederSpitVelocity))
                .alongWith(
                    TransportCommands.setTransportVelocity(
                        transport, HardwareConstants.SpitVelocities.transportSpitVelocity))
                .alongWith(
                    intakeRollerCommands.setRollerVelocity(
                        intakeRoller, HardwareConstants.SpitVelocities.rollerSpitVelocity)));

    // Lock to heading calculated by dynamic shoot vectors when A button is held (Xbox still
    // controls angle)
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> MathUtil.clamp(controller.getLeftY() + getThrustY(), -1.0, 1.0),
                () -> MathUtil.clamp(controller.getLeftX() + getThrustX(), -1.0, 1.0),
                () -> drive.getHeadingForShootDynamic()));

    // Basic controls for testing
    // Flywheel, hood, and prestage
    buttonPanel
        .button(1)
        .whileTrue(
            FlywheelCommands.setFlywheelVelocity(
                    flywheel, HardwareConstants.TestVelocities.FlywheelVelocity)
                .alongWith(
                    PrestageCommands.setPrestageVelocity(
                        prestage, HardwareConstants.TestVelocities.prestageVelocity))
                .alongWith(
                    HoodCommands.setHoodPos(hood, HardwareConstants.TestPositions.hoodPos3Test)));
    // Transport and feeder
    buttonPanel
        .button(2)
        .whileTrue(
            FeederCommands.setFeederVelocity(
                    feeder, HardwareConstants.TestVelocities.feederVelocity)
                .alongWith(
                    TransportCommands.setTransportVelocity(
                        transport, HardwareConstants.TestVelocities.transportVelocity)));

    // Move hood
    buttonPanel
        .button(3)
        .onTrue(HoodCommands.setHoodPos(hood, HardwareConstants.TestPositions.hoodPos2Test));

    // Pivot intake - 4 for up, 5 for down
    // buttonPanel
    //     .button(4)
    //     .whileTrue(
    //         intakeSliderCommands.setSliderVoltage(
    //             intakeSlider, HardwareConstants.TestVoltages.intakeSliderTestVoltageUp));
    // buttonPanel
    //     .button(4)
    //     .whileTrue(
    //         intakeSliderCommands.setSliderRotations(
    //             intakeSlider, HardwareConstants.TestPositions.intakeDegreesUpTest));
    // buttonPanel
    //     .button(5)
    //     .whileTrue(
    //         intakeSliderCommands.setSliderVoltage(
    //             intakeSlider, HardwareConstants.TestVoltages.intakeSliderTestVoltageDown));
    // buttonPanel
    //     .button(5)
    //     .whileTrue(
    //         intakeSliderCommands.setSliderRotations(
    //             intakeSlider, HardwareConstants.TestPositions.intakeDegreesDownTest));

    // Run roller
    buttonPanel
        .button(6)
        .whileTrue(
            intakeRollerCommands.setRollerVoltage(
                intakeRoller, HardwareConstants.TestVoltages.intakeRollerTestVoltage));

    // Controls for testing distance-based shooting
    // Set hood pos based on distance from hub
    buttonPanel.button(7).onTrue(HoodCommands.setHoodPosForHub(hood));
    // Set flywheel velocity based on distance from hub
    buttonPanel.button(8).whileTrue(FlywheelCommands.setVelocityForHub(flywheel));

    // Intake spit
    buttonPanel
        .button(9)
        .whileTrue(
            intakeRollerCommands.setRollerVoltage(
                intakeRoller, HardwareConstants.SpitVelocities.rollerSpitVolts));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
