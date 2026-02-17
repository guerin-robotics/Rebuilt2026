// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeederCommands;
import frc.robot.commands.FlywheelCommands;
import frc.robot.commands.PrestageCommands;
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
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.io.FlywheelIO;
import frc.robot.subsystems.flywheel.io.FlywheelIOPhoenix6;
import frc.robot.subsystems.intakeRoller.intakeRoller;
import frc.robot.subsystems.intakeRoller.io.intakeRollerIO;
import frc.robot.subsystems.intakeRoller.io.intakeRollerIOReal;
import frc.robot.subsystems.intakeSlider.intakeSlider;
import frc.robot.subsystems.intakeSlider.io.intakeSliderIO;
import frc.robot.subsystems.intakeSlider.io.intakeSliderIOReal;
import frc.robot.subsystems.prestage.Prestage;
import frc.robot.subsystems.prestage.io.PrestageIO;
import frc.robot.subsystems.prestage.io.PrestageIOReal;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.transport.io.TransportIO;
import frc.robot.subsystems.transport.io.TransportIOReal;
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
  private final Flywheel shooter;
  private final Feeder feeder;
  private final Prestage prestage;
  private final intakeSlider intakeSlider;
  private final intakeRoller intakeRoller;
  private final Transport transport;

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);
  private final Joystick thrustmaster = new Joystick(1);
  private final CommandJoystick buttonPanel = new CommandJoystick(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private static final double DEADBAND = 0.08;

  // Shooter voltage for button control (tunable)
  private static final double SHOOTER_TEST_VOLTAGE = 6.0; // Volts

  // Feeder test voltage
  private static final double FeederTestVoltage = 3.0;
  private static final double PrestageTestVoltage = 3.0;
  private static final double TransportTestVoltage = 3.0;
  private static final double intakeSliderTestVoltage = 3.0;
  private static final double intakeRollerTestVoltage = 3.0;

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
        shooter = new Flywheel(new FlywheelIOPhoenix6());
        feeder = new Feeder(new FeederIOReal());
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
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));
        shooter = new Flywheel(new FlywheelIO() {});
        feeder = new Feeder(new FeederIO() {});
        prestage = new Prestage(new PrestageIO() {});
        transport = new Transport(new TransportIO() {});
        intakeSlider = new intakeSlider(new intakeSliderIO() {});
        intakeRoller = new intakeRoller(new intakeRollerIO() {});
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
        shooter = new Flywheel(new FlywheelIO() {});
        feeder = new Feeder(new FeederIO() {});
        prestage = new Prestage(new PrestageIO() {});
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
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    configureButtonBindings();
  }

  private double deadband(double value) {
    return MathUtil.applyDeadband(value, DEADBAND);
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
            () -> MathUtil.clamp(-controller.getLeftY() + -getThrustY(), -1.0, 1.0),
            () -> MathUtil.clamp(-controller.getLeftX() + -getThrustX(), -1.0, 1.0),
            () -> MathUtil.clamp(-controller.getRightX() + -getThrustRot(), -1.0, 1.0)));
    // Lock to 0° when A button is held (Xbox still controls angle)
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> MathUtil.clamp(-controller.getLeftY() + -getThrustY(), -1.0, 1.0),
                () -> MathUtil.clamp(-controller.getLeftX() + -getThrustX(), -1.0, 1.0),
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

    // ==================== SHOOTER CONTROLS (BUTTON PANEL) ====================
    // Button 1: Run shooter at test voltage while held, stop when released
    buttonPanel
        .button(1)
        .whileTrue(FlywheelCommands.runVoltage(shooter, Volts.of(SHOOTER_TEST_VOLTAGE)));
    // Button 2: Stop shooter immediately (safety)
    buttonPanel.button(2).onTrue(FlywheelCommands.stop(shooter));

    // FEEDER CONTROLS
    // Button 3 (label "L1"): Run feeder
    buttonPanel
        .button(3)
        .whileTrue(FeederCommands.runFeederVoltage(feeder, Volts.of(FeederTestVoltage)));
    // Button 7 (Label "OUT"): Stop feeder
    buttonPanel.button(7).onTrue(FeederCommands.stop(feeder));

    // PRESTAGE CONTROLS
    // Button 4: Run prestage
    buttonPanel
        .button(4)
        .whileTrue(PrestageCommands.runPrestageVoltage(prestage, Volts.of(PrestageTestVoltage)));
    // Button 5: Stop prestage
    buttonPanel.button(5).onTrue(PrestageCommands.stop(prestage));

    // INTAKE CONTROLS
    // Button 9: Run transport out
    buttonPanel
        .button(9)
        .whileTrue(
            intakeSliderCommands.runIntakeForward(intakeSlider, Volts.of(intakeSliderTestVoltage)));
    // Button 10: Stop transport
    buttonPanel.button(10).onTrue(intakeSliderCommands.stopIntakeSlider(intakeSlider));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
