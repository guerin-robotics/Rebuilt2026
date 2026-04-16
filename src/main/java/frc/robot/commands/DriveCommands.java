// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.AllianceFlipUtil;
import frc.lib.FieldConstants;
import frc.robot.HardwareConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 7.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Apply rotation deadband
              double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

              // Square rotation value for more precise control
              omega = Math.copySign(omega * omega, omega);

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega * drive.getMaxAngularSpeedRadPerSec());
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)
        .withName("JoystickDrive");
  }

  public static Command joystickDriveLimited(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Apply rotation deadband
              double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

              // Square rotation value for more precise control
              omega = Math.copySign(omega * omega, omega);

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * DriveConstants.limitedVelo,
                      linearVelocity.getY() * DriveConstants.limitedVelo,
                      omega * drive.getMaxAngularSpeedRadPerSec());
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)
        .withName("JoystickDriveLimited");
  }

  // public static Command driveLucasProof(
  //     Drive drive,
  //     DoubleSupplier xSupplier,
  //     DoubleSupplier ySupplier,
  //     DoubleSupplier omegaSupplier,
  //     Trigger override) {
  //   return new ContinuousConditionalCommand(
  //           joystickDrive(drive, xSupplier, ySupplier, omegaSupplier),
  //           joystickDriveLimited(drive, xSupplier, ySupplier, omegaSupplier),
  //           () -> {
  //             boolean driveNormal =
  //                 (Triggers.getInstance().isIntakeSafe.getAsBoolean() ||
  // override.getAsBoolean());
  //             Logger.recordOutput(
  //                 "RobotState/isIntakeSafe",
  //                 Triggers.getInstance().isIntakeSafe.getAsBoolean()
  //                     ? "intakeSafe"
  //                     : "intakeUnsafe");
  //             Logger.recordOutput(
  //                 "RobotState/isOverrideActive",
  //                 override.getAsBoolean() ? "override" : "noOverride");
  //             return driveNormal;
  //           })
  //       .withName("DriveLucasProof");
  // }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Log target and current angles every loop for debugging
              Logger.recordOutput("AutoAim/TargetAngle", rotationSupplier.get());
              Logger.recordOutput("AutoAim/CurrentAngle", drive.getRotation());
              Logger.recordOutput("AutoAim/AngleErrorRad", angleController.getPositionError());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()))
        .withName("JoystickDriveAtAngle");
  }

  /**
   * Field relative drive command that snaps the robot to the nearest "straight" X-axis heading.
   *
   * <p>The two X-axis headings are 0° (facing the red alliance wall) and 180° (facing the blue
   * alliance wall). The command picks whichever heading is closest to the robot's current heading
   * so the robot doesn't spin around. For example:
   *
   * <ul>
   *   <li>If the robot is roughly facing the red wall (heading between -90° and 90°), it snaps to
   *       0°.
   *   <li>If the robot is roughly facing the blue wall (heading outside that range), it snaps to
   *       180°.
   * </ul>
   *
   * <p>This is useful for quickly straightening out to drive through the trench.
   *
   * @param drive The drive subsystem
   * @param xSupplier Joystick X axis (left/right translation)
   * @param ySupplier Joystick Y axis (forward/back translation)
   */
  public static Command joystickDriveAlignForTrench(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

    return joystickDriveAtAngle(
        drive,
        xSupplier,
        ySupplier,
        () -> {
          // // Get the robot's current heading in radians (-PI to PI)
          // double currentRadians = drive.getRotation().getRadians();
          // // If the absolute heading is <= 90° (PI/2), the robot is closer to 0° (facing red
          // wall)
          // // Otherwise, it's closer to 180° (facing blue wall)
          // if (Math.abs(currentRadians) <= Math.PI / 2.0) {
          //   return Rotation2d.kZero; // Snap to 0°
          // } else {
          //   return Rotation2d.kPi; // Snap to 180°
          // }

          double currentY = RobotState.getInstance().getEstimatedPose().getY();

          // If the robot's y-position is less than the center line, it should snap to -90;
          // otherwise, to 90
          if (currentY < FieldConstants.LinesHorizontal.center) {
            return Rotation2d.kCCW_90deg; // Snap to -90°
          } else {
            return Rotation2d.kCW_90deg; // Snap to 90°
          }
        });
  }

  public static Command joystickDriveAlignForBump(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return joystickDriveAtAngle(
        drive,
        xSupplier,
        ySupplier,
        () -> {
          // Get the robot's current heading in radians (-PI to PI)
          double currentRadians = drive.getRotation().getRadians();

          // If the absolute heading is <= 90° (PI/2), the robot is closer to 45° (facing red wall)
          // Otherwise, it's closer to 135° (facing blue wall)
          if (Math.abs(currentRadians) <= Math.PI / 2.0) {
            return Rotation2d.fromRadians(Math.PI / 4.0); // Snap to 45°
          } else if (Math.abs(currentRadians) <= Math.PI) {
            return Rotation2d.fromRadians((3 * Math.PI) / 4.0); // Snap to 135°
          } else if (Math.abs(currentRadians) <= ((3 * Math.PI) / 2.0)) {
            return Rotation2d.fromRadians((5 * Math.PI) / 4.0); // Snap to 225°
          } else {
            return Rotation2d.fromRadians((7 * Math.PI) / 4.0); // Snap to 315°
          }
        });
  }

  public static Command joystickDriveAlignForTower(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return joystickDriveAtAngle(
        drive,
        xSupplier,
        ySupplier,
        () -> {
          double currentY =
              AllianceFlipUtil.applyY(RobotState.getInstance().getEstimatedPose().getY());

          if (currentY > FieldConstants.LinesHorizontal.center) {
            Logger.recordOutput("RobotState/towerAlign", "farSide");
            return AllianceFlipUtil.apply(Rotation2d.kCW_90deg);
          } else {
            Logger.recordOutput("RobotState/towerAlign", "nearSide");
            return AllianceFlipUtil.apply(Rotation2d.kCCW_90deg);
          }
        });
  }

  public static Command joystickDriveAlignForWall(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return joystickDriveAtAngle(
        drive,
        xSupplier,
        ySupplier,
        () -> {
          double currentY = RobotState.getInstance().getEstimatedPose().getY();
          if (currentY > FieldConstants.fieldWidth + HardwareConstants.Zones.zoneOffset) {
            return Rotation2d.kCCW_Pi_2;
          } else if (currentY < HardwareConstants.Zones.zoneOffset) {
            return Rotation2d.kCW_Pi_2;
          } else {
            return Rotation2d.kPi;
          }
        });
  }

  public static Command joystickDriveAlignForSweepToAllianceZone(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return joystickDriveAtAngle(
        drive,
        xSupplier,
        ySupplier,
        () -> {
          // Get the robot's y-coordinate (which side it's on)
          double fieldSide =
              AllianceFlipUtil.applyY(frc.robot.RobotState.getInstance().getEstimatedPose().getY());
          Rotation2d targetRotation;

          // If on near side (low y), set rotation to -45
          if (fieldSide < (FieldConstants.fieldWidth / 2)) {
            targetRotation = Rotation2d.fromRadians((5 * Math.PI) / 4);
          }
          // If on far side (high y), set rotation to 45
          else {
            targetRotation = Rotation2d.fromRadians((3 * Math.PI) / 4);
          }
          return AllianceFlipUtil.apply(targetRotation);
        });
  }

  public static Command joystickDriveAlignForSweep(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return joystickDriveAtAngle(
        drive,
        xSupplier,
        ySupplier,
        () -> {
          // Get the robot's current heading in radians (-PI to PI)
          Rotation2d currentRotation = AllianceFlipUtil.apply(drive.getRotation());
          double currentRadians = currentRotation.getRadians();
          // Get the robot's y-coordinate (which side it's on)
          double fieldSide =
              AllianceFlipUtil.applyY(frc.robot.RobotState.getInstance().getEstimatedPose().getY());
          Rotation2d targetRotation;

          // If on near side (low y), set rotation to -45
          if (fieldSide < (FieldConstants.fieldWidth / 2)) {
            if (currentRadians > (Math.PI / 2) && currentRadians < ((3 * Math.PI) / 2)) {
              targetRotation = Rotation2d.fromRadians((5 * Math.PI) / 4);
            } else {
              targetRotation = Rotation2d.fromRadians((7 * Math.PI) / 4);
            }
          }
          // If on far side (high y), set rotation to 45
          else {
            if (currentRadians > (Math.PI / 2) && currentRadians < ((3 * Math.PI) / 2)) {
              targetRotation = Rotation2d.fromRadians((3 * Math.PI) / 4);
            } else {
              targetRotation = Rotation2d.fromRadians(Math.PI / 4);
            }
          }
          return AllianceFlipUtil.apply(targetRotation);
        });
  }

  public static Command stopWithX(Drive drive) {
    return Commands.run(
            () -> {
              Logger.recordOutput("RobotState/Drive", "Stopping with X");
              drive.stopWithX();
            },
            drive)
        .withName("StopWithX");
  }

  public static Command stopWithXAfterWait(Drive drive) {
    return Commands.sequence(new WaitCommand(2), Commands.runOnce(() -> drive.stopWithX(), drive))
        .withName("StopWithXAfterWait");
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
