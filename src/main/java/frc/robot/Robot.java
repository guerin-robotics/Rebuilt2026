// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.AllianceFlipUtil;
import frc.robot.util.BatteryLogger;
import frc.robot.util.Elastic;
import frc.robot.util.HubShiftUtil;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  /** Shared battery/energy logger — subsystems call {@code reportCurrentUsage()} each loop. */
  public static final BatteryLogger batteryLogger = new BatteryLogger();

  // Field2d widget to display the robot's current pose on the dashboard.
  // This is updated every loop so the drive team can always see where the robot thinks it is.
  private final Field2d fieldMap = new Field2d();

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // RobotState is a singleton accessed via static getInstance() calls, so AdvantageKit's
    // automatic field scanner cannot find it through the Robot → RobotContainer object graph.
    // We must manually register it here so that @AutoLogOutput annotations on its methods
    // (e.g. getEstimatedPose, getAngleToAllianceHub) are picked up and logged every loop cycle.
    AutoLogOutputManager.addObject(RobotState.getInstance());

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    // Publish the robot pose Field2d to the dashboard so we can see where the robot is
    SmartDashboard.putData("Robot Pose Field Map", fieldMap);
    SmartDashboard.putString("Battery Changed", "Battery Changed");
    SmartDashboard.putString("Battery Secured", "Battery Secured");
    SmartDashboard.putString("Auto Path Changed", "Auto Path Changed");
    SmartDashboard.putString("Bumpers On", "Bumpers On");
    SmartDashboard.putString("Code Deployed", "Code Deployed");

    SmartDashboard.putString("Intake Pivot", "Intake Pivot");
    SmartDashboard.putString("Intake Roller", "Intake Roller");
    SmartDashboard.putString("Shoot Sequence", "Shoot Sequence");
    SmartDashboard.putString("Movement", "Movement");
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Update battery logger with voltage and RIO current, then log after scheduler
    batteryLogger.setBatteryVoltage(RobotController.getBatteryVoltage());
    batteryLogger.setRioCurrent(RobotController.getInputCurrent());
    batteryLogger.periodicAfterScheduler();

    // Refresh the cached alliance color once per loop so that AllianceFlipUtil.shouldFlip()
    // doesn't call DriverStation.getAlliance() (which creates an Optional) 20-30+ times per cycle.
    AllianceFlipUtil.refresh();

    // CPU FIX: cache pose once — was calling getEstimatedPose() 4 separate times here,
    // plus getBroadZone(pose) was called again inside getSpecificZone and getApproachingZone.
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();

    // Update the robot's pose on the main field map dashboard widget every loop.
    // This must be in robotPeriodic() so it runs in ALL modes (disabled, teleop, auto, test).
    fieldMap.setRobotPose(currentPose);

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);

    // Set tuning mode to false if connected to FMS
    if (HardwareConstants.TuningConstants.atComp) {
      if (DriverStation.isFMSAttached()) {
        HardwareConstants.TuningConstants.TUNING_MODE = false;
      } else {
        HardwareConstants.TuningConstants.TUNING_MODE = HardwareConstants.TuningConstants.isTuning;
      }
    } else {
      HardwareConstants.TuningConstants.TUNING_MODE = HardwareConstants.TuningConstants.isTuning;
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    // Update auto path preview and starting pose check.
    // This lets the drive team verify the selected auto path and robot placement.
    robotContainer.updateAutoPreview();
    robotContainer.checkStartPose();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Update the auto preview field with the robot's current pose during auto
    // so we can see the robot following the path in real time
    robotContainer.autoPreviewField.setRobotPose(RobotState.getInstance().getEstimatedPose());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    CommandScheduler.getInstance().schedule(robotContainer.getAutoStopCommand());
    HubShiftUtil.initialize();

    CommandScheduler.getInstance().schedule(robotContainer.getIntakeRollerCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getIntakePivotCommand());

    // Automated tab switching
    Elastic.selectTab("Teleoperated");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Publish time left in shift to the dashboard
    SmartDashboard.putNumber(
        "Time Left in Shift",
        Math.round(HubShiftUtil.getShiftedShiftInfo().remainingTime() * 10.0) / 10.0);
    // Displays whether the alliance was won by our team or opposing team
    SmartDashboard.putBoolean("Win Auto?", !HubShiftUtil.isActiveFirst());
    // Displays whether our alliance's hub is active or not
    SmartDashboard.putBoolean("Is Hub Active", HubShiftUtil.getShiftedShiftInfo().active());
    // Displays the match time
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    Logger.recordOutput("RobotState/HubShift", HubShiftUtil.getShiftedShiftInfo().active());
    Logger.recordOutput("RobotState/firstActiveAlliancer", HubShiftUtil.getFirstActiveAlliance());
    Logger.recordOutput(
        "RobotState/timeRemainingInShift", HubShiftUtil.getShiftedShiftInfo().remainingTime());
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
