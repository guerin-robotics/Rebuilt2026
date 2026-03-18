package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.hood.io.HoodIO;
import frc.robot.subsystems.hood.io.HoodIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs;

  public Hood(HoodIO io) {
    this.io = io;
    this.inputs = new HoodIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  public void setHoodPos(double position) {
    final boolean hoodCheck = isHoodSafeVelo();
    Logger.recordOutput("Hood/isHoodSafeVelo", hoodCheck);
    if (isHoodSafeVelo()) {
      io.setHoodPos(position);
    }
  }

  public void incrementHoodPos() {
    final boolean hoodCheck = isHoodSafeVelo();
    Logger.recordOutput("Hood/isHoodSafeVelo", hoodCheck);
    double position = inputs.servoPos;
    if (isHoodSafeVelo()) {
      io.setHoodPos(position + 0.05);
    }
  }

  public void setHoodPosForHub() {
    final boolean hoodCheck = isHoodSafeVelo();
    Logger.recordOutput("Hood/isHoodSafeVelo", hoodCheck);
    double position = HoodPosCalculator.getInstance().getHoodPosForHub();
    if (isHoodSafeVelo()) {
      io.setHoodPos(position);
    }
  }

  // Returns false if robot is in or near a trench zone
  public boolean isHoodSafePos() {
    return !((RobotState.getInstance().getRobotZone()
            == HardwareConstants.Zones.Zone.ALLIANCE_TRENCH)
        || (RobotState.getInstance().getRobotZone() == HardwareConstants.Zones.Zone.OPPOSING_TRENCH)
        || (RobotState.getInstance().getRobotZone()
            == HardwareConstants.Zones.Zone.NEAR_ALLIANCE_TRENCH)
        || (RobotState.getInstance().getRobotZone()
            == HardwareConstants.Zones.Zone.NEAR_OPPOSING_TRENCH));
  }

  // Returns false if robot is in a trench zone, or if it is near a trench zone and moving towards
  // the trench
  public boolean isHoodSafeVelo() {
    return !((RobotState.getInstance().getRobotZone()
            == HardwareConstants.Zones.Zone.ALLIANCE_TRENCH)
        || (RobotState.getInstance().getRobotZone() == HardwareConstants.Zones.Zone.OPPOSING_TRENCH)
        || (RobotState.getInstance().getRobotZone()
                == HardwareConstants.Zones.Zone.NEAR_ALLIANCE_TRENCH
            && RobotState.getInstance().getFieldRelativeVelocity().vxMetersPerSecond > 0)
        || (RobotState.getInstance().getRobotZone()
                == HardwareConstants.Zones.Zone.NEAR_OPPOSING_TRENCH
            && RobotState.getInstance().getFieldRelativeVelocity().vxMetersPerSecond < 0));
  }

  public void stopHood() {
    io.stopHood();
  }
}
