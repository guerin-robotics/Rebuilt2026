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
    if (RobotState.getInstance().isHoodSafeVelo()) {
      io.setHoodPos(position);
    }
  }

  public void incrementHoodPos() {
    double position = inputs.servoPos;
    if (RobotState.getInstance().isHoodSafeVelo()) {
      io.setHoodPos(position + 0.05);
    }
  }

  public void setHoodPosForHub() {
    double position = HoodPosCalculator.getInstance().getHoodPosForHub();
    if (RobotState.getInstance().isHoodSafeVelo()) {
      io.setHoodPos(position);
    } else {
      io.setHoodPos(HardwareConstants.TestPositions.hoodPos1Test);
    }
  }

  public void stopHood() {
    io.stopHood();
  }
}
