package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareConstants;
import frc.robot.RobotState;
import frc.robot.Triggers;
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
    // if (Triggers.getInstance().isHoodSafe(RobotState.getInstance().getEstimatedPose())) {
    io.setHoodPos(position);
    // }
  }

  public void incrementHoodPos() {
    double position = inputs.servoPos;
    // if (RobotState.getInstance().isHoodSafe(RobotState.getInstance().getEstimatedPose())) {
    io.setHoodPos(position + 0.05);
    // }
  }

  /**
   * Sets the hood to the calculated position for the alliance hub.
   *
   * <p><b>Performance note:</b> This runs as the default command (every 20 ms). The call chain is:
   *
   * <ol>
   *   <li>{@code HoodPosCalculator.getHoodPosForHub()} → {@code getAllianceHubTarget()} → {@code
   *       AllianceFlipUtil} (cached) → {@code getDistanceToPoint()} → {@code Pose2d.getTranslation
   *       .getDistance()} → interpolation table lookup
   *   <li>{@code RobotState.isHoodSafe()} → {@code getRobotZone()} + velocity check (now caches
   *       zone result internally)
   * </ol>
   *
   * Both paths are now optimised (AllianceFlipUtil caches alliance per-loop, RobotState caches zone
   * per call). Further reduction would require caching the hood position across loops, but the
   * distance changes every loop so the lookup is necessary.
   */
  public void setHoodPosForHub() {
    double position = HoodPosCalculator.getInstance().getHoodPosForHub();
    if (Triggers.getInstance().isHoodSafe(RobotState.getInstance().getEstimatedPose())) {
      io.setHoodPos(position);
    } else {
      io.setHoodPos(HardwareConstants.CompConstants.Positions.hoodDownPos);
    }
  }

  public void setHoodPosForShoot() {
    double position = HoodPosCalculator.getInstance().getHoodPosForHub();
    io.setHoodPos(position);
  }

  public void stopHood() {
    io.stopHood();
  }
}
