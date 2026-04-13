package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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

    // Report hood current usage to the battery logger
    Robot.batteryLogger.reportCurrentUsage(
        "Hood",
        false,
        inputs.hoodSupplyCurrent != null ? inputs.hoodSupplyCurrent.in(Units.Amps) : 0.0);
  }

  public void setHoodPos(Angle position) {
    // if (Triggers.getInstance().isHoodSafe(RobotState.getInstance().getEstimatedPose())) {
    io.setHoodPos(position);
    // }
  }

  public void incrementHoodPos() {
    Angle position = inputs.hoodPosition;
    io.setHoodPos(position.plus(Degrees.of(5)));
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
    Angle position = HoodPosCalculator.getInstance().getHoodPosForHub();
    io.setHoodPos(position);
  }

  public void stowHood() {
    io.setHoodPos(Degrees.of(0));
  }

  /**
   * Returns the current hood position (mechanism angle).
   *
   * @return The hood angle as reported by the CANcoder
   */
  public Angle getPosition() {
    return inputs.hoodPosition;
  }
}
