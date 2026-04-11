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

  /**
   * Persistent offset (in degrees) added to every hood position command. In tuning mode the
   * operator can increment this with a button so each shot lands a little higher/lower without
   * editing constants. The offset survives across commands because it lives on the subsystem.
   */
  private Angle hoodOffset = Degrees.of(0);

  public Hood(HoodIO io) {
    this.io = io;
    this.inputs = new HoodIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    // Log the tuning offset so we can see it in AdvantageScope
    Logger.recordOutput("Hood/TuningOffset", hoodOffset.in(Degrees));

    // Report hood current usage to the battery logger
    Robot.batteryLogger.reportCurrentUsage(
        "Hood",
        false,
        inputs.hoodSupplyCurrent != null ? inputs.hoodSupplyCurrent.in(Units.Amps) : 0.0);
  }

  /**
   * Sets the hood to the given position <b>plus</b> the current tuning offset. Every command that
   * targets a hood angle goes through here, so the offset is always applied.
   */
  public void setHoodPos(Angle position) {
    io.setHoodPos(position.plus(hoodOffset));
  }

  /**
   * Increments the persistent tuning offset by 5°. This is intended for use in tuning mode so the
   * operator can nudge the hood angle between shots without changing code constants.
   */
  public void incrementHoodOffset() {
    hoodOffset = hoodOffset.plus(Degrees.of(5));
  }

  /** Resets the tuning offset back to 0°. */
  public void resetHoodOffset() {
    hoodOffset = Degrees.of(0);
  }

  /** Returns the current tuning offset. */
  public Angle getHoodOffset() {
    return hoodOffset;
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
}
