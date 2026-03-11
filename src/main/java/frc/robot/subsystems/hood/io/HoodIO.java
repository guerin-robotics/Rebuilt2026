package frc.robot.subsystems.hood.io;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  @AutoLog
  public static class HoodIOInputs {
    public double servoSpeed;
    public double servoPos;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  /**
   * Sets the hood position using a normalized value.
   *
   * @param position 0.0 (min) to 1.0 (max)
   */
  public default void setHoodPos(double position) {}

  public default void stopHood() {}
}
