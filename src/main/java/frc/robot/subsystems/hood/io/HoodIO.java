package frc.robot.subsystems.hood.io;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  @AutoLog
  public static class HoodIOInputs {
    public double servoSpeed;
    public double servoPos;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setHoodPos() {}

  public default void stopHood() {}
}
