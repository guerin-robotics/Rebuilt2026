package frc.robot.subsystems.hood;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class HoodConstants {

  public static class Mechanical {
    public static double hoodMaxPos = 1.0;
    public static double hoodMinPos = 0.0;

    public static double leftServoOffset = -0.02;
  }

  public class HoodMap {
    public static final InterpolatingDoubleTreeMap ANGLE_MAP = new InterpolatingDoubleTreeMap();

    // Key is distance from center of hub (converted to meters)
    // Value is hood position (0.0-1.0)
    static {
      ANGLE_MAP.put(inchesToMeters(70), 0.45);
      ANGLE_MAP.put(inchesToMeters(83), 0.475);
      ANGLE_MAP.put(inchesToMeters(90), 0.5);
      ANGLE_MAP.put(inchesToMeters(100), 0.5);
      ANGLE_MAP.put(inchesToMeters(120), 0.55);
      ANGLE_MAP.put(inchesToMeters(130), 0.5);
      ANGLE_MAP.put(inchesToMeters(150), 0.65);
      ANGLE_MAP.put(inchesToMeters(205), 0.65);
    }
  }
}
