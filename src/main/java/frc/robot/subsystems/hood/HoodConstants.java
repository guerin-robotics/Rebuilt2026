package frc.robot.subsystems.hood;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class HoodConstants {

  public static class Mechanical {
    public static double hoodMaxPos = 1.0;
    public static double hoodMinPos = 0.0;
  }

  public class HoodMap {
    public static final InterpolatingDoubleTreeMap ANGLE_MAP = new InterpolatingDoubleTreeMap();

    // Key is distance from center of hub (converted to meters)
    // Value is hood position (0.0-1.0)
    static {
      ANGLE_MAP.put(3.41, 0.55);
      ANGLE_MAP.put(3.80, 0.6);
      ANGLE_MAP.put(2.38, 0.45);
    }
  }
}
