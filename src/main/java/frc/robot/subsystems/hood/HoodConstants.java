package frc.robot.subsystems.hood;

import static edu.wpi.first.math.util.Units.inchesToMeters;

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
      ANGLE_MAP.put(inchesToMeters(24), 0.25);
      ANGLE_MAP.put(inchesToMeters(82), 0.5);
    }
  }
}
