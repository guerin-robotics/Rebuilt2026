package frc.robot.subsystems.hood;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Time;

public class HoodConstants {

  public static class Mechanical {
    public static final double magnetSensorDiscontinuityPoint = 1;
    public static final double magnetOffset = -0.16;
    public static double hoodMaxPos = 1.0;
    public static double hoodMinPos = 0.0;

    public static double leftServoOffset = -0.02;

    public static double hoodRatio = 122.0 / 12.0;
  }

  public static class SoftwareConstants {
    public static final double softwareLowerRotationLimit = 0;
    public static final SensorDirectionValue ENCODER_DIRECTION =
        SensorDirectionValue.Clockwise_Positive;
    public static boolean MOTOR_INVERTED = false;
    public static double softwareUpperRotationLimit = 2.0;
  }

  public static class CurrentLimits {
    public static final double HOOD__MAIN_SUPPLY_AMP = 40;
    public static final double HOOD_MAIN_SUPPLY_TRIGGER_AMP = 35;
    public static final Time HOOD_MAIN_SUPPLY_TRIGGER_TIME_SEC = Seconds.of(1);
    public static final double HOOD_MAIN_STATOR_AMP = 20;
  }

  public static class PID {
    public static final double KG = 10; // 0.8
    public static final double KP = 0; // 0
    public static final double KD = 0; // 0
  }

  public static class HoodMagicConstants {
    public static double hoodAccel = 10.0;
    public static double hoodVelo = 1;
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
