package frc.robot.subsystems.hood;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;

public class HoodConstants {

  public static class Mechanical {
    public static final double magnetSensorDiscontinuityPoint = 0.75 / 2 + 0.25; // 1;
    public static final double magnetOffset = -0.16;
    // Hood position limits in degrees (mechanism output)
    public static final Angle hoodMaxPos = Degrees.of(234); // was 0.65 rotations
    public static final Angle hoodMinPos = Degrees.of(0);

    /**
     * Gear ratio from the output shaft (where the CANcoder is) to the hood mechanism.
     *
     * <p>The 12T lantern gear on the shaft meshes with the 122T hood gear, so one shaft rotation =
     * 12/122 hood rotations → ratio = 122/12.
     */
    public static double shaftToHoodRatio = 122.0 / 12.0;

    /**
     * Belt ratio from the motor rotor to the output shaft.
     *
     * <p>This needs updated to the real values: GH 4/11
     *
     * <p>30T pulley on motor → belt → 20T pulley on output shaft. One motor rotation = 20/30 shaft
     * rotations → ratio = 30/20.
     */
    public static double motorToShaftRatio = 5.33;
  }

  public static class SoftwareConstants {
    // Software limits in degrees — converted to mechanism rotations when applied to the motor
    public static final Angle softwareLowerLimit = Degrees.of(0);
    public static final Angle softwareUpperLimit = Degrees.of(62); // matches hoodMaxPos
    public static final SensorDirectionValue ENCODER_DIRECTION =
        SensorDirectionValue.Clockwise_Positive;
    public static boolean MOTOR_INVERTED = false;
  }

  public static class CurrentLimits {
    public static final double HOOD__MAIN_SUPPLY_AMP = 40;
    public static final double HOOD_MAIN_SUPPLY_TRIGGER_AMP = 35;
    public static final Time HOOD_MAIN_SUPPLY_TRIGGER_TIME_SEC = Seconds.of(1);
    public static final double HOOD_MAIN_STATOR_AMP = 20;
  }

  public static class PID {
    public static final double KS = 9;
    public static final double KP = 4000; // 0
    public static final double KD = 0; // 0
  }

  public static class HoodMagicConstants {
    public static double hoodAccel = 10.0;
    public static double hoodVelo = 1;
  }

  public class HoodMap {
    public static final InterpolatingDoubleTreeMap ANGLE_MAP = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap PASSING_ANGLE_MAP = new InterpolatingDoubleTreeMap();

    // Key is distance from center of hub (converted to meters)
    // Value is hood position in degrees (scaled to 5°..55°)
    static {
      ANGLE_MAP.put(inchesToMeters(75.0), 1.0); // Auto shot 1
      ANGLE_MAP.put(inchesToMeters(85.0), 1.5); // Auto shot 2
      ANGLE_MAP.put(inchesToMeters(110.0), 2.5); // Tower shot
      ANGLE_MAP.put(inchesToMeters(130.0), 3.5);
      ANGLE_MAP.put(inchesToMeters(145.0), 5.25);
      ANGLE_MAP.put(inchesToMeters(160.0), 11.0);
    }

    static {
      // Passing numbers are guesses
      PASSING_ANGLE_MAP.put(inchesToMeters(120.0),40.0);
      PASSING_ANGLE_MAP.put(inchesToMeters(320.0), 50.0);
      PASSING_ANGLE_MAP.put(inchesToMeters(450.0), 60.0);
    }
  }
}
