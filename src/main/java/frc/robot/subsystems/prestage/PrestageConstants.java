package frc.robot.subsystems.prestage;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;

public class PrestageConstants {
  /** Current limits for the prestage motors. */
  public static class CurrentLimits {
    public static final int PRESTAGE_MAIN_SUPPLY_AMP = 40;
    public static final int PRESTAGE_MAIN_SUPPLY_TRIGGER_AMP = 35;
    public static final Time PRESTAGE_MAIN_SUPPLY_TRIGGER_TIME_SEC = Seconds.of(1);
    public static final int PRESTAGE_MAIN_STATOR_AMP = 60;
  }

  public static class SoftwareConstants {
    public static final boolean INVERTED = false;
  }

  public static class Mechanical {
    public static final double prestageRatio = 0.5;
  }

  // NOT Tuned yet - placeholder values
  public static class PID {
    public static final double KS = 15.0;
    public static final double KV = 0.0;
    public static final double KP = 5.0;

    public static final double followerKS = 5.0;
    public static final double followerKV = 0.0;
    public static final double followerKP = 3.0;

    public static final double KI = 0.0;

    public static final double KD = 0.0;
  }

  public static class prestageMagicConstants {
    public static final double prestageAccel = 60.0;
  }
}
