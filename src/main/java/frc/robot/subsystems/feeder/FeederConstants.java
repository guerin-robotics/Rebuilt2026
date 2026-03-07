package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;

public class FeederConstants {

  /** Current limits for the feeder motors. */
  public static class CurrentLimits {
    public static final int FEEDER_MAIN_SUPPLY_AMP = 40;
    public static final int FEEDER_MAIN_SUPPLY_TRIGGER_AMP = 35;
    public static final Time FEEDER_MAIN_SUPPLY_TRIGGER_TIME_SEC = Seconds.of(1);
    public static final int FEEDER_MAIN_STATOR_AMP = 60;
  }

  public static class SoftwareConstants {
    public static final boolean INVERTED = false;
  }

  public static class Mechanical {
    public static final double feederRatio = 0.417;
  }

  // NOT Tuned yet - placeholder values
  public static class PID {
    public static final double KS = 2.0;
    public static final double KV = 0.0;
    public static final double KP = 1.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
  }

  public static class feederMagicConstants {
    public static final double feederAccel = 60.0;
  }
}
