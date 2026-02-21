package frc.robot.subsystems.transport;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;

public class TransportConstants {

  /** Current limits for the TRANSPORT motors. */
  public static class CurrentLimits {
    public static final int TRANSPORT_MAIN_SUPPLY_AMP = 40;
    public static final int TRANSPORT_MAIN_SUPPLY_TRIGGER_AMP = 35;
    public static final Time TRANSPORT_MAIN_SUPPLY_TRIGGER_TIME_SEC = Seconds.of(1);
    public static final int TRANSPORT_MAIN_STATOR_AMP = 60;
  }

  public static class SoftwareConstants {
    public static final boolean INVERTED = false;
  }

  // NOT Tuned yet - placeholder values
  public static class PID {
    public static final double KS = 35;
    public static final double KV = 0.12;
    public static final double KP = 0.15;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
  }

  public static class transportMagicConstants {
    public static final double transportAccel = 2.0;
  }
}
