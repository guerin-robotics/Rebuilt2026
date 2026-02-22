package frc.robot.subsystems.intakeSlider;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;

public class intakeSliderConstants {
  /** Current limits. */
  public static class CurrentLimits {
    public static final int INTAKE_SLIDER_MAIN_SUPPLY_AMP = 40;
    public static final int INTAKE_SLIDER_MAIN_SUPPLY_TRIGGER_AMP = 35;
    public static final Time INTAKE_SLIDER_MAIN_SUPPLY_TRIGGER_TIME_SEC = Seconds.of(1);
    public static final int INTAKE_SLIDER_MAIN_STATOR_AMP = 70;
  }

  public static class SoftwareConstants {
    public static final boolean INVERTED = false;
  }

  // NOT Tuned yet - placeholder values
  public static class PID {
    public static final double KS = 15.0;
    public static final double KV = 0.0;
    public static final double KP = 7.5;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
  }

  public static class Mechanical {
    public static final double rotationsPerInch = 1.0;
    public static final double rotationsWhenOut = 21;
  }

  public static class sliderMagicConstants {
    public static final double sliderAccel = 60.0;
  }
}
