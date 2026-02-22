package frc.robot.subsystems.intakeRoller;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;

public class intakeRollerConstants {
  /** Current limits for the prestage motors. */
  public static class CurrentLimits {
    public static final int INTAKE_ROLLER_MAIN_SUPPLY_AMP = 40;
    public static final int INTAKE_ROLLER_MAIN_SUPPLY_TRIGGER_AMP = 35;
    public static final Time INTAKE_ROLLER_MAIN_SUPPLY_TRIGGER_TIME_SEC = Seconds.of(1);
    public static final int INTAKE_ROLLER_MAIN_STATOR_AMP = 60;
  }

  public static class SoftwareConstants {
    public static final boolean INVERTED = false;
  }

  public static class rollerMagicConstants {
    public static final double rollerAccel = 20.0;
  }

  // NOT Tuned yet - placeholder values
  public static class PID {
    public static final double KS = 35.0;
    public static final double KV = 0.12;
    public static final double KP = 0.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
  }
}
