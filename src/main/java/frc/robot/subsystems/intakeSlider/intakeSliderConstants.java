package frc.robot.subsystems.intakeSlider;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.system.plant.DCMotor;
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
    public static final double sliderRatio = 0.417;
    public static final double sliderJostleCurrentLimit = 70;
  }

  public static class sliderMagicConstants {
    public static final double sliderAccel = 60.0;
  }

  /** Simulation constants for the intake slider motor. */
  public static class Sim {
    /** One Kraken X60 FOC motor drives the slider. */
    public static final DCMotor SLIDER_MOTOR = DCMotor.getKrakenX60Foc(1);

    /** Number of motors driving the slider (used for DCMotorSim). */
    public static final int NUM_MOTORS = 1;

    /** Moment of inertia of the slider mechanism (kg·m²). Approximate value. */
    public static final double SLIDER_MOI = 0.002;

    // Sim PID gains for TalonFX closed-loop in simulation
    public static final double KS = 0.0;
    public static final double KV = 0.12;
    public static final double KP = 1.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
  }
}
