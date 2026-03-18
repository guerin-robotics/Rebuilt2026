package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Time;

public class FeederConstants {

  /** Current limits for the feeder motors. */
  public static class CurrentLimits {
    public static final int FEEDER_MAIN_SUPPLY_AMP = 40;
    public static final int FEEDER_MAIN_SUPPLY_TRIGGER_AMP = 35;
    public static final Time FEEDER_MAIN_SUPPLY_TRIGGER_TIME_SEC = Seconds.of(1);
    public static final int FEEDER_MAIN_STATOR_AMP = 80;
  }

  public static class SoftwareConstants {
    public static final boolean INVERTED = false;
  }

  public static class Mechanical {
    public static final double feederRatio = 1 / 0.417;
  }

  // NOT Tuned yet - placeholder values
  public static class PID {
    public static final double KS = 5.0;
    public static final double KV = 0.0;
    public static final double KP = 3.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
  }

  public static class feederMagicConstants {
    public static final double feederAccel = 60.0;
  }

  /** Simulation constants for the feeder motor. */
  public static class Sim {
    /** One Kraken X60 FOC motor drives the feeder. */
    public static final DCMotor FEEDER_MOTOR = DCMotor.getKrakenX60Foc(1);

    /** Number of motors driving the feeder (used for DCMotorSim). */
    public static final int NUM_MOTORS = 1;

    /** Moment of inertia of the feeder mechanism (kg·m²). Approximate value. */
    public static final double FEEDER_MOI = 0.001;

    // Sim PID gains for TalonFX closed-loop in simulation
    public static final double KS = 0.0;
    public static final double KV = 0.12;
    public static final double KP = 1.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
  }
}
