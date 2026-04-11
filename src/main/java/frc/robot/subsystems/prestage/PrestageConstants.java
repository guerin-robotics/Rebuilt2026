package frc.robot.subsystems.prestage;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants;

public class PrestageConstants {
  /** Current limits for the prestage motors. */
  public static class CurrentLimits {
    public static final int PRESTAGE_MAIN_SUPPLY_AMP = 40;
    public static final int PRESTAGE_MAIN_SUPPLY_TRIGGER_AMP = 35;
    public static final Time PRESTAGE_MAIN_SUPPLY_TRIGGER_TIME_SEC = Seconds.of(1);
    public static final int PRESTAGE_MAIN_STATOR_AMP = 60;
  }

  public static class SoftwareConstants {
    public static final boolean INVERTED = true;
  }

  public static class Mechanical {
    public static final double prestageRatio = 1;
  }

  // Real robot PID gains for torque-current velocity control
  public static class PID {
    public static final double KS = 9.0; // 8.0
    public static final double KV = 0.0; // 0.0
    public static final double KP = 10.0; // 15.0

    public static final double KI = 0.0; // 0.0

    public static final double KD = 0.0; // 0.0
  }

  public static class prestageMagicConstants {
    public static final double prestageAccel = 100.0;
  }

  /** Simulation constants for the prestage motors. */
  public static class Sim {
    /** One Kraken X60 FOC motor per prestage side. */
    public static final DCMotor PRESTAGE_MOTOR = DCMotor.getKrakenX60Foc(1);

    /** Number of motors per prestage side (used for DCMotorSim). */
    public static final int NUM_MOTORS_PER_SIDE = 1;

    /** Moment of inertia of each prestage roller (kg·m²). Approximate value. */
    public static final double PRESTAGE_MOI = 0.001;

    // Sim-specific torque-current PID gains for the left motor
    public static final double LEFT_KS = 2.0;
    public static final double LEFT_KV = 0.0;
    public static final double LEFT_KP = 3.0;
    public static final double LEFT_KI = 0.0;
    public static final double LEFT_KD = 0.0;

    // Sim-specific torque-current PID gains for the right motor
    public static final double RIGHT_KS = 2.0;
    public static final double RIGHT_KV = 0.0;
    public static final double RIGHT_KP = 3.0;
    public static final double RIGHT_KI = 0.0;
    public static final double RIGHT_KD = 0.0;
  }

  /** Returns the appropriate left KS for the current mode (real vs sim). */
  public static double getLeftKS() {
    return Constants.currentMode == Constants.Mode.SIM ? Sim.LEFT_KS : PID.KS;
  }

  /** Returns the appropriate left KP for the current mode (real vs sim). */
  public static double getLeftKP() {
    return Constants.currentMode == Constants.Mode.SIM ? Sim.LEFT_KP : PID.KP;
  }
}
