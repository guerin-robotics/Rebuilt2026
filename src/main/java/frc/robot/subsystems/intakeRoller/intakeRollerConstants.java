package frc.robot.subsystems.intakeRoller;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants;

public class intakeRollerConstants {
  /** Current limits for the prestage motors. */
  public static class CurrentLimits {
    public static final int INTAKE_ROLLER_MAIN_SUPPLY_AMP = 40;
    public static final int INTAKE_ROLLER_MAIN_SUPPLY_TRIGGER_AMP = 35;
    public static final Time INTAKE_ROLLER_MAIN_SUPPLY_TRIGGER_TIME_SEC = Seconds.of(1);
    public static final int INTAKE_ROLLER_MAIN_STATOR_AMP = 150;
  }

  public static class SoftwareConstants {
    public static final boolean INVERTED = false;
  }

  public static class Mechanical {
    public static final double rollerRatio = 1 / 0.625;
  }

  public static class rollerMagicConstants {
    public static final double rollerAccel = 60.0;
  }

  // Real robot PID gains for torque-current velocity control
  public static class PID {
    public static final double KS = 3.5;
    public static final double KV = 0.0;
    public static final double KP = 1.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
  }

  /** Simulation constants for the intake roller. */
  public static class Sim {
    /** Two Kraken X60 FOC motors (leader + follower) drive the intake roller. */
    public static final DCMotor ROLLER_MOTOR = DCMotor.getKrakenX60Foc(2);

    /** Number of motors driving the roller (used for DCMotorSim). */
    public static final int NUM_MOTORS = 2;

    /** Moment of inertia of the intake roller (kg·m²). Approximate value. */
    public static final double ROLLER_MOI = 0.001;

    // Sim-specific torque-current PID gains
    public static final double KS = 2.0;
    public static final double KV = 0.0;
    public static final double KP = 1.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
  }

  /** Returns the appropriate KS for the current mode (real vs sim). */
  public static double getKS() {
    return Constants.currentMode == Constants.Mode.SIM ? Sim.KS : PID.KS;
  }

  /** Returns the appropriate KV for the current mode (real vs sim). */
  public static double getKV() {
    return Constants.currentMode == Constants.Mode.SIM ? Sim.KV : PID.KV;
  }

  /** Returns the appropriate KP for the current mode (real vs sim). */
  public static double getKP() {
    return Constants.currentMode == Constants.Mode.SIM ? Sim.KP : PID.KP;
  }

  /** Returns the appropriate KD for the current mode (real vs sim). */
  public static double getKD() {
    return Constants.currentMode == Constants.Mode.SIM ? Sim.KD : PID.KD;
  }
}
