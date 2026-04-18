package frc.robot.subsystems.upperFeeder;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants;

public class UpperFeederConstants {

  /** Current limits for the feeder motors. */
  public static class CurrentLimits {
    public static final int UPPER_FEEDER_MAIN_SUPPLY_AMP = 40;
    public static final int UPPER_FEEDER_MAIN_SUPPLY_TRIGGER_AMP = 35;
    public static final Time UPPER_FEEDER_MAIN_SUPPLY_TRIGGER_TIME_SEC = Seconds.of(1);
    public static final int UPPER_FEEDER_MAIN_STATOR_AMP = 40;
  }

  public static class SoftwareConstants {
    public static final boolean INVERTED = false;
  }

  public static class Mechanical {
    public static final double upperFeederRatio = 24.0 / 11;
  }

  // Real robot PID gains for torque-current velocity control
  public static class PID {
    public static final double KS = 2.0; // 6.0
    public static final double KV = 0.0; // 0.0
    public static final double KP = 13.0; // 4.0
    public static final double KI = 0.0; // 0.0
    public static final double KD = 0.0; // 0.0
  }

  public static class feederMagicConstants {
    public static final double upperFeederAccel = 120.0;
  }

  /** Simulation constants for the feeder motor. */
  public static class Sim {
    /** One Kraken X60 FOC motor drives the feeder. */
    public static final DCMotor UPPER_FEEDER_MOTOR = DCMotor.getKrakenX60Foc(1);

    /** Number of motors driving the feeder (used for DCMotorSim). */
    public static final int NUM_MOTORS = 1;

    /** Moment of inertia of the feeder mechanism (kg·m²). Approximate value. */
    public static final double UPPER_FEEDER_MOI = 0.001;

    // Sim-specific torque-current PID gains.
    // The TalonFX firmware still runs TorqueCurrentFOC in sim and produces
    // a motor voltage output that we feed into DCMotorSim.
    public static final double KS = 3.0; // 3.0
    public static final double KV = 0.0;
    public static final double KP = 0.0; // 2.0
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
