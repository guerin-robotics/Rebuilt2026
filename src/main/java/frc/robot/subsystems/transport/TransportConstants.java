package frc.robot.subsystems.transport;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants;

public class TransportConstants {

  /** Current limits for the TRANSPORT motors. */
  public static class CurrentLimits {
    public static final int TRANSPORT_MAIN_SUPPLY_AMP = 60;
    public static final int TRANSPORT_MAIN_SUPPLY_TRIGGER_AMP = 55;
    public static final Time TRANSPORT_MAIN_SUPPLY_TRIGGER_TIME_SEC = Seconds.of(1);
    public static final int TRANSPORT_MAIN_STATOR_AMP = 200;
  }

  public static class SoftwareConstants {
    public static final boolean INVERTED = false;
  }

  public static class Mechanical {
    public static final double transportRatio = 33 / 11;
  }

  // Real robot PID gains for torque-current velocity control
  public static class PID {
    public static final double KS = 14.0;
    public static final double KV = 0.0;
    public static final double KP = 2.5;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
  }

  public static class transportMagicConstants {
    public static final double transportAccel = 60.0;
  }

  /** Simulation constants for the transport motor. */
  public static class Sim {
    /** One Kraken X60 FOC motor drives the transport. */
    public static final DCMotor TRANSPORT_MOTOR = DCMotor.getKrakenX60Foc(1);

    /** Number of motors driving the transport (used for DCMotorSim). */
    public static final int NUM_MOTORS = 1;

    /** Moment of inertia of the transport mechanism (kg·m²). Approximate value. */
    public static final double TRANSPORT_MOI = 0.001;

    // Sim-specific torque-current PID gains
    public static final double KS = 8.0;
    public static final double KV = 0.0;
    public static final double KP = 2.0;
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
