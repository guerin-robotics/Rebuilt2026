package frc.robot.subsystems.transport;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Time;

public class TransportConstants {

  /** Current limits for the TRANSPORT motors. */
  public static class CurrentLimits {
    public static final int TRANSPORT_MAIN_SUPPLY_AMP = 40;
    public static final int TRANSPORT_MAIN_SUPPLY_TRIGGER_AMP = 35;
    public static final Time TRANSPORT_MAIN_SUPPLY_TRIGGER_TIME_SEC = Seconds.of(1);
    public static final int TRANSPORT_MAIN_STATOR_AMP = 100;
  }

  public static class SoftwareConstants {
    public static final boolean INVERTED = false;
  }

  public static class Mechanical {
    public static final double transportRatio = 0.44;
  }

  // NOT Tuned yet - placeholder values
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

    // Sim PID gains for TalonFX closed-loop in simulation
    public static final double KS = 0.0;
    public static final double KV = 0.12;
    public static final double KP = 1.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
  }
}
