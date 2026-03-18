package frc.robot.util;

import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Logs current draw, power draw, and energy usage for the entire robot.
 *
 * <p>Each subsystem calls {@link #reportCurrentUsage(String, double...)} with its supply current(s)
 * every loop. The logger then aggregates these into:
 *
 * <ul>
 *   <li><b>Current (amps)</b> — affects battery voltage moment-to-moment; high total triggers
 *       brownouts.
 *   <li><b>Power (watts)</b> — current × voltage; shows which subsystems drain the battery fastest.
 *   <li><b>Energy (watt-hours)</b> — cumulative power over time; the most critical metric for match
 *       battery planning.
 * </ul>
 *
 * <p>Metrics are logged per-mechanism, per-group (e.g. "Drive"), and as a robot-wide total.
 *
 * <p>Adapted from Team 6328 (Mechanical Advantage) BatteryLogger.
 */
public class BatteryLogger {

  /** Duration of one robot loop in seconds (20 ms). */
  private static final double LOOP_PERIOD_SECS = 0.02;

  // ── Running totals (reset each loop except energy, which accumulates) ──
  private double totalCurrent = 0.0;
  private double totalPower = 0.0;
  private double totalEnergy = 0.0;

  /** Battery voltage used for power calculations. Updated externally each loop. */
  private double batteryVoltage = 12.6;

  /** roboRIO supply current reported externally each loop. */
  private double rioCurrent = 0.0;

  // Per-subsystem maps (keys use "/" for hierarchy, e.g. "Drive/Module0-Drive")
  private Map<String, Double> subsystemCurrents = new HashMap<>();
  private Map<String, Double> subsystemPowers = new HashMap<>();
  private Map<String, Double> subsystemEnergies = new HashMap<>();

  // ── Setters ─────────────────────────────────────────────────────────────

  /** Set the current battery voltage (call once per loop from RobotController). */
  public void setBatteryVoltage(double voltage) {
    this.batteryVoltage = voltage;
  }

  /** Set the roboRIO supply current (call once per loop from RobotController). */
  public void setRioCurrent(double current) {
    this.rioCurrent = current;
  }

  // ── Reporting ───────────────────────────────────────────────────────────

  /**
   * Report the supply current draw for a mechanism.
   *
   * <p>Use hierarchical keys separated by "/" or "-" to group mechanisms. For example:
   *
   * <ul>
   *   <li>"Drive/Module0-Drive" and "Drive/Module0-Turn" will be summed under "Drive/Module0" and
   *       then under "Drive".
   *   <li>"Flywheel" has no sub-groups.
   * </ul>
   *
   * @param key hierarchical name (e.g. "Drive/Module0-Drive")
   * @param amps one or more supply-current readings (all will be summed)
   */
  public void reportCurrentUsage(String key, double... amps) {
    double totalAmps = 0.0;
    for (double amp : amps) {
      totalAmps += Math.abs(amp);
    }

    double power = totalAmps * batteryVoltage;
    double energy = power * LOOP_PERIOD_SECS;

    // Add to robot-wide totals
    totalCurrent += totalAmps;
    totalPower += power;
    totalEnergy += energy;

    // Store per-key values
    subsystemCurrents.put(key, totalAmps);
    subsystemPowers.put(key, power);
    subsystemEnergies.merge(key, energy, Double::sum);

    // Aggregate into parent groups
    // e.g. "Drive/Module0-Drive" → parent "Drive/Module0" → parent "Drive"
    String[] keys = key.split("/|-");
    if (keys.length < 2) {
      return;
    }

    String subkey = "";
    for (int i = 0; i < keys.length - 1; i++) {
      subkey += keys[i];
      if (i < keys.length - 2) {
        subkey += "/";
      }
      subsystemCurrents.merge(subkey, totalAmps, Double::sum);
      subsystemPowers.merge(subkey, power, Double::sum);
      subsystemEnergies.merge(subkey, energy, Double::sum);
    }
  }

  // ── Periodic logging (call AFTER the scheduler runs each loop) ─────────

  /**
   * Log all energy metrics to AdvantageKit.
   *
   * <p>Call this once per loop <b>after</b> all subsystem periodic() methods have run, so every
   * subsystem has had a chance to report its current draw.
   */
  public void periodic() {
    // Report fixed-draw devices
    reportCurrentUsage("Controls/roboRIO", rioCurrent);
    reportCurrentUsage("Controls/CANcoders", 0.05 * 4); // 4 swerve CANcoders
    reportCurrentUsage("Controls/Pigeon", 0.04);
    reportCurrentUsage("Controls/CANivore", 0.03);
    reportCurrentUsage("Controls/Radio", 0.5);

    // Log robot-wide totals
    Logger.recordOutput("EnergyLogger/Current", totalCurrent, "amps");
    Logger.recordOutput("EnergyLogger/Power", totalPower, "watts");
    Logger.recordOutput("EnergyLogger/Energy", joulesToWattHours(totalEnergy), "watt hours");

    // Log per-subsystem values
    for (var entry : subsystemCurrents.entrySet()) {
      Logger.recordOutput("EnergyLogger/Current/" + entry.getKey(), entry.getValue(), "amps");
      subsystemCurrents.put(entry.getKey(), 0.0);
    }
    for (var entry : subsystemPowers.entrySet()) {
      Logger.recordOutput("EnergyLogger/Power/" + entry.getKey(), entry.getValue(), "watts");
      subsystemPowers.put(entry.getKey(), 0.0);
    }
    for (var entry : subsystemEnergies.entrySet()) {
      Logger.recordOutput(
          "EnergyLogger/Energy/" + entry.getKey(),
          joulesToWattHours(entry.getValue()),
          "watt hours");
    }

    // Reset per-loop totals (energy accumulates, current/power do not)
    totalPower = 0.0;
    totalCurrent = 0.0;
  }

  // ── Getters ─────────────────────────────────────────────────────────────

  public double getTotalCurrent() {
    return totalCurrent;
  }

  public double getTotalPower() {
    return totalPower;
  }

  public double getTotalEnergy() {
    return totalEnergy;
  }

  // ── Helpers ─────────────────────────────────────────────────────────────

  private double joulesToWattHours(double joules) {
    return joules / 3600.0;
  }
}
