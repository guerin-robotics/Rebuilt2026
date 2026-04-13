package frc.robot.util;

import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Tracks current draw, power consumption, and cumulative energy usage for every subsystem.
 *
 * <p>Each subsystem calls {@link #reportCurrentUsage(String, boolean, double...)} once per loop
 * inside its {@code periodic()} method, passing the supply-current readings (in amps) from its
 * motors. The logger aggregates the data and writes it to AdvantageKit under the {@code
 * BatteryLogger/} namespace so it can be viewed in AdvantageScope.
 *
 * <p>Adapted from <a
 * href="https://github.com/Mechanical-Advantage/RobotCode2026Public">6328&rsquo;s
 * BatteryLogger</a>.
 */
public class BatteryLogger {

  /** Default loop period in seconds (20 ms). */
  private static final double LOOP_PERIOD_SECS = 0.02;

  // ---- Running totals for the current loop cycle ----
  private double totalCurrent = 0.0;
  private double driveCurrent = 0.0;
  private double totalPower = 0.0;
  private double totalEnergy = 0.0;

  // Battery voltage, updated each loop from RobotController
  private double batteryVoltage = 12.6;

  // RoboRIO current, set from Robot.java each loop
  private double rioCurrent = 0.0;

  // ---- Per-subsystem maps ----
  private final Map<String, Double> subsystemCurrents = new HashMap<>();
  private final Map<String, Double> subsystemPowers = new HashMap<>();
  private final Map<String, Double> subsystemEnergies = new HashMap<>();

  /**
   * Reports the current draw of one or more motors belonging to a subsystem.
   *
   * @param key Hierarchical key, e.g. {@code "Drive/Module0-Drive"}. Slashes or hyphens separate
   *     levels so parent groups are automatically aggregated.
   * @param isDrive {@code true} if this current contributes to the drivetrain total.
   * @param amps One value per motor — supply current in amps.
   */
  public void reportCurrentUsage(String key, boolean isDrive, double... amps) {
    double totalAmps = 0.0;
    for (double amp : amps) {
      totalAmps += Math.abs(amp);
    }

    if (isDrive) {
      driveCurrent += totalAmps;
    }

    double power = totalAmps * batteryVoltage;
    double energy = power * LOOP_PERIOD_SECS;

    totalCurrent += totalAmps;
    totalPower += power;
    totalEnergy += energy;

    // Store the leaf key
    subsystemCurrents.put(key, totalAmps);
    subsystemPowers.put(key, power);
    subsystemEnergies.merge(key, energy, Double::sum);

    // Aggregate parent keys (split on "/" or "-")
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

  /**
   * Call once per loop <b>after</b> all subsystem {@code periodic()} methods and the command
   * scheduler have run. Logs all aggregated data to AdvantageKit and resets per-cycle totals.
   */
  public void periodicAfterScheduler() {
    // Include fixed overhead currents
    reportCurrentUsage("Controls/roboRIO", false, rioCurrent);
    reportCurrentUsage("Controls/CANcoders", false, 0.05 * 4);
    reportCurrentUsage("Controls/Pigeon", false, 0.04);
    reportCurrentUsage("Controls/CANivore", false, 0.03);
    reportCurrentUsage("Controls/Radio", false, 0.5);

    // Log totals
    Logger.recordOutput("BatteryLogger/Current", totalCurrent);
    Logger.recordOutput("BatteryLogger/DriveCurrent", driveCurrent);
    Logger.recordOutput("BatteryLogger/Power", totalPower);
    Logger.recordOutput("BatteryLogger/Energy", joulesToWattHours(totalEnergy));

    // Log per-subsystem breakdowns
    for (var entry : subsystemCurrents.entrySet()) {
      Logger.recordOutput("BatteryLogger/Current/" + entry.getKey(), entry.getValue());
      subsystemCurrents.put(entry.getKey(), 0.0);
    }
    for (var entry : subsystemPowers.entrySet()) {
      Logger.recordOutput("BatteryLogger/Power/" + entry.getKey(), entry.getValue());
      subsystemPowers.put(entry.getKey(), 0.0);
    }
    for (var entry : subsystemEnergies.entrySet()) {
      Logger.recordOutput(
          "BatteryLogger/Energy/" + entry.getKey(), joulesToWattHours(entry.getValue()));
    }

    // Reset per-cycle totals (energy accumulates across the match)
    resetTotals();
  }

  // ---- Setters called from Robot.java each loop ----

  public void setBatteryVoltage(double voltage) {
    this.batteryVoltage = voltage;
  }

  public void setRioCurrent(double amps) {
    this.rioCurrent = amps;
  }

  // ---- Getters ----

  public double getTotalCurrent() {
    return totalCurrent;
  }

  public double getDriveCurrent() {
    return driveCurrent;
  }

  public double getTotalPower() {
    return totalPower;
  }

  public double getTotalEnergy() {
    return totalEnergy;
  }

  // ---- Helpers ----

  private void resetTotals() {
    totalPower = 0.0;
    totalCurrent = 0.0;
    driveCurrent = 0.0;
  }

  private double joulesToWattHours(double joules) {
    return joules / 3600.0;
  }
}
