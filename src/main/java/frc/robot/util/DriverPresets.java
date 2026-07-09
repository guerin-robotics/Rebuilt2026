package frc.robot.util;

import java.util.EnumMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Driver input-shaping presets, selectable from Elastic via the "Driver Preset" chooser.
 *
 * <p>Each preset defines the exponent applied to the translation magnitude and the rotation axis in
 * {@code DriveCommands}. Higher exponents give finer control near center at the cost of
 * responsiveness. Every preset's exponents are published as NetworkTables values under
 * "Tune/DriverPreset/&lt;Name&gt;/" so they can be edited in Elastic. The enum values are only the
 * boot defaults — promote a tuned value here to make it permanent, since NT edits are lost on robot
 * reboot.
 *
 * <p>The chooser and NT values are read exactly once per teleop enable, via {@link #refresh()}
 * called from {@code Robot.teleopInit()}. The drive loop reads only the cached plain-double fields.
 * Do NOT read NT or call the chooser from the 50 Hz drive loop — doing so saturated the RIO CPU and
 * stretched loop times (the reason PR #89 was reverted). To apply a preset or exponent change,
 * select/edit it in Elastic, then disable and re-enable teleop.
 */
public class DriverPresets {
  public enum Preset {
    // PARKER matches the shaping in use before presets existed (drive squared, steer ^1.35)
    PARKER("Parker", 2.0, 1.35),
    CRISTIAN("Cristian", 2.0, 2.0);

    public final String displayName;
    public final double defaultDriveExponent;
    public final double defaultSteerExponent;

    Preset(String displayName, double defaultDriveExponent, double defaultSteerExponent) {
      this.displayName = displayName;
      this.defaultDriveExponent = defaultDriveExponent;
      this.defaultSteerExponent = defaultSteerExponent;
    }
  }

  private static DriverPresets instance;

  public static DriverPresets getInstance() {
    if (instance == null) {
      instance = new DriverPresets();
    }
    return instance;
  }

  private final LoggedDashboardChooser<Preset> chooser;
  private final Map<Preset, LoggedNetworkNumber> driveExponents = new EnumMap<>(Preset.class);
  private final Map<Preset, LoggedNetworkNumber> steerExponents = new EnumMap<>(Preset.class);

  // Cached at teleop enable by refresh(); read every loop by DriveCommands
  private double activeDriveExponent = Preset.PARKER.defaultDriveExponent;
  private double activeSteerExponent = Preset.PARKER.defaultSteerExponent;

  private DriverPresets() {
    chooser = new LoggedDashboardChooser<>("Driver Preset");
    chooser.addDefaultOption(Preset.PARKER.displayName, Preset.PARKER);
    for (Preset preset : Preset.values()) {
      if (preset != Preset.PARKER) {
        chooser.addOption(preset.displayName, preset);
      }
      String root = "Tune/DriverPreset/" + preset.displayName + "/";
      driveExponents.put(
          preset, new LoggedNetworkNumber(root + "DriveExponent", preset.defaultDriveExponent));
      steerExponents.put(
          preset, new LoggedNetworkNumber(root + "SteerExponent", preset.defaultSteerExponent));
    }
  }

  /**
   * Reads the selected preset and its exponents from NetworkTables and caches them. Called once
   * from {@code Robot.teleopInit()} — never from a periodic loop.
   */
  public void refresh() {
    Preset selected = chooser.get();
    if (selected == null) {
      selected = Preset.PARKER;
    }
    activeDriveExponent = driveExponents.get(selected).get();
    activeSteerExponent = steerExponents.get(selected).get();
    Logger.recordOutput("DriverPresets/Active", selected.displayName);
    Logger.recordOutput("DriverPresets/DriveExponent", activeDriveExponent);
    Logger.recordOutput("DriverPresets/SteerExponent", activeSteerExponent);
  }

  /** Cached exponent for the translation joystick magnitude. No NT access — safe in hot paths. */
  public double getDriveExponent() {
    return activeDriveExponent;
  }

  /** Cached exponent for the rotation joystick axis. No NT access — safe in hot paths. */
  public double getSteerExponent() {
    return activeSteerExponent;
  }
}
