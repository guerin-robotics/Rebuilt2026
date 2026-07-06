package frc.robot.util;

import java.util.EnumMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Driver input-shaping presets, selectable live from Elastic via the "Driver Preset" chooser.
 *
 * <p>Each preset defines the exponent applied to the translation magnitude and the rotation axis in
 * {@code DriveCommands}. Higher exponents give finer control near center at the cost of
 * responsiveness. Every preset's exponents are published as live NetworkTables values under
 * "Tune/DriverPreset/&lt;Name&gt;/" so they can be edited in Elastic while driving. The enum values
 * are only the boot defaults — promote a tuned value here to make it permanent, since NT edits are
 * lost on robot reboot.
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

  private Preset getSelected() {
    Preset selected = chooser.get();
    return selected != null ? selected : Preset.PARKER;
  }

  /** Exponent applied to the translation joystick magnitude (deadband already applied). */
  public double getDriveExponent() {
    Preset selected = getSelected();
    double exponent = driveExponents.get(selected).get();
    Logger.recordOutput("DriverPresets/Active", selected.displayName);
    Logger.recordOutput("DriverPresets/DriveExponent", exponent);
    return exponent;
  }

  /** Exponent applied to the rotation joystick axis (sign preserved by the caller). */
  public double getSteerExponent() {
    Preset selected = getSelected();
    double exponent = steerExponents.get(selected).get();
    Logger.recordOutput("DriverPresets/SteerExponent", exponent);
    return exponent;
  }
}
