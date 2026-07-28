package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Verifies which physical button each intake / turbo function listens to in each drive mode.
 *
 * <p>The mode gate matters for more than convenience: flight-stick 3 and 4 are the alliance flipper
 * while the Xbox drives, so the intake triggers must go dead there or one press would fire two
 * unrelated functions.
 */
class TriggerRoutingTest {

  private static final int XBOX = HardwareConstants.ControllerConstants.XboxControllerPort;
  private static final int STICK = HardwareConstants.ControllerConstants.JoystickControllerPort;

  private static final int LB = XboxController.Button.kLeftBumper.value;
  private static final int RB = XboxController.Button.kRightBumper.value;

  // CommandJoystick.button(n) is 1-indexed and maps straight onto the sim's button index.
  private static final int STICK_INTAKE_IN = 3;
  private static final int STICK_INTAKE_OUT = 4;
  private static final int STICK_TURBO = 9;

  private static boolean originalMode;

  @BeforeAll
  static void initHal() {
    assert HAL.initialize(500, 0);
    originalMode = HardwareConstants.ControllerConstants.XBOX_DRIVE_MODE;
  }

  @BeforeEach
  void setup() {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    // Enough buttons for the flight stick's 16 and the Xbox's 10.
    DriverStationSim.setJoystickButtonCount(XBOX, 12);
    DriverStationSim.setJoystickButtonCount(STICK, 16);
    releaseAll();
  }

  @AfterEach
  void teardown() {
    releaseAll();
    HardwareConstants.ControllerConstants.XBOX_DRIVE_MODE = originalMode;
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  private void releaseAll() {
    for (int b = 1; b <= 16; b++) {
      DriverStationSim.setJoystickButton(STICK, b, false);
      if (b <= 12) {
        DriverStationSim.setJoystickButton(XBOX, b, false);
      }
    }
    refresh();
  }

  /** Pushes the simulated button state through to the DriverStation the Triggers read. */
  private void refresh() {
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  private void press(int port, int button) {
    DriverStationSim.setJoystickButton(port, button, true);
    refresh();
  }

  private void xboxDrives(boolean on) {
    HardwareConstants.ControllerConstants.XBOX_DRIVE_MODE = on;
  }

  private Triggers t() {
    return Triggers.getInstance();
  }

  // ── Xbox mode ────────────────────────────────────────────────────────────

  @Test
  void xboxMode_leftBumperDrivesTheIntakeToggle() {
    xboxDrives(true);
    press(XBOX, LB);

    assertTrue(t().intakePivotToggleButton().getAsBoolean(), "LB should be the intake toggle");
    assertFalse(t().turboButton().getAsBoolean(), "LB must not trip turbo");
  }

  @Test
  void xboxMode_rightBumperDrivesTurbo() {
    xboxDrives(true);
    press(XBOX, RB);

    assertTrue(t().turboButton().getAsBoolean(), "RB should be turbo");
    assertFalse(t().intakePivotToggleButton().getAsBoolean(), "RB must not trip the intake toggle");
  }

  @Test
  void xboxMode_separateIntakeButtonsAreDead() {
    xboxDrives(true);
    press(XBOX, LB);

    // Both directions live on the toggle here; the absolute-direction triggers must stay quiet.
    assertFalse(t().intakeInButton().getAsBoolean(), "intake in has no Xbox home");
    assertFalse(t().intakeOutButton().getAsBoolean(), "intake out has no Xbox home");
  }

  @Test
  void xboxMode_flightStickIntakeButtonsDoNotFireTheIntake() {
    // The regression this guards: flight stick 3 and 4 are the alliance flipper in Xbox mode.
    // If the intake triggers stayed bound to the device rather than the mode, one press would
    // both flip the alliance and move the pivot.
    xboxDrives(true);
    press(STICK, STICK_INTAKE_IN);
    press(STICK, STICK_INTAKE_OUT);

    assertFalse(
        t().intakeInButton().getAsBoolean(),
        "flight stick 3 is the alliance flipper in Xbox mode, not intake in");
    assertFalse(
        t().intakeOutButton().getAsBoolean(),
        "flight stick 4 is the alliance flipper in Xbox mode, not intake out");
    assertTrue(t().allianceWinFlipper().getAsBoolean(), "3 and 4 should still flip the alliance");
  }

  @Test
  void xboxMode_flightStickTurboButtonIsDead() {
    xboxDrives(true);
    press(STICK, STICK_TURBO);

    assertFalse(t().turboButton().getAsBoolean(), "turbo is on RB while the Xbox drives");
  }

  // ── Flight-stick mode ────────────────────────────────────────────────────

  @Test
  void stickMode_button3StillDrivesIntakeIn() {
    xboxDrives(false);
    press(STICK, STICK_INTAKE_IN);

    assertTrue(t().intakeInButton().getAsBoolean(), "button 3 should still be intake in");
    assertFalse(t().intakeOutButton().getAsBoolean(), "button 3 must not also fire intake out");
  }

  @Test
  void stickMode_button4StillDrivesIntakeOut() {
    xboxDrives(false);
    press(STICK, STICK_INTAKE_OUT);

    assertTrue(t().intakeOutButton().getAsBoolean(), "button 4 should still be intake out");
    assertFalse(t().intakeInButton().getAsBoolean(), "button 4 must not also fire intake in");
  }

  @Test
  void stickMode_intakeButtonsAreIndependent() {
    // The two-button behaviour the drive team is used to: they are separate, not a toggle.
    xboxDrives(false);
    press(STICK, STICK_INTAKE_IN);
    press(STICK, STICK_INTAKE_OUT);

    assertTrue(t().intakeInButton().getAsBoolean(), "3 held");
    assertTrue(t().intakeOutButton().getAsBoolean(), "4 held");
    assertFalse(
        t().intakePivotToggleButton().getAsBoolean(),
        "the toggle is Xbox-only and must stay dead on the flight stick");
  }

  @Test
  void stickMode_button9DrivesTurbo() {
    xboxDrives(false);
    press(STICK, STICK_TURBO);

    assertTrue(t().turboButton().getAsBoolean(), "button 9 should be turbo");
  }

  @Test
  void stickMode_xboxBumpersAreDead() {
    xboxDrives(false);
    press(XBOX, LB);
    press(XBOX, RB);

    assertFalse(t().turboButton().getAsBoolean(), "turbo is on stick 9 while the stick drives");
    assertFalse(t().intakePivotToggleButton().getAsBoolean(), "the toggle is Xbox-mode only");
  }

  @Test
  void turboAndIntakeNeverShareAButtonInEitherMode() {
    for (boolean xbox : new boolean[] {true, false}) {
      xboxDrives(xbox);
      String mode = xbox ? "Xbox mode" : "flight-stick mode";

      press(xbox ? XBOX : STICK, xbox ? RB : STICK_TURBO);
      assertTrue(t().turboButton().getAsBoolean(), "turbo should be live in " + mode);
      assertFalse(t().intakeInButton().getAsBoolean(), "turbo must not fire intake in, " + mode);
      assertFalse(t().intakeOutButton().getAsBoolean(), "turbo must not fire intake out, " + mode);
      assertFalse(
          t().intakePivotToggleButton().getAsBoolean(),
          "turbo must not fire the intake toggle, " + mode);
      releaseAll();
    }
  }
}
