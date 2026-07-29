package frc.robot.commands;

import static edu.wpi.first.units.Units.Amps;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Verifies turbo mode raises and restores the drive current limit on every exit path: normal
 * release, the safety timeout, and a competing drive command.
 *
 * <p>These cover the command through Drive and Module down to the ModuleIO boundary. The Phoenix
 * config math inside ModuleIOTalonFX is not exercised here -- that needs real hardware.
 */
class TurboModeTest {

  private static final double TURBO =
      HardwareConstants.CompConstants.Currents.driveTurboCurrent.in(Amps);
  private static final double DEFAULT =
      HardwareConstants.CompConstants.Currents.driveDefaultCurrent.in(Amps);
  private static final double TIMEOUT = HardwareConstants.CompConstants.Waits.turboMaxSeconds;

  /** Records what the command actually pushed down to the hardware layer. */
  private static class RecordingModuleIO implements ModuleIO {
    double limitAmps = Double.NaN;
    int calls = 0;

    void reset() {
      limitAmps = Double.NaN;
      calls = 0;
    }

    @Override
    public void setDriveCurrentLimit(double amps) {
      limitAmps = amps;
      calls++;
    }
  }

  // Built once: AutoBuilder.configure() is a global singleton and warns if a second Drive is
  // constructed. Per-test isolation comes from resetting the recorders instead.
  private static RecordingModuleIO[] ios;
  private static Drive drive;

  @BeforeAll
  static void createDrive() {
    assert HAL.initialize(500, 0);
    ios =
        new RecordingModuleIO[] {
          new RecordingModuleIO(),
          new RecordingModuleIO(),
          new RecordingModuleIO(),
          new RecordingModuleIO()
        };
    drive = new Drive(new GyroIO() {}, ios[0], ios[1], ios[2], ios[3]);
  }

  @BeforeEach
  void setup() {
    // The scheduler cancels any command that is not runsWhenDisabled(), so the simulated driver
    // station has to report enabled or turbo would be killed on the first run().
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();

    SimHooks.pauseTiming();
    CommandScheduler.getInstance().cancelAll();
    for (RecordingModuleIO io : ios) {
      io.reset();
    }
  }

  @AfterEach
  void teardown() {
    CommandScheduler.getInstance().cancelAll();
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    SimHooks.resumeTiming();
  }

  private void assertAllModules(double expected, String context) {
    for (int i = 0; i < 4; i++) {
      assertEquals(expected, ios[i].limitAmps, 1e-9, context + " (module " + i + ")");
    }
  }

  @Test
  void raisesLimitOnAllFourModulesWhenScheduled() {
    Command turbo = DriveCommands.turboMode(drive);
    CommandScheduler.getInstance().schedule(turbo);
    CommandScheduler.getInstance().run();

    assertTrue(turbo.isScheduled(), "turbo should still be running well before the timeout");
    assertAllModules(TURBO, "limit should be raised while turbo runs");

    // Engaging must be one-shot per module. If this ever fired every loop it would flood the CAN
    // bus with config frames.
    CommandScheduler.getInstance().run();
    CommandScheduler.getInstance().run();
    for (int i = 0; i < 4; i++) {
      assertEquals(1, ios[i].calls, "limit should be pushed once per engage (module " + i + ")");
    }
  }

  @Test
  void restoresDefaultWhenCancelled() {
    Command turbo = DriveCommands.turboMode(drive);
    CommandScheduler.getInstance().schedule(turbo);
    CommandScheduler.getInstance().run();
    assertAllModules(TURBO, "precondition: turbo engaged");

    // Simulates the driver releasing the button (whileTrue cancels on the falling edge).
    turbo.cancel();
    CommandScheduler.getInstance().run();

    assertFalse(turbo.isScheduled(), "turbo should be done after cancel");
    assertAllModules(DEFAULT, "limit must fall back to the slip-point default on release");
  }

  @Test
  void restoresDefaultWhenSafetyTimeoutExpires() {
    Command turbo = DriveCommands.turboMode(drive);
    CommandScheduler.getInstance().schedule(turbo);
    CommandScheduler.getInstance().run();
    assertAllModules(TURBO, "precondition: turbo engaged");

    // Hold to just short of the cap - still engaged.
    SimHooks.stepTiming(TIMEOUT - 0.1);
    CommandScheduler.getInstance().run();
    assertTrue(turbo.isScheduled(), "turbo should still hold just before the cap");
    assertAllModules(TURBO, "limit should still be raised just before the cap");

    // Cross the cap without ever cancelling: the timeout must drop it on its own, which is the
    // whole point of the cap - a driver leaning on the button does not get an unbounded window.
    SimHooks.stepTiming(0.2);
    CommandScheduler.getInstance().run();

    assertFalse(turbo.isScheduled(), "safety timeout should have ended turbo");
    assertAllModules(DEFAULT, "limit must fall back to default when the cap expires");
  }

  @Test
  void survivesACompetingDriveCommandAndStillRestores() {
    Command turbo = DriveCommands.turboMode(drive);
    CommandScheduler.getInstance().schedule(turbo);
    CommandScheduler.getInstance().run();
    assertAllModules(TURBO, "precondition: turbo engaged");

    // Something else takes the drive subsystem mid-turbo.
    Command hog = DriveCommands.stopWithX(drive);
    CommandScheduler.getInstance().schedule(hog);
    CommandScheduler.getInstance().run();

    // turboMode holds no requirements, so a drive-requiring command must not evict it. This is
    // the same property that keeps it from interrupting a PathPlanner auto group.
    assertTrue(
        turbo.isScheduled(),
        "turbo declares no requirements, so a drive-requiring command must not evict it");
    assertAllModules(TURBO, "limit should stay raised alongside another drive command");

    hog.cancel();
    turbo.cancel();
    CommandScheduler.getInstance().run();
    assertAllModules(DEFAULT, "limit must fall back to default once turbo ends");
  }

  @Test
  void doesNotDeclareDriveRequirement() {
    // This is what lets the "Turbo" EventTrigger run mid-path without killing the auto group.
    assertTrue(
        DriveCommands.turboMode(drive).getRequirements().isEmpty(),
        "turboMode must declare no requirements so it cannot interrupt a PathPlanner auto group");
  }
}
