package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakePivot.io.IntakePivotIO;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Verifies the Xbox-mode intake toggle alternates the pivot, and that it stays correct after the
 * pivot has been repositioned by something other than the toggle itself.
 */
class IntakePivotToggleTest {

  private static final Angle UP = HardwareConstants.CompConstants.Positions.pivotUpPos;
  private static final Angle DOWN = HardwareConstants.CompConstants.Positions.pivotDownPos;
  private static final Angle JOSTLE_UP = HardwareConstants.CompConstants.Positions.pivotJostleUpPos;
  private static final Angle JOSTLE_FIRST =
      HardwareConstants.CompConstants.Positions.pivotJostleFirstPos;

  /** Records every position the pivot was commanded to, in order. */
  private static class RecordingPivotIO implements IntakePivotIO {
    final List<Angle> commanded = new ArrayList<>();

    @Override
    public void setPivotPosition(Angle position) {
      commanded.add(position);
    }
  }

  private RecordingPivotIO io;
  private IntakePivot pivot;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();

    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();

    io = new RecordingPivotIO();
    pivot = new IntakePivot(io);
  }

  @AfterEach
  void teardown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  /** One press of the Xbox intake bumper, run to completion. */
  private void pressToggle() {
    CommandScheduler.getInstance().schedule(IntakePivotCommands.togglePivot(pivot));
    for (int i = 0; i < 5; i++) {
      CommandScheduler.getInstance().run();
    }
  }

  /**
   * Repositions the pivot the way a compress sequence or auto marker would, bypassing the toggle.
   */
  private void repositionOutsideTheToggle(Angle position) {
    pivot.setPivotPosition(position);
    io.commanded.clear();
  }

  private void assertCommanded(int index, Angle expected, String context) {
    assertTrue(io.commanded.size() > index, context + " — nothing was commanded");
    assertEquals(expected.in(Rotations), io.commanded.get(index).in(Rotations), 1e-9, context);
  }

  @Test
  void firstPressRetractsFromTheDeployedRestingState() {
    pressToggle();

    assertEquals(1, io.commanded.size(), "one press should command exactly one position");
    assertCommanded(0, UP, "first press should retract");
  }

  @Test
  void secondPressDeploysAgain() {
    pressToggle();
    pressToggle();

    assertEquals(2, io.commanded.size());
    assertCommanded(0, UP, "first press retracts");
    assertCommanded(1, DOWN, "second press deploys");
  }

  @Test
  void keepsAlternatingAcrossManyPresses() {
    for (int i = 0; i < 6; i++) {
      pressToggle();
    }

    assertEquals(6, io.commanded.size());
    for (int i = 0; i < 6; i++) {
      assertCommanded(i, i % 2 == 0 ? UP : DOWN, "press " + (i + 1) + " should alternate");
    }
  }

  // ── The toggle must follow repositioning it did not perform ──────────────
  //
  // These are the cases a latch kept next to the binding got wrong: it only knew about its own
  // presses, so after any of the paths below the next press re-commanded the position already
  // held and looked like a dead button.

  @Test
  void followsTheFlightStickRetractButton() {
    repositionOutsideTheToggle(UP);

    pressToggle();

    assertCommanded(0, DOWN, "after a stick retract, the next toggle press should deploy");
  }

  @Test
  void followsTheFlightStickDeployButton() {
    repositionOutsideTheToggle(DOWN);

    pressToggle();

    assertCommanded(0, UP, "after a stick deploy, the next toggle press should retract");
  }

  @Test
  void followsTheAutonomousRetractIntakeMarker() {
    // EventTrigger("RetractIntake") calls intakePivot.setPivotPosition(up) directly.
    repositionOutsideTheToggle(UP);

    pressToggle();

    assertCommanded(0, DOWN, "the first teleop press after an auto retract should deploy");
  }

  @Test
  void followsTheAutonomousDeployIntakeMarker() {
    repositionOutsideTheToggle(DOWN);

    pressToggle();

    assertCommanded(0, UP, "the first teleop press after an auto deploy should retract");
  }

  @Test
  void treatsAParkedCompressAsRetracted() {
    // compressPivot leaves the pivot at the jostle-up position, which is neither end of travel.
    repositionOutsideTheToggle(JOSTLE_UP);

    pressToggle();

    assertCommanded(0, DOWN, "jostle-up sits nearer retracted, so the next press should deploy");
  }

  @Test
  void treatsAnEarlyCompressStageAsDeployed() {
    repositionOutsideTheToggle(JOSTLE_FIRST);

    pressToggle();

    assertCommanded(
        0, UP, "the first jostle stage sits nearer deployed, so the next press retracts");
  }

  @Test
  void toggleRequiresThePivotSoItCannotFightAnotherPivotCommand() {
    assertTrue(
        IntakePivotCommands.togglePivot(pivot).getRequirements().contains(pivot),
        "the toggle moves the pivot, so it must require it");
  }
}
