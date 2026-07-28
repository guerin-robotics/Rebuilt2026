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
 * Verifies the Xbox-mode intake toggle alternates the pivot, and that the flight-stick latch writes
 * keep the two control schemes agreeing on where the pivot ended up.
 */
class IntakePivotToggleTest {

  private static final Angle UP = HardwareConstants.CompConstants.Positions.pivotUpPos;
  private static final Angle DOWN = HardwareConstants.CompConstants.Positions.pivotDownPos;

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

  // Stands in for RobotContainer's pivotRetracted latch. Starts deployed, same as the field.
  private boolean pivotRetracted;

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
    pivotRetracted = false;
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
    CommandScheduler.getInstance()
        .schedule(
            IntakePivotCommands.togglePivot(
                pivot, () -> pivotRetracted = !pivotRetracted, () -> pivotRetracted));
    // sequence(runOnce, either(runOnce)) needs a few iterations to finish.
    for (int i = 0; i < 5; i++) {
      CommandScheduler.getInstance().run();
    }
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
    assertTrue(pivotRetracted, "latch should read retracted after the first press");
  }

  @Test
  void secondPressDeploysAgain() {
    pressToggle();
    pressToggle();

    assertEquals(2, io.commanded.size());
    assertCommanded(0, UP, "first press retracts");
    assertCommanded(1, DOWN, "second press deploys");
    assertTrue(!pivotRetracted, "latch should read deployed after the second press");
  }

  @Test
  void keepsAlternatingAcrossManyPresses() {
    for (int i = 0; i < 6; i++) {
      pressToggle();
    }

    assertEquals(6, io.commanded.size());
    for (int i = 0; i < 6; i++) {
      // Odd presses (0-indexed even) retract, the ones after them deploy.
      assertCommanded(i, i % 2 == 0 ? UP : DOWN, "press " + (i + 1) + " should alternate");
    }
  }

  @Test
  void flightStickRetractMakesTheNextTogglePressDeploy() {
    // Flight-stick button 3 sets the latch directly rather than flipping it. Without that write
    // the first Xbox press after a mode switch would re-command a position already held.
    pivotRetracted = true; // as RobotContainer's intakeInButton binding does

    pressToggle();

    assertCommanded(0, DOWN, "after a stick retract, the next toggle press should deploy");
  }

  @Test
  void flightStickDeployMakesTheNextTogglePressRetract() {
    pivotRetracted = false; // as RobotContainer's intakeOutButton binding does

    pressToggle();

    assertCommanded(0, UP, "after a stick deploy, the next toggle press should retract");
  }

  @Test
  void toggleRequiresThePivotSoItCannotFightAnotherPivotCommand() {
    var cmd =
        IntakePivotCommands.togglePivot(
            pivot, () -> pivotRetracted = !pivotRetracted, () -> pivotRetracted);

    assertTrue(
        cmd.getRequirements().contains(pivot), "the toggle moves the pivot, so it must require it");
  }
}
