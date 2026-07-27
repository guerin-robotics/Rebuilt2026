package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import java.util.Optional;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Headless simulation of the hub-shift schedule, driving {@link HubShiftUtil} through a simulated
 * teleop clock exactly as {@code Robot.robotPeriodic()} does.
 *
 * <p>Written to verify the per-loop caching of {@code getShiftedShiftInfo()} and the FMS clock
 * resync sign fix. The GUI simulator cannot check this: the shift schedule only advances in an
 * enabled teleop with a driver-station clock, and the interesting cases (a shift boundary, an FMS
 * clock disagreeing with the local timer) are not reachable by hand.
 *
 * <p><b>The schedule this pins down.</b> Official windows are {@code [0,10,35,60,85,110]} to {@code
 * [10,35,60,85,110,140]}. With the current tuning constants the two derived offsets are {@code
 * approachingActiveFudge = -(0.75 + 1.0) = -1.75} and {@code endingActiveFudge = 3.0 - (1.0 + 2.0)
 * = 0.0}, giving:
 *
 * <pre>
 * alliance active first    [0,10) T  [10,35) T  [35,58.25) F  [58.25,85) T  [85,108.25) F  [108.25,140) T
 * alliance inactive first  [0,10) T  [10,33.25) F  [33.25,60) T  [60,83.25) F  [83.25,110) T  [110,140) T
 * </pre>
 *
 * <p><b>These are characterization assertions, not a claim the tuning is correct.</b> In particular
 * {@code endingActiveFudge} is currently exactly {@code 0.0}, so an active window closes at its
 * official boundary with no allowance for a ball already in flight — {@link
 * #activeWindowClosesAtTheOfficialBoundary()} pins that so it cannot change silently, and will need
 * updating if the fudge constants are retuned.
 */
public class HubShiftTimingSimTest {

  private static final double TOL = 1e-6;

  @BeforeAll
  static void setupHal() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
  }

  @BeforeEach
  void enteringTeleopOnBlue() {
    // Blue alliance, enabled teleop, no FMS. Without FMS the resync branch in getShiftInfo is
    // skipped entirely, so currentTime is just the local shift timer.
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setFmsAttached(false);
    DriverStationSim.notifyNewData();
  }

  @AfterAll
  static void clearOverride() {
    HubShiftUtil.setAllianceWinOverride(Optional::empty);
  }

  /**
   * Forces which schedule is in play.
   *
   * <p>{@code getFirstActiveAlliance()} treats the override as "flip the DS alliance". We are Blue,
   * so {@code false} leaves first-active = Blue = us, and {@code getSchedule()} picks
   * activeSchedule. {@code true} makes first-active = Red, so we get inactiveSchedule.
   */
  private static void ourAllianceStartsActive(boolean startsActive) {
    HubShiftUtil.setAllianceWinOverride(() -> Optional.of(!startsActive));
  }

  /** Restarts the shift clock, advances it to {@code teleopSeconds}, and refreshes the cache. */
  private static HubShiftUtil.ShiftInfo sampleAt(double teleopSeconds) {
    HubShiftUtil.initialize(); // resets shiftTimerOffset and restarts the timer
    SimHooks.stepTiming(teleopSeconds);
    HubShiftUtil.refresh(); // what Robot.robotPeriodic() calls each loop
    return HubShiftUtil.getShiftedShiftInfo();
  }

  private static boolean activeAt(double teleopSeconds) {
    return sampleAt(teleopSeconds).active();
  }

  @Test
  void scheduleWhenOurAllianceStartsActive() {
    ourAllianceStartsActive(true);

    assertTrue(activeAt(5.0), "opening transition window should be active");
    assertTrue(activeAt(20.0), "first scheduled active window");
    assertFalse(activeAt(40.0), "handed off to the other alliance");
    assertTrue(activeAt(70.0), "second active window");
    assertFalse(activeAt(95.0), "handed off again");
    assertTrue(activeAt(120.0), "endgame window should be active");
  }

  @Test
  void scheduleWhenOurAllianceStartsInactive() {
    ourAllianceStartsActive(false);

    assertTrue(activeAt(5.0), "opening transition window is active for both alliances");
    assertFalse(activeAt(20.0), "other alliance holds the hub first");
    assertTrue(activeAt(40.0), "our first active window");
    assertFalse(activeAt(70.0), "handed off");
    assertTrue(activeAt(95.0), "our second active window");
    assertTrue(activeAt(120.0), "endgame window should be active");
  }

  /**
   * An active window opens {@code approachingActiveFudge} = 1.75 s BEFORE its official start, so
   * the robot can spin up and have fuel in the air when the window actually opens.
   */
  @Test
  void activeWindowOpensEarlyByTheApproachingFudge() {
    ourAllianceStartsActive(true);

    assertFalse(activeAt(58.0), "still inactive 2.0 s before the official 60 s boundary");
    assertTrue(activeAt(58.5), "should already be active 1.5 s before the official boundary");
  }

  /**
   * Characterization of a live tuning gap: {@code endingActiveFudge} is currently exactly 0.0, so
   * an active window closes at its official boundary with no extension for fuel already in flight.
   * If the fudge constants are retuned this test is expected to fail — update it deliberately.
   */
  @Test
  void activeWindowClosesAtTheOfficialBoundary() {
    ourAllianceStartsActive(true);

    assertTrue(activeAt(34.9), "still active just before the official 35 s boundary");
    assertFalse(activeAt(35.1), "closes exactly at the boundary — endingActiveFudge is 0.0");
  }

  /**
   * Regression test for the FMS clock resync sign.
   *
   * <p>{@code getShiftInfo} defines {@code currentTime = timerValue - shiftTimerOffset}. When the
   * local clock and the field clock disagree by more than the 3 s threshold, the offset is updated
   * by {@code (currentTime - fieldTeleopTime)} and currentTime must be recomputed with that same
   * sign, which lands it exactly on fieldTeleopTime.
   *
   * <p>Setup: local timer at 20 s, field teleop clock at 40 s. Correct behaviour snaps to 40 s,
   * which is inside the inactive window [35, 58.25). The previous {@code +} sign produced 20 +
   * (-20) = 0 s, which is the opening window and reports ACTIVE — so this asserts the exact
   * opposite of the buggy result, plus the elapsed time to prove it landed on 40 s rather than
   * merely somewhere else inactive.
   */
  @Test
  void fmsResyncSnapsLocalClockOntoFieldClock() {
    ourAllianceStartsActive(true);

    DriverStationSim.setFmsAttached(true);
    DriverStationSim.setMatchTime(100.0); // fieldTeleopTime = 140 - 100 = 40 s
    DriverStationSim.notifyNewData();

    HubShiftUtil.initialize();
    SimHooks.stepTiming(20.0); // local clock is 20 s behind the field clock
    HubShiftUtil.refresh();

    HubShiftUtil.ShiftInfo info = HubShiftUtil.getShiftedShiftInfo();

    assertFalse(
        info.active(),
        "after resync the clock should read 40 s (inactive window); ACTIVE here means it read ~0 s,"
            + " which is the pre-fix '+' sign overshooting in the wrong direction");
    assertEquals(
        5.0,
        info.elapsedTime(),
        1e-3,
        "40 s is 5 s into the window starting at 35 s — proves the resync landed exactly on the"
            + " field clock rather than just anywhere inactive");
    assertEquals(
        18.25, info.remainingTime(), 1e-3, "window [35, 58.25) leaves 18.25 s remaining at 40 s");
  }

  /**
   * The whole point of the caching change: {@code getShiftedShiftInfo()} must be a pure read that
   * returns the same answer all loop long, and must pick up the new state on the next {@code
   * refresh()}. Before caching, every call recomputed, so two callers in the same loop could
   * straddle a boundary and disagree.
   */
  @Test
  void cachedValueIsStableWithinALoopAndUpdatesOnRefresh() {
    ourAllianceStartsActive(true);

    HubShiftUtil.initialize();
    SimHooks.stepTiming(20.0);
    HubShiftUtil.refresh();

    HubShiftUtil.ShiftInfo first = HubShiftUtil.getShiftedShiftInfo();
    assertTrue(first.active(), "20 s is inside the first active window");

    // Advance well past a boundary WITHOUT refreshing — every read must still agree with the
    // snapshot taken at the top of the loop.
    SimHooks.stepTiming(20.0); // now 40 s, which is an inactive window
    for (int i = 0; i < 5; i++) {
      HubShiftUtil.ShiftInfo repeat = HubShiftUtil.getShiftedShiftInfo();
      assertTrue(repeat.active(), "reads within a loop must not change");
      assertEquals(
          first.elapsedTime(), repeat.elapsedTime(), TOL, "reads within a loop must not change");
    }

    // Next loop picks up the new window.
    HubShiftUtil.refresh();
    assertFalse(
        HubShiftUtil.getShiftedShiftInfo().active(), "after refresh the 40 s state should be live");
  }
}
