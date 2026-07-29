package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Verifies turbo reaches the robot from the autonomous side — the PathPlanner {@code "Turbo"} event
 * marker — as opposed to the driver button, which {@link TriggerRoutingTest} covers.
 *
 * <p>{@link frc.robot.commands.TurboModeTest} already proves the command raises and restores the
 * current limit once it runs. What is unproven without this file is that the auto marker actually
 * starts it, stops it when the zone ends, and does so without evicting the path-following command.
 * That last property is the one that would silently break an auto: an event trigger holding a
 * subsystem requirement interrupts the path command, and the robot stops mid-path.
 *
 * <p>Two pieces of PathPlanner internals are reached by reflection, both {@code protected static}:
 *
 * <ul>
 *   <li>{@code EventTrigger.setCondition} — what {@code EventScheduler} calls when a path crosses a
 *       zoned marker boundary.
 *   <li>{@code EventScheduler.getEventLoop} — {@code EventTrigger} binds to this loop, <b>not</b>
 *       the CommandScheduler button loop, so {@code CommandScheduler.run()} alone never fires a
 *       marker. Polling it here is what {@code EventScheduler.execute()} does each loop of a path.
 * </ul>
 *
 * <p>Driving the markers directly rather than running a whole trajectory keeps this a test of our
 * wiring instead of a test of PathPlanner.
 */
class TurboAutoWiringTest {

  private static final String MARKER = "Turbo";
  private static final String TURBO_COMMAND = "Drive_TurboMode";
  private static final String PATH_COMMAND = "StandIn_PathFollowing";

  private static Drive drive;
  private static Method setCondition;
  private static EventLoop eventLoop;

  /** Turbo talks to the modules; this test only cares about what gets scheduled. */
  private static class StubModuleIO implements ModuleIO {}

  // Static so the scheduler listeners can be registered exactly once. Registering them per-test
  // would stack duplicates, since CommandScheduler has no way to remove a listener.
  private static final List<String> initialized = new ArrayList<>();
  private static final List<String> ended = new ArrayList<>();

  @BeforeAll
  static void setUpAll() throws Exception {
    assert HAL.initialize(500, 0);
    drive =
        new Drive(
            new GyroIO() {},
            new StubModuleIO(),
            new StubModuleIO(),
            new StubModuleIO(),
            new StubModuleIO());

    setCondition =
        EventTrigger.class.getDeclaredMethod("setCondition", String.class, boolean.class);
    setCondition.setAccessible(true);

    Method getEventLoop = EventScheduler.class.getDeclaredMethod("getEventLoop");
    getEventLoop.setAccessible(true);
    eventLoop = (EventLoop) getEventLoop.invoke(null);

    CommandScheduler.getInstance().onCommandInitialize(c -> initialized.add(c.getName()));
    CommandScheduler.getInstance().onCommandFinish(c -> ended.add(c.getName()));
    CommandScheduler.getInstance().onCommandInterrupt(c -> ended.add(c.getName()));

    // Bound once, matching RobotContainer.configureAutoBindings. Re-binding per test would leave
    // earlier triggers live on the static event loop and fire several turbo commands at once.
    new EventTrigger(MARKER).whileTrue(DriveCommands.turboMode(drive));
  }

  private static void marker(boolean active) throws Exception {
    setCondition.invoke(null, MARKER, active);
  }

  /** One auto loop: poll the marker conditions, then run the scheduler. */
  private static void autoTick() {
    eventLoop.poll();
    CommandScheduler.getInstance().run();
  }

  @BeforeEach
  void setup() throws Exception {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();

    SimHooks.pauseTiming();
    CommandScheduler.getInstance().cancelAll();

    // Settle the falling edge from the previous test before recording.
    marker(false);
    autoTick();
    initialized.clear();
    ended.clear();
  }

  @AfterEach
  void teardown() throws Exception {
    marker(false);
    CommandScheduler.getInstance().cancelAll();
    autoTick();
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    SimHooks.resumeTiming();
  }

  @Test
  void markerStartsTurboWhenThePathEntersTheZone() throws Exception {
    autoTick();
    assertFalse(
        initialized.contains(TURBO_COMMAND),
        "turbo must not run before the path reaches the marker zone");

    marker(true);
    autoTick();

    assertTrue(
        initialized.contains(TURBO_COMMAND),
        "entering the Turbo marker zone must schedule turbo; got " + initialized);
  }

  @Test
  void markerStopsTurboWhenThePathLeavesTheZone() throws Exception {
    marker(true);
    autoTick();
    assertTrue(initialized.contains(TURBO_COMMAND), "precondition: turbo engaged inside the zone");

    // Zoned markers have a start AND an end position; leaving the zone drops the condition.
    marker(false);
    autoTick();

    assertTrue(
        ended.contains(TURBO_COMMAND),
        "leaving the Turbo zone must end turbo so the raised current limit does not persist for the"
            + " rest of the auto; got "
            + ended);
  }

  @Test
  void markerDoesNotInterruptThePathFollowingCommand() throws Exception {
    // Stand-in for the PathPlanner auto group: it requires the drive subsystem, exactly like a
    // path-following command does.
    Command pathFollowing = Commands.run(() -> {}, drive).withName(PATH_COMMAND);
    pathFollowing.schedule();
    autoTick();
    assertTrue(pathFollowing.isScheduled(), "precondition: the path command is running");

    marker(true);
    autoTick();
    autoTick();

    assertTrue(
        initialized.contains(TURBO_COMMAND), "precondition: the marker started turbo mid-path");
    assertTrue(
        pathFollowing.isScheduled(),
        "the Turbo marker must not interrupt the path-following command — if it does, the robot"
            + " stops mid-path. This is why turboMode declares no requirements.");
    assertFalse(
        ended.contains(PATH_COMMAND),
        "the path command must not have been interrupted by the Turbo marker; ended = " + ended);

    pathFollowing.cancel();
  }
}
