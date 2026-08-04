package frc.robot.simulation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.junit.jupiter.api.Test;

/**
 * Covers the mechanics the human-player Fuel return depends on.
 *
 * <p>MapleSim consumes Fuel scored in the Hub and never returns it — {@code RebuiltHub} extends
 * {@code Goal}, which counts the piece and drops it. Without the return, the field drains as you
 * shoot until there is nothing left to intake.
 *
 * <p>The rate limiting and circulation accounting live in {@code MapleSimWorld.returnScoredFuel()},
 * which needs a running HAL and logger; what is checked here is that the arena genuinely accepts
 * new Fuel and that the configured target is coherent with what the arena starts with.
 */
public class FuelReturnTest {

  private static final int ARENA_STARTING_FUEL = 192;

  @Test
  public void arenaStartsWithTheExpectedFuelCount() {
    RebuiltArena arena = new RebuiltArena();
    arena.placeGamePiecesOnField();

    assertEquals(
        ARENA_STARTING_FUEL,
        arena.getGamePiecesArrayByType("Fuel").length,
        "The 2026 arena is expected to place "
            + ARENA_STARTING_FUEL
            + " Fuel. If maple-sim changed this, TARGET_FUEL_IN_CIRCULATION should be revisited.");
  }

  @Test
  public void fuelCanBeReturnedToTheField() {
    RebuiltArena arena = new RebuiltArena();
    arena.placeGamePiecesOnField();
    int before = arena.getGamePiecesArrayByType("Fuel").length;

    arena.addGamePiece(new RebuiltFuelOnField(SimulationConstants.FUEL_RETURN_POSITION));

    assertEquals(
        before + 1,
        arena.getGamePiecesArrayByType("Fuel").length,
        "Adding a RebuiltFuelOnField should put one more Fuel in play. If this fails, the human"
            + " player return cannot replenish the field.");
  }

  @Test
  public void returnPositionIsOnTheField() {
    Translation2d spawn = SimulationConstants.FUEL_RETURN_POSITION;
    double scatter = SimulationConstants.FUEL_RETURN_SCATTER_METERS;

    // Field is 16.54 x 8.05 m. Returned Fuel must land inside it even at full scatter, or it
    // would be spawned out of bounds and immediately culled.
    assertTrue(
        spawn.getX() - scatter > 0 && spawn.getX() + scatter < 16.54,
        "Fuel return X " + spawn.getX() + " +/- " + scatter + " falls outside the field");
    assertTrue(
        spawn.getY() - scatter > 0 && spawn.getY() + scatter < 8.05,
        "Fuel return Y " + spawn.getY() + " +/- " + scatter + " falls outside the field");
  }

  @Test
  public void targetCirculationIsAtLeastWhatTheArenaPlaces() {
    // A target below the starting count would mean the return never fires, which is a silent
    // way to disable the feature. Use FUEL_RETURN_ENABLED for that instead.
    assertTrue(
        SimulationConstants.TARGET_FUEL_IN_CIRCULATION >= ARENA_STARTING_FUEL,
        "TARGET_FUEL_IN_CIRCULATION ("
            + SimulationConstants.TARGET_FUEL_IN_CIRCULATION
            + ") is below the "
            + ARENA_STARTING_FUEL
            + " the arena places, so Fuel would never be returned. Set FUEL_RETURN_ENABLED = false"
            + " if that is the intent.");
  }

  @Test
  public void returnedFuelLandsNearTheDepot() {
    RebuiltArena arena = new RebuiltArena();
    arena.addGamePiece(new RebuiltFuelOnField(SimulationConstants.FUEL_RETURN_POSITION));

    Pose3d[] fuel = arena.getGamePiecesArrayByType("Fuel");
    assertTrue(fuel.length > 0, "Expected the returned Fuel to exist");

    Translation2d expected = SimulationConstants.FUEL_RETURN_POSITION;
    boolean nearDepot =
        java.util.Arrays.stream(fuel)
            .anyMatch(p -> p.getTranslation().toTranslation2d().getDistance(expected) < 0.5);
    assertTrue(nearDepot, "Returned Fuel should appear at the depot, not somewhere else");
  }
}
