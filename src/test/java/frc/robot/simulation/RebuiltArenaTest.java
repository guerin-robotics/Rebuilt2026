package frc.robot.simulation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.Rectangle;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.junit.jupiter.api.Test;

/**
 * Locks in that {@link RebuiltArena} removes exactly the two field bumps and nothing else.
 *
 * <p>The bumps are matched by physical size, not by index, so a maple-sim update that reorders or
 * resizes the obstacle map would silently stop removing them — and the only symptom would be autos
 * mysteriously stopping dead again mid-path. This test turns that into a build failure.
 */
public class RebuiltArenaTest {

  /** {@code physicsWorld} is protected; reach it through a subclass. */
  private static class ProbeStock extends Arena2026Rebuilt {
    List<Body> bodies() {
      return physicsWorld.getBodies();
    }
  }

  private static class ProbeCustom extends RebuiltArena {
    List<Body> bodies() {
      return physicsWorld.getBodies();
    }
  }

  private static final double BUMP_WIDTH_METERS = 1.194;
  private static final double BUMP_LENGTH_METERS = 5.512;
  private static final double TOLERANCE = 0.05;

  @Test
  public void stockArenaContainsExactlyTwoBumps() {
    assertEquals(
        2,
        countBumps(new ProbeStock().bodies()),
        "Stock Arena2026Rebuilt should contain 2 bump obstacles. If this changed, maple-sim's"
            + " field map was updated and RebuiltArena's dimensions need rechecking.");
  }

  @Test
  public void customArenaRemovesBothBumps() {
    assertEquals(
        0,
        countBumps(new ProbeCustom().bodies()),
        "RebuiltArena should leave no bump obstacles when BUMPS_ARE_PASSABLE is set.");
  }

  @Test
  public void customArenaRemovesNothingElse() {
    int stock = new ProbeStock().bodies().size();
    int custom = new ProbeCustom().bodies().size();

    assertEquals(
        2,
        stock - custom,
        "RebuiltArena should remove the 2 bumps and no other obstacle — walls, the Hub"
            + " structures, and the Outpost must all survive.");
  }

  @Test
  public void fieldPerimeterSurvives() {
    // Four border segments keep the robot on the field. Removing one would let it drive off.
    long segments =
        new ProbeCustom()
            .bodies().stream()
                .filter(
                    body ->
                        body.getFixtureCount() > 0
                            && body.getFixture(0).getShape() instanceof org.dyn4j.geometry.Segment)
                .count();
    assertTrue(segments >= 4, "Expected the 4 field perimeter walls to remain, found " + segments);
  }

  private static int countBumps(List<Body> bodies) {
    int count = 0;
    for (Body body : bodies) {
      for (int i = 0; i < body.getFixtureCount(); i++) {
        if (body.getFixture(i).getShape() instanceof Rectangle rectangle && isBump(rectangle)) {
          count++;
          break;
        }
      }
    }
    return count;
  }

  private static boolean isBump(Rectangle rectangle) {
    double w = rectangle.getWidth();
    double h = rectangle.getHeight();
    return (matches(w, BUMP_WIDTH_METERS) && matches(h, BUMP_LENGTH_METERS))
        || (matches(w, BUMP_LENGTH_METERS) && matches(h, BUMP_WIDTH_METERS));
  }

  private static boolean matches(double actual, double expected) {
    return Math.abs(actual - expected) < TOLERANCE;
  }
}
