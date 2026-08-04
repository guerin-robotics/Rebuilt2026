package frc.robot.simulation;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.List;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.Rectangle;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

/**
 * The 2026 Rebuilt arena with the field bumps optionally made passable.
 *
 * <p><b>Why this exists:</b> MapleSim's physics engine (dyn4j) is strictly two-dimensional. The
 * chassis has no Z coordinate, so every entry in the arena's obstacle map is a wall of infinite
 * height. A bump the real robot drives over and a guardrail it cannot cross are the same object to
 * the engine — there is no "drive over" to model.
 *
 * <p>Stock {@link Arena2026Rebuilt} includes the two field bumps as solid obstacles, so a simulated
 * auto that crosses one simply stops dead against it. That makes most autos impossible to run.
 *
 * <p>Neither available behavior is correct. A bump that blocks the robot entirely is wrong; a bump
 * the robot crosses at no cost is also wrong. The second is far more useful, because it lets whole
 * autos run end to end — so it is the default, and the trade is stated rather than hidden.
 *
 * <p><b>What this means for your results:</b> simulated crossings are free. The real robot loses
 * time and traction going over a bump, and can be deflected by it. Do not trust a sim auto's timing
 * across a bump, and do not conclude a path is clean because it worked here.
 */
public class RebuiltArena extends Arena2026Rebuilt {

  /**
   * Footprint of a field bump, from the stock arena's obstacle map: 47 in × 217 in. Matched by size
   * rather than by index so a maple-sim update that reorders the map cannot silently start removing
   * the wrong obstacle.
   */
  private static final double BUMP_WIDTH_METERS = 1.194;

  private static final double BUMP_LENGTH_METERS = 5.512;

  /** Dimension match tolerance, generous enough to survive small upstream tweaks. */
  private static final double MATCH_TOLERANCE_METERS = 0.05;

  public RebuiltArena() {
    if (SimulationConstants.BUMPS_ARE_PASSABLE) {
      removeBumps();
    }
  }

  /**
   * Deletes the bump obstacles from the physics world so the robot can drive across them.
   *
   * <p>Reports how many were removed. If this ever prints a count other than 2, the stock field map
   * changed and the dimensions above need rechecking — silently removing nothing would look like
   * the bump "coming back" with no explanation.
   */
  private void removeBumps() {
    List<Body> bumps = new ArrayList<>();

    for (Body body : physicsWorld.getBodies()) {
      for (int i = 0; i < body.getFixtureCount(); i++) {
        if (body.getFixture(i).getShape() instanceof Rectangle rectangle && isBump(rectangle)) {
          bumps.add(body);
          break;
        }
      }
    }

    bumps.forEach(physicsWorld::removeBody);

    if (bumps.size() != 2) {
      DriverStation.reportWarning(
          "MapleSim: expected 2 field bumps to remove, found "
              + bumps.size()
              + ". The arena's obstacle map likely changed — recheck RebuiltArena's dimensions.",
          false);
    }
  }

  private static boolean isBump(Rectangle rectangle) {
    double width = rectangle.getWidth();
    double height = rectangle.getHeight();
    return (matches(width, BUMP_WIDTH_METERS) && matches(height, BUMP_LENGTH_METERS))
        || (matches(width, BUMP_LENGTH_METERS) && matches(height, BUMP_WIDTH_METERS));
  }

  private static boolean matches(double actual, double expected) {
    return Math.abs(actual - expected) < MATCH_TOLERANCE_METERS;
  }
}
