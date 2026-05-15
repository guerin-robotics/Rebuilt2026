# Choreo

## Status: Not Used

Choreo is **not** integrated in this codebase. This document records what was
found, why it may have been considered, and what would be needed to add it.

---

## What Was Found

### No Choreo vendor library

`vendordeps/` contains:
- Phoenix6-26.2.0.json
- AdvantageKit.json
- PathplannerLib.json
- photonlib.json
- WPILibNewCommands.json
- Studica.json

`choreolib.json` is **not present**. ChoreoLib is not a dependency.

### No Choreo trajectory files

```
find src/main/deploy -name "*.traj"  →  (no results)
```

No `.traj` files were found in the deploy directory.

### No Choreo API calls in source

No source file imports or references:
- `choreo.Choreo`
- `choreo.trajectory.ChoreoTrajectory`
- `choreo.auto.AutoFactory`
- `edu.wpi.first.math.trajectory.Trajectory` (WPILib old-style)

### `choreoAuto: false` in all auto files

Every `.auto` file in `src/main/deploy/pathplanner/autos/` contains:

```json
"choreoAuto": false
```

This is a PathPlanner field that, when `true`, would cause PathPlanner to load
the path as a Choreo `.traj` file instead of a PathPlanner `.path` file. It is
false everywhere.

---

## Why "Choreo" May Appear in Conversation

PathPlanner 2025+ introduced trajectory generation that borrows physics-based
optimization concepts from Choreo. Team members sometimes use "Choreo" loosely
to mean "the new PathPlanner path format" or "physics-optimized paths." In this
codebase, all paths are standard PathPlanner paths — not Choreo trajectories.

---

## Difference Between PathPlanner and Choreo

| | PathPlanner | Choreo |
|-|-------------|--------|
| Path authoring | GUI spline editor | Waypoint-based optimizer |
| Trajectory type | Bézier spline with trapezoidal profile | Time-optimal using differential flatness |
| Constraint enforcement | Per-segment constraints | Global dynamics model |
| Physics model | `RobotConfig` with mass/MOI | Same |
| Runtime library | PathPlannerLib | ChoreoLib |
| File format | `.path` / `.auto` (JSON) | `.traj` (JSON) |
| WPILib auto integration | `AutoBuilder` | `AutoFactory` |
| AKit logging | `LocalADStarAK` wrapper | No built-in AKit support |
| Dynamic pathfinding | `LocalADStar` (replanning) | Not supported |

Choreo's primary advantage over PathPlanner is that its optimizer produces
theoretically faster paths because it accounts for robot dynamics across the
entire trajectory simultaneously rather than segment-by-segment. The resulting
paths are time-optimal given the robot's physics model.

PathPlanner's primary advantage for this use case is its dynamic replanning
(`LocalADStar`), which handles robot deviation and partial path following
gracefully — something Choreo doesn't support.

---

## What Would Be Needed to Add Choreo

If Choreo trajectories were desired for specific high-speed auto segments:

### 1. Add ChoreoLib vendordep

Download `choreolib.json` from the Choreo GitHub releases and place it in
`vendordeps/`. Run `./gradlew dependencies` to resolve.

### 2. Create trajectory files

Use the Choreo desktop app to design trajectories. Export `.traj` files to
`src/main/deploy/choreo/`.

### 3. Hybrid usage with PathPlanner

PathPlanner 2025+ supports loading Choreo trajectories by setting
`choreoAuto: true` in a `.auto` file and naming the path to match a `.traj`
file. This would allow mixing Choreo segments (for time-critical segments) with
PathPlanner named commands and event markers.

```json
{
  "choreoAuto": true,
  "command": {
    "type": "sequential",
    "data": {
      "commands": [
        { "type": "path", "data": { "pathName": "FastSegment" } }
      ]
    }
  }
}
```

PathPlanner would load `FastSegment.traj` from the deploy directory.

### 4. AdvantageKit logging gap

`LocalADStarAK` works only with PathPlanner paths. If Choreo trajectories are
added, their path points would not be logged through AdvantageKit's replay
pipeline. A custom `LoggableInputs` wrapper for the Choreo `AutoFactory` would
be needed for full replay fidelity.

### 5. Alliance flipping

PathPlanner handles alliance flipping automatically for `.path` files. Choreo
trajectories require manual implementation of the flip transform applied to the
`.traj` data at load time, or authoring separate red/blue trajectory pairs.

---

## Recommendation

The current PathPlanner-only setup is appropriate for this robot's auto
requirements. The `LocalADStar` dynamic replanning provides collision avoidance
and path recovery that Choreo cannot match.

Choreo would only add value if a specific auto segment is demonstrably
time-constrained and PathPlanner's path profile is leaving more than ~0.2 s on
the table compared to the theoretical time-optimal trajectory. At current
competition schedules, the bottleneck is typically shot accuracy and flywheel
spinup time, not path traversal speed.

If Choreo is considered in a future season, the hybrid PathPlanner+Choreo
approach (PathPlanner `.auto` files referencing `.traj` segments where needed)
minimizes disruption to the existing named command and event trigger framework.
