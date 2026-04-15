# CPU Optimization Review — 2026-04-15

Branch: `cpu-optimization`

**Problem:** CPU usage near 100% even at idle. Root cause: redundant calculations running every
robot loop cycle (every 20ms), including zone calculations that re-called the same sub-methods
dozens of times per invocation, dead `@AutoLogOutput` methods running expensive math for no
consumer, and unused methods that were never called from any other code.

---

## Findings

### How the Robot Loop Works (Context)

The WPILib `CommandScheduler.run()` is called once per 20ms loop in `Robot.robotPeriodic()`. Every
subsystem's `periodic()` method runs here. Additionally, AdvantageKit calls every method annotated
with `@AutoLogOutput` on any registered object **every single loop cycle** — this is how RobotState
auto-logs its fields. If an `@AutoLogOutput` method does expensive work, that work runs every 20ms
unconditionally.

---

### Issue 1 — `getApproachingZone()` internally called sub-methods up to 12 times per invocation

**File:** `RobotState.java`

`getApproachingZone(pose)` had 6 if/else branches, each calling `getApproachingZoneX(pose)` and
`getApproachingZoneY(pose)` separately without caching:

```java
// BEFORE — called getApproachingZoneX up to 6 times and getApproachingZoneY up to 6 times
if ((getApproachingZoneX(pose) == ...) && (getApproachingZoneY(pose) == ...)) { ... }
else if ((getApproachingZoneX(pose) == ...) && (getApproachingZoneY(pose) == ...)) { ... }
// ... 4 more branches
```

Each of those sub-method calls internally called `getBroadZone(pose)` → `AllianceFlipUtil.applyX()`
again. So a single `getApproachingZone()` call could trigger **12+ redundant computations** of the
same values.

`getApproachingZoneY()` also called `getBroadZone(pose)` **twice** internally without caching.

**Fix:** Cache the results of `getApproachingZoneX()` and `getApproachingZoneY()` in local
variables before the if/else chain. Cache `getBroadZone()` inside `getApproachingZoneY()`.

---

### Issue 2 — `getSpecificZone()` called `getBroadZone()` up to 4 times per invocation

**File:** `RobotState.java`

Same pattern — each branch of the if/else re-called `getBroadZone(pose)`:

```java
// BEFORE — getBroadZone called at each branch
if ((getBroadZone(pose) == ALLIANCE_ZONE) && ...) { ... }
else if (getBroadZone(pose) == ALLIANCE_TRENCH) { ... }
else if (getBroadZone(pose) == OPPOSING_TRENCH) { ... }
else if ((getBroadZone(pose) == OPPOSING_ZONE) && ...) { ... }
```

**Fix:** Cache `getBroadZone(pose)` in a local variable at the top of the method.

---

### Issue 3 — `Robot.robotPeriodic()` called `getEstimatedPose()` 4 separate times

**File:** `Robot.java`

```java
// BEFORE — 4 separate calls to getEstimatedPose() in the same loop cycle
fieldMap.setRobotPose(RobotState.getInstance().getEstimatedPose());
RobotState.getInstance().getBroadZone(RobotState.getInstance().getEstimatedPose());
RobotState.getInstance().getSpecificZone(RobotState.getInstance().getEstimatedPose());
RobotState.getInstance().getApproachingZone(RobotState.getInstance().getEstimatedPose());
```

Each of the zone methods also called `getEstimatedPose()` again internally via sub-calls.

**Fix:** Cache pose once at the top of the block and pass it to all three logging calls.

---

### Issue 4 — `getPassTarget()` called `DriverStation.getAlliance()` twice, creating Optional allocations

**File:** `RobotState.java`

```java
// BEFORE — creates 2 Optional objects every call
if (DriverStation.getAlliance().isEmpty()
    || DriverStation.getAlliance().get() == DriverStation.Alliance.Red) { ... }
```

`Robot.robotPeriodic()` already calls `AllianceFlipUtil.refresh()` once per loop to cache the
alliance color for exactly this reason. `getPassTarget()` bypassed this and called
`DriverStation.getAlliance()` directly.

Also, `getPassTarget()` called `RobotState.getInstance().getEstimatedPose()` instead of just
`getEstimatedPose()` (unnecessary singleton lookup from inside the singleton itself).

**Fix:** Replaced `DriverStation.getAlliance()` with `AllianceFlipUtil.shouldFlip()` (free cached
read). Replaced `RobotState.getInstance().getEstimatedPose()` with direct `getEstimatedPose()`.

---

### Issue 5 — `getDistanceToAllianceHub()` logged its return value twice per loop

**File:** `RobotState.java`

The method had both an `@AutoLogOutput` annotation (AdvantageKit auto-logs the return value every
loop) AND a manual `Logger.recordOutput()` call inside the method body. This wrote the same value
to two different log keys every loop.

**Fix:** Removed the redundant manual `Logger.recordOutput()` call.

---

## Dead Code Removed

These methods were never called from any other code. In two cases they also had `@AutoLogOutput`
annotations, meaning AdvantageKit was calling them every loop cycle just to log a value nobody
reads.

| Method | File | Why Costly |
|---|---|---|
| `getDistanceToOpposingHub()` | `RobotState.java` | `@AutoLogOutput` — ran `AllianceFlipUtil.apply()` + `getEstimatedPose()` + `getDistance()` every loop with zero consumers |
| `getOpposingHubTarget()` | `RobotState.java` | Only caller was `getDistanceToOpposingHub()` |
| `getShootAngleForZoneAndTime()` | `RobotState.java` | Never called. Internally called `getPassTarget()` **twice** in one expression, plus `getAngleToAllianceHub()`, `Triggers.getInstance()`, and more |
| `getShootAngleForZone()` | `RobotState.java` | Never called. Same expensive chain |
| `getFuturePose()` | `RobotState.java` | Never called. Would have called `getEstimatedPose()` 3 times and `getFieldRelativeVelocity()` 3 times per invocation |
| `isHoodSafe(Pose2d)` | `Triggers.java` | Only call site was commented out in `Hood.java`. Also had a design bug: created a **new `LoggedTrigger` object every call** (GC allocation on every invocation), and the captured `Pose2d` parameter was never re-evaluated inside the lambda |

---

## Changes Made

### `RobotState.java`

- **Removed** `getDistanceToOpposingHub()` — dead `@AutoLogOutput` method
- **Removed** `getOpposingHubTarget()` — only caller was removed above
- **Removed** `getShootAngleForZoneAndTime()` — never called externally
- **Removed** `getShootAngleForZone()` — never called externally
- **Fixed** `getSpecificZone()` — cache `getBroadZone()` result (was called up to 4 times)
- **Fixed** `getApproachingZoneY()` — cache `getBroadZone()` result (was called up to 2 times)
- **Fixed** `getApproachingZone()` — cache `getApproachingZoneX()` and `getApproachingZoneY()` results (each was called up to 6 times)
- **Fixed** `getPassTarget()` — replace `DriverStation.getAlliance()` with `AllianceFlipUtil.shouldFlip()`; replace `RobotState.getInstance().getEstimatedPose()` with `getEstimatedPose()`
- **Fixed** `getDistanceToAllianceHub()` — remove redundant `Logger.recordOutput()` (already logged by `@AutoLogOutput`)
- **Removed** unused imports: `DriverStation`, `HubShiftUtil`

### `Robot.java`

- **Fixed** `robotPeriodic()` — cache `getEstimatedPose()` once instead of calling it 4 times in the same block

### `Triggers.java`

- **Removed** `isHoodSafe(Pose2d)` — dead code with per-call object allocation bug
- **Removed** unused import: `Pose2d`

---

## Per-Loop Call Count Reduction (Estimated)

| Computation | Before (calls/loop) | After (calls/loop) |
|---|---|---|
| `getBroadZone()` invocations from zone logging block | ~10+ | 3 |
| `getApproachingZoneX/Y()` invocations from one `getApproachingZone()` call | Up to 12 | 2 |
| `getEstimatedPose()` calls in `robotPeriodic()` logging block | 4 | 1 |
| `DriverStation.getAlliance()` Optional allocations per `getPassTarget()` call | 2 | 0 |
| `@AutoLogOutput` methods doing expensive work for no consumer | 2 | 0 |
