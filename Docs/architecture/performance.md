# Performance, Concurrency, and Risks

All findings confirmed by source reading. File:line references included.

---

## 1. Threading Model

### Threads in this codebase

| Thread | Name | Frequency | Owner |
|--------|------|-----------|-------|
| Main robot thread | `Thread-0` (WPILib) | 50 Hz (20ms) | WPILib HAL |
| Phoenix odometry thread | `PhoenixOdometryThread` | 250 Hz (CAN FD) / 100 Hz (standard) | Drive.java:121 |
| NetworkTables thread | WPILib internal | Event-driven | WPILib |
| PhotonVision NT listener | WPILib NT thread | Event-driven | VisionIOPhotonVision |

Only two threads interact with robot state: the main thread and
PhoenixOdometryThread.

### Lock: odometryLock (Drive.java:85)

```java
static final Lock odometryLock = new ReentrantLock();
```

This single `ReentrantLock` is the only explicit synchronization primitive in
the codebase.

**What it protects:**
- `SwerveModulePosition[] lastModulePositions` in Drive
- `Rotation2d rawGyroRotation` in Drive
- `SwerveDrivePoseEstimator` update calls
- The per-module odometry queues drained in Drive.periodic()

**Acquisition points:**
- `PhoenixOdometryThread.run()` — holds lock while appending samples to queues
- `Drive.periodic()` — holds lock while draining queues and calling poseEstimator

**Risk:** If the main loop overruns 20ms or the odometry thread takes unexpectedly
long (e.g., CAN bus timeout), they contend for the lock. Phoenix 6 signals are
non-blocking reads from a ring buffer, so the odometry thread should be fast.

---

## 2. Performance-Critical Loops

### Loop 1: Drive.periodic() — 50 Hz, ~3–5ms budget

Each cycle:
1. Acquire odometryLock
2. `io.updateInputs(gyroInputs)` — reads Pigeon2 signal (buffered, fast)
3. `io.updateInputs(moduleInputs[i]) × 4` — reads 4 × (drive vel + turn pos + current) = ~8 signals
4. `Logger.processInputs(...)` × 5 — serialize and write to log buffer
5. Drain up to N timestamped samples from odometry queues (typically 4–5 per 20ms on CAN FD)
6. `poseEstimator.updateWithTime(...)` × N — matrix math per sample
7. Release lock
8. `RobotState.setModuleStates(...)` — write 4 module states
9. `Logger.recordOutput(...)` × several — serialize pose, path data

**Risk:** With 5 odometry updates per cycle, step 6 runs the Kalman filter update
equation 5 times. This is more expensive than single-update systems but still
well within 20ms on the roboRIO 2 (Cortex-A9, dual core).

### Loop 2: Vision.periodic() — 50 Hz, variable cost

Each cycle processes up to 4 cameras × N observations:
- 4 × `io.updateInputs()` — NT4 reads (fast if no new data)
- Angular velocity check from RobotState (trivial)
- Per-observation: 8 rejection checks + matrix construction + consumer callback
- `poseEstimator.addVisionMeasurement()` per accepted observation

**Typical cost:** 0 observations per cycle (most cycles have no new camera data
since PhotonVision publishes at ~30 Hz). Burst cost when new data arrives:
1–3 observations × 4 cameras = up to 12 filter updates in one cycle.

**Risk:** Vision processing and Drive.periodic() both call `poseEstimator.*`
methods. They both run on the main thread (Vision is a subsystem, its
periodic runs in the same scheduler loop as Drive). No lock is needed because
they run sequentially in the scheduler, not concurrently.

However: `drive::addVisionMeasurement` is called from `Vision.periodic()`,
which runs on the main thread AFTER `Drive.periodic()` in the scheduler order
(subsystems are called in registration order). This means vision measurements
are applied after wheel odometry for the same cycle. The timestamp-based
retroactive correction in `SwerveDrivePoseEstimator` handles this correctly.

### Loop 3: CommandScheduler.run() — 50 Hz

Every trigger in the system is polled:
- `isShootSafeZone` — calls RobotState.getBroadZone() → reads pose → AllianceFlipUtil
- `isShootClear` — AND of two boolean suppliers
- `isAlignedForCurrentShot` — calls RobotState.isAlignedToHub() → heading math + debounce
- `isFlywheelSpunUp` — reads flywheel velocity from inputs struct
- All button triggers — reads joystick axis/button state

~10–15 trigger evaluations per cycle. Each is a few arithmetic operations.
Negligible cost individually; multiplied across all registered triggers
(including PathPlanner's internal event triggers during auto), total is
still <1ms.

### Loop 4: PhoenixOdometryThread — 250 Hz on CAN FD

Runs independently of the main loop:
- Reads pre-refreshed signal values (Phoenix 6 ring buffer)
- Acquires odometryLock
- Appends sample to module queues
- Releases lock

This thread must not do any heavy work. It doesn't — it only reads and
appends. The lock hold time is microseconds.

---

## 3. Identified Risks

### Risk 1: RobotState singleton is not thread-safe

**File:** RobotState.java:59–72

```java
private static RobotState instance;
public static RobotState getInstance() {
    if (instance == null) {
        instance = new RobotState();  // NOT synchronized
    }
    return instance;
}
```

`PhoenixOdometryThread` starts in Drive's constructor. If it calls any method
that eventually invokes `RobotState.getInstance()` before the main thread
finishes constructing `RobotState`, a second instance could be created (or a
partially-initialized one returned).

**In practice:** PhoenixOdometryThread only reads motor signals and appends to
queues — it does not call RobotState. The risk is theoretical.

**Severity:** Low (no observed failure path)
**Fix:** `private static final RobotState instance = new RobotState();` (eager init)

---

### Risk 2: Event trigger hardware mutation without requirement

**Files:** RobotContainer.java:349–384, ShootSequences.java:19–51

Event triggers call `intakePivot.setPivotPosition()` etc. without declaring
subsystem requirements. While `autoShootToHub` also requires and controls
`IntakePivot`, the event trigger's `runOnce` can fire mid-sequence and override
the pivot position.

**Example conflict:** `autoShootToHub` runs `autoPivotCompress` (which moves
pivot to jostle positions). If a `DeployIntake` event trigger fires at the same
moment, it moves the pivot to `down=0.0 rot` — undoing the compress motion.

**Severity:** Medium (unexpected intake behavior during auto could miss notes)
**Fix:** Either (a) don't use event triggers for subsystems also controlled by
named commands in the same auto, or (b) ensure event marker timing never
overlaps with named command execution windows.

---

### Risk 3: No teleop guard on condition-based triggers

**File:** RobotContainer.java, configureButtonBindings()

Triggers like `isShootSafeZone` evaluate field position, not button state.
During autonomous, if the robot happens to be in the alliance zone (which it
often is during shooting sequences), `isShootSafeZone` will be true. If the
flywheel is also running, `isFlywheelSpunUp` may trigger.

No binding currently causes a conflict from this (all shoot bindings also require
physical button presses), but adding new condition-based bindings in the future
could inadvertently activate during auto.

**Severity:** Low (current bindings all gate on physical buttons)
**Fix:** Add `.and(() -> DriverStation.isTeleopEnabled())` to condition-based
triggers that should only act in teleop.

---

### Risk 4: compressCancelled stale at mode transition

**File:** RobotContainer.java:675–677

```java
shootButton.onFalse(Commands.runOnce(() -> {
    compressCancelled = false;
    xCancelled = false;
    doubleCompress = false;
}));
```

Reset only fires on button release. If the robot is disabled while the button is
held, `onFalse()` never fires. At re-enable, `compressCancelled` (or others) may
remain true from the previous teleop period.

**Severity:** Low (defaults to `false` at construction; only affects single match)
**Fix:** Reset flags in `teleopInit()`.

---

### Risk 5: Subsystem registration order and Vision→Drive sequencing

**WPILib scheduler** runs `periodic()` in subsystem registration order.
`Vision.periodic()` calls `drive::addVisionMeasurement`. For this to apply the
measurement at the correct timestamp (using the retroactive buffer), Vision must
register **after** Drive.

If Vision is registered before Drive and a measurement arrives with a timestamp
before the oldest buffered odometry sample, `SwerveDrivePoseEstimator` silently
discards it.

**In practice:** The pose estimator's buffer is 1.5 seconds deep by default,
and PhotonVision latency is ~100ms. Registration order doesn't affect correctness
here, but if latency ever exceeds the buffer depth (e.g., heavy CAN congestion),
measurements are silently dropped.

**Severity:** Low (standard PhotonVision latency is well within buffer)

---

### Risk 6: Duplicate shot angle logic

**Files:** Triggers.java:171–182, DriveCommands.java:173–182

`alignOrXForShoot` uses `Triggers.isAlignedForCurrentShot` as its condition.
That trigger reads `RobotState.isAlignedToHub()`, which has its own threshold
(1.5°). The drive alignment command also uses the same angle from RobotState.

If the alignment tolerance is changed in one place and not the other, the
command will switch to X-lock at a different heading error than the visual
indicator on the dashboard suggests.

**Severity:** Low (single constant `HardwareConstants.hubAlignmentThreshold`)
**Fix:** Already mitigated — both read from the same constant. No action needed
unless the threshold is split.

---

### Risk 7: ShotCalculator and HoodPosCalculator are independent tables

**Files:** `flywheel/ShotCalculator.java`, `hood/HoodPosCalculator.java`

The flywheel RPM table and the hood angle table are maintained separately. If
a distance point is added to one but not the other, the shooting solution
becomes inconsistent at that range.

**Severity:** Low/Medium (tuning error, not a code bug)
**Recommendation:** Consider a single `ShootingParameters` lookup table that
returns both RPM and angle for a given distance, making them impossible to
desync.

---

## 4. Potential Optimization Areas

### Area 1: Vision stdDev computation

`Vision.periodic()` recomputes standard deviations for every accepted
observation every cycle. The computation includes `Math.pow(distance, 2.0)` and
several multiplications per observation. This is fast but could be cached if
camera parameters rarely change.

**Impact:** Negligible on roboRIO 2. Not a real concern.

### Area 2: RobotState computed values (getDistanceToHub, getSpeakerAngleRad)

Both methods call `AllianceFlipUtil.apply(FieldConstants.hubPosition)` every
time they are invoked. `DriveCommands` calls these in `execute()` (every 20ms).
`Triggers` calls them for debounce evaluation (every 20ms).

**Impact:** Trivial arithmetic. No concern.

### Area 3: Logger.recordOutput in execute paths

`Flywheel.isSpunUp()` (called by the `isFlywheelSpunUp` trigger every cycle)
calls `Logger.recordOutput(...)` as a side effect. Logging in a getter is
unusual and means the log value is produced even when nothing else reads it.

**Impact:** Minimal extra serialization. Not a real concern but unusual pattern
— getters with side effects can be surprising.

### Area 4: Multiple poseEstimator.updateWithTime calls per cycle

Up to 5 updates per 20ms cycle from the odometry thread (on CAN FD). Each
update runs the Kalman prediction step. On the roboRIO 2, matrix multiplication
of 3×3 matrices is fast, but this is still ~5× more work than single-update
systems.

**Impact:** Accepted tradeoff for improved odometry accuracy. Within budget.

### Area 5: 4 cameras × full scan every cycle

`Vision.periodic()` iterates all 4 cameras even when PhotonVision has published
no new data. The `io.updateInputs()` call reads from NT4 which returns the
cached value if nothing changed — fast but not zero-cost.

**Impact:** ~4 NT4 reads per cycle for unchanged data. Acceptable.

---

## 5. Summary Risk Matrix

| Risk | Severity | Likelihood | Has Mitigation? |
|------|---------|-----------|-----------------|
| Event trigger vs. named command pivot conflict | Medium | Medium | Partial (no-requirement pattern) |
| RobotState not thread-safe | Low | Low | No (theoretical only) |
| No teleop guard on condition triggers | Low | Low | No |
| compressCancelled stale at mode transition | Low | Low | No |
| ShotCalculator/HoodPosCalculator desync | Low/Medium | Low | No |
| Vision measurement dropped if >1.5s latency | Low | Very Low | No |
| Named commands registered after buildAutoChooser | High | Low | Yes (comments in code) |
| Auto start pose mismatch | Medium | Medium | Yes (checkStartPose()) |

---

## 6. Architectural Strengths

- **IO abstraction** eliminates hardware from business logic, enabling full
  replay and simulation without code changes.
- **Single pose estimator** in Drive with vision fed via callback — no redundant
  estimation and no direct Drive↔Vision coupling.
- **PhoenixOdometryThread** provides high-frequency odometry that meaningfully
  improves pose accuracy on fast swerve robots.
- **AdvantageKit replay** enables offline debugging of match incidents without
  physical hardware.
- **Named command registration order** is explicitly documented and enforced by
  code comment — a common source of subtle PathPlanner bugs is handled.
- **Distance-based shot tables** centralize tuning to two files and decouple
  distance measurement from ballistic calculation.
- **`LoggedTrigger` for isFlywheelSpunUp** — the spinup state appears in
  AdvantageScope logs, making it easy to diagnose timing issues in shoot
  sequences after a match.
