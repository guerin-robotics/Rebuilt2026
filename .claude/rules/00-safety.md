# Safety Rules

These rules exist because this robot runs on a 120-lb machine with fast-spinning wheels, 
high-current motors, and a live match environment. Incorrect code changes can cause:
- Robot mechanism failure mid-match
- Unexpected motion that injures people
- Brownouts from overcurrent
- Incorrect field positioning that violates game rules

---

## Absolute Hard Stops

The following require explicit user confirmation with the word "confirm" or equivalent 
before any code is written. If unsure whether a change falls here, it does.

### CAN Configuration
- **CAN IDs** — wrong ID silently controls the wrong motor
- **CAN bus assignment** (CANivore vs rio) — wrong bus means the device is invisible
- **Motor inversion flags** — inverted drive motor causes uncontrolled swerve rotation
- **Steer encoder offsets** — incorrect offset = all swerve modules point wrong direction

### Control Gains
- **PID Kp/Ki/Kd** — changing these on a live subsystem can cause oscillation or runaway
- **Feedforward Ks/Kv/Ka** — wrong FF causes undershoot/overshoot at velocity targets
- **MotionMagic cruise velocity/acceleration** — too aggressive causes mechanism damage

### Current Limits
- **Supply current limit** — too high risks brownout or wire fire
- **Stator current limit** — too high destroys motor windings
- **Never raise a current limit** without understanding why it was set where it is

### Safety Interlocks
- **Alignment timeouts** in feeder/transport wait sequences
- **Zone checks** in `Triggers.isShootClear`
- **`isShootSafeTime` / `isShootSafeZone`** triggers
- **Demo mode flag** — must be false for competition

### AdvantageKit Integrity
- **`Logger.processInputs()`** — removing this breaks replay; every periodic() must call it
- **`@AutoLog` fields** — adding/removing fields changes the log schema; real + sim must match
- **`BatteryLogger.reportCurrentUsage()`** — removing this hides brownout data

---

## High-Risk Changes Requiring Explicit Summary

Make these changes only after stating: "This changes [X] from [old] to [new]. 
The behavioral consequence is [Y]. The failure mode if wrong is [Z]."

- Any change to drive or vision subsystem code
- Any change to `RobotState` alignment booleans or tolerances
- Any change to `Triggers` composite state triggers
- Any change to `ShootSequences` or `SpitSequences`
- Any change to PathPlanner auto configuration (`AutoBuilder.configure()`)
- Any change to odometry or vision fusion (`addVisionMeasurement()`)
- Any change to `PhoenixOdometryThread`

---

## Failure Mode Catalog

Know these before modifying related code:

| Change | Failure mode |
|---|---|
| Wrong CAN ID | Controls wrong motor; mechanism moves unexpectedly |
| Wrong inversion | Swerve module fights itself; robot spins uncontrollably |
| Wrong encoder offset | All modules point wrong; robot drives sideways |
| Missing `processInputs()` | Replay broken; cannot diagnose post-match |
| Missing timeout on `waitUntil` | Robot hangs mid-sequence forever |
| Wrong current limit | Brownout (too high) or mechanism can't move (too low) |
| Modified vision filter threshold | Bad poses accepted; robot teleports during auto |
| Removed zone check | Robot shoots when unsafe |
| Wrong pass target coordinates | Pass shot misses alliance partner |
