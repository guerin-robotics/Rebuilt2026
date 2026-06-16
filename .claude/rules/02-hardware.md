# Hardware & CAN Rules

---

## CAN ID Map — Source of Truth

All CAN IDs live in `HardwareConstants.CanIds`. This is the only place.
Never hardcode a CAN ID inline in an IO implementation.

```java
// CORRECT
new TalonFX(HardwareConstants.CanIds.FLYWHEEL_LEADER, "rio");

// WRONG
new TalonFX(30, "rio");   ← magic number, invisible in review
```

**Full CAN layout:** See [docs/hardware-layout.md](../../docs/hardware-layout.md)

---

## CAN Bus Assignment Rules

| Bus | What belongs on it | Why |
|---|---|---|
| `"Canivore"` | All swerve devices (drive, steer, encoders, Pigeon2) | Low-latency, deterministic; swerve odometry needs sub-ms jitter |
| `"Canivore"` | Intake pivot (historical — document if changing) | Legacy placement; do not move without testing |
| `"rio"` | All other subsystems | Sufficient for non-odometry-critical control |

When adding a CAN device:
1. Decide the bus intentionally
2. Add the ID constant to `HardwareConstants.CanIds`
3. Add a comment on the constant line: `// CANivore` or `// RIO CAN`
4. Document it in [docs/hardware-layout.md](../../docs/hardware-layout.md)

---

## TalonFX Configuration Rules

Always use `PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config))`.

**Why:** CTRE devices silently ignore configuration if the CAN bus is busy at startup.
`tryUntilOk` retries until the config sticks. Without it, motors boot with default
configs (no current limits, no gains) and the first match move may brown out the robot.

Always set:
- `NeutralMode` explicitly (`Brake` or `Coast`)
- Supply current limit with `SupplyCurrentLimitEnable = true`
- Stator current limit for mechanisms that could stall

Never raise current limits without understanding why they were set. The limits in
`HardwareConstants` were tuned against real motor heating data.

---

## Motor Inversion

Inversion in Phoenix 6 has two independent axes:
- **Motor output inversion** (`MotorOutput.Inverted`) — reverses the motor shaft
- **Sensor inversion** (`FeedbackConfigs` for CANcoder) — reverses the position/velocity reading

These are not the same. Getting one wrong while fixing the other causes oscillation.

When changing inversion:
1. State which type you're changing
2. State what physical motion is expected after the change
3. Test with slow voltage before enabling closed-loop

---

## Swerve Encoder Offsets

The values in `COMP_TunerConstants.java` (e.g., `kFrontLeftEncoderOffset = 0.139404296875`)
were calibrated by physically zeroing each module. They are not guessable.

**Never change these values** unless you have run the CTRE Tuner X swerve calibration
wizard on the physical robot after a physical change (encoder swap, module rebuild).

---

## Phoenix 6 Status Signal Frequency

```java
// Standard subsystems — 50 Hz
BaseStatusSignal.setUpdateFrequencyForAll(50, signal1, signal2, ...);

// Odometry-critical signals — 250 Hz (swerve drive/steer only)
BaseStatusSignal.setUpdateFrequencyForAll(250, driveVelocity, steerPosition, ...);
```

Always call `motor.optimizeBusUtilization()` after setting signal frequencies.
Unused signals at default 100 Hz waste CAN bandwidth.

---

## Pigeon2 (Gyro)

- CAN ID: 0, Bus: CANivore
- `GyroIOPigeon2` implementation — do not modify unless there is a hardware problem
- Yaw data is consumed by `PhoenixOdometryThread` at odometry frequency
- Yaw velocity is consumed by `Vision` for the angular velocity rejection filter

If the gyro drifts or reads incorrectly:
- Check that the Pigeon2 is firmly mounted (vibration causes drift)
- Check CANivore bus health in CTRE Tuner X
- Do not adjust yaw offset in code — use `drive.setPose()` to reset field position

---

## Follower Motors

Follower TalonFX motors must use the `Follower` control request:
```java
follower.setControl(new Follower(leader.getDeviceID(), opposeLeaderDirection));
```

The `opposeLeaderDirection` boolean is `true` when the follower is physically
mounted in the opposite direction. For the flywheel: IDs 31, 32, 33 oppose; ID 34 does not.

**Never control a follower directly** while the leader is running.
