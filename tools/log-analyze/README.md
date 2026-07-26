# Match log analyzer

Drivetrain metrics from AdvantageKit `.wpilog` match logs — command smoothness, drive current
saturation, brownouts, and auto path-following error. Built to measure whether a drivetrain change
actually helped, by comparing an event's logs against the baseline below.

## Running it

From the repo root:

```powershell
.\tools\log-analyze\analyze.ps1 logs\*.wpilog
```

It resolves the project's compile classpath through Gradle, compiles `MatchAnalyze.java` against the
same WPILib and PathPlanner jars the robot uses, and runs it. Truncated logs (robot lost power
mid-write) are handled — it stops at the bad record and reports everything up to that point.

## What each number means

**Commanded accel / brake** — how hard the *command* changed, in the field frame, in m/s². This is
what the code asked for, not what the robot achieved. Values far above the ceiling mean the command
was asking for a velocity step that the hardware could only answer with a current spike.

**Command step** — the same thing in m/s per command update. Easier to reason about: a 1.5 m/s step
is the command jumping over a third of the robot's top speed between two loops.

**Drive stator current** — percentage of samples at or above the configured slip current
(`kSlipCurrent` in `COMP_TunerConstants`). Sustained time at the limit means torque-saturated
wheels, which is wasted current if it exceeds what the tires can hold.

**Brownouts** — split by auto and teleop, since they usually have different causes, plus what was
drawing current beforehand.

> Caveat on the current shares: `BatteryLogger` sums **stator** currents, and stator current only
> approaches battery draw at high duty cycle. A mechanism running fast (flywheel) is represented
> faithfully; one making peak torque at low speed (drive) has stator current well above what it
> actually pulls from the battery. Treat the shares as directional, not exact.

**Path-follow error** — distance between PathPlanner's target pose and the estimated pose. Error
that grows during acceleration and collapses when the trajectory slows means the robot cannot meet
the trajectory's acceleration demand; a constant offset means something else, usually pose
estimation.

## Baseline: July 2026 Indiana event (9 matches, q6 → e2)

Measured **before** the traction-control work (slew limiting, COF correction). Auto paths were
still planned at 7.0 m/s² globals at this point.

| Metric | Baseline |
|---|---|
| Brownouts | **209 teleop, 0 auto** |
| Min battery voltage | 6.13 V teleop, 7.11 V auto |
| Accounted current at brownout | 427 A, drive 79%, flywheel 16% |
| Drive stator peak | 81–120 A (against an 80 A limit) |
| Command step, median | 0.145 m/s per update |
| Command step, p95 | 1.49 m/s per update |
| Auto path-follow error | mean 0.46–0.82 m, max 2.45 m |

The signature in the auto error was acceleration lag: error grew to ~1 m while the trajectory
demanded speed and collapsed to ~0.1 m as soon as it slowed — the robot could not accelerate as
fast as paths planned at 7.0 m/s² required.

## What is not measurable here

Peak achieved acceleration. Differentiating the measured chassis speed gives a p99 above the
friction limit, so the tail is sensor and kinematics noise rather than motion. Getting that number
needs a controlled test: full stick from a standstill, straight line, fresh battery, on carpet.
