# Intake Roller Bench Tune — Velocity Loop

Procedure for validating and finishing the intake roller's closed-loop velocity
tune on the real robot. The gains in `intakeRollerConstants.PID` are a
**starting point derived from IRI match logs, not a validated tune** — this
procedure is what turns them into one.

**Why not simulation:** `intakeRollerIOSim` uses a frictionless `DCMotorSim`, so
it holds any speed on ~0 A while the real roller needs ~9 A against hopper drag.
The KS/KV terms that decide whether the roller holds its setpoint are exactly
what sim cannot model, and `ROLLER_MOI = 0.001` is an approximation besides.
This has to happen on hardware.

---

## Before you start

| Item | Value |
|---|---|
| Setpoint | 3000 RPM (`CompConstants.Velocities.intakeRollerVelocity`) |
| Control mode | `MotionMagicVelocityTorqueCurrentFOC`, gains in **amps** |
| Starting gains | KS 1.5, KV 0.18, KP 2.0, KI 0, KD 0 |
| Stator limit | 60 A (`INTAKE_ROLLER_MAIN_STATOR_AMP`) |
| Supply limit | 40 A, lower limit 35 A after 1 s |
| Motion Magic accel | 100 rot/s² → ~0.5 s minimum ramp to 50 rot/s |

Safety:

- Robot on blocks or the intake clear of hands — the roller is a two-Kraken
  mechanism with no soft limits.
- `TUNING_MODE` and `DEMO_MODE` false unless you are deliberately using them.
- Have someone on the disable button for the first spin-up.

---

## Signals to watch in AdvantageScope

Plot these four together on one graph:

```
/Intake Roller/IntakeRollerVelocity            <- rad/s, NOT RPM (see note)
/Intake Roller/RollerClosedLoopReference
/Intake Roller/RollerClosedLoopError
/Intake Roller/IntakeRollerStatorCurrent
```

Add `/Intake Roller/IntakeRollerVoltage` and `/SystemStats/BatteryVoltage` on a
second axis.

> **Unit note:** AdvantageKit serializes `Measure`-typed fields in SI base units.
> Velocity channels are **rad/s**, not RPM — 3000 RPM reads as **314.2**.
> Temperature channels are **Kelvin** — 307 is 34 °C, not a fault.
> Multiply rad/s by 9.549 to get RPM.

Target: **3000 RPM = 314.2 rad/s = 50.0 rot/s.**

---

## Step 1 — Free-spin baseline

Enable with nothing in the hopper. Let the roller reach steady state.

Record:

- Steady-state velocity (rad/s → RPM)
- Steady-state stator current
- Applied voltage

**Expect:** roughly 3000 RPM at well under 9 A, since there is no load. If it
sits far below setpoint even unloaded, KV is too low or KP is too weak — go to
Step 3 before adding load.

**Reference:** the old 12 V open-loop command free-spun at ~3200 RPM (p50 3186,
max 3429 across the IRI logs). If closed loop cannot reach 3000 unloaded,
something is wrong beyond gains — check that the gains actually applied
(`tryUntilOk` is not used in this IO) and that the follower is not fighting.

---

## Step 2 — Loaded steady state

Load the hopper to a realistic match state and hold the roller at setpoint.

Record steady-state velocity and stator current.

**Expect:** ~8–12 A, matching the IRI measurements at 2900–3100 RPM
(p25 7.6 / p50 8.8 / p75 12.3 A).

**Read the droop** — steady-state error in RPM:

| Droop | Meaning | Action |
|---|---|---|
| < 50 RPM | Feedforward is right | Done, go to Step 4 |
| 50–200 RPM low | KV slightly low | Raise KV by `droop_RPM / 9.549 / 50` A per rot/s |
| > 200 RPM low | KV materially low | Recompute: KV = (observed sustain current − KS) / 50 |
| Overshoots setpoint | KV too high | Lower KV proportionally |

KV is the term that fixes droop. Reach for KP only if droop is already small and
you want faster disturbance rejection.

---

## Step 3 — KP for disturbance rejection

With KV set, test the response to a real disturbance: drop a game piece in, or
briefly load the roller by hand against a tool (**not** by hand directly).

Watch `RollerClosedLoopError` recover.

- **Sluggish recovery, error lingers** → raise KP in steps of 1.0
- **Recovers but rings / oscillates** → back off KP ~30%, then add KD in steps of
  0.05 only if ringing persists
- **Current slamming into the 60 A stator limit on small disturbances** → KP is
  too high; the limit is doing your damping for you, which is not a tune

Stop when recovery is inside ~0.3 s with no visible ringing.

---

## Step 4 — Only if droop persists

If steady-state error is still non-zero after KV and KP are right, add KI
**reluctantly**, starting at 0.05, and watch for windup during the periods the
roller is held at 0 V by a shoot sequence. Integral windup on a mechanism that
gets commanded to zero mid-match is a real failure mode here — the roller is
held at 0 V during every hub and pass shot.

Prefer living with 30–50 RPM of droop over adding KI.

---

## Step 5 — Confirm the duty cycle still works

The roller default is always-on; the shoot/pass/tower sequences override it with
agitate voltage. Velocity control must not break those handoffs.

Check, in a full teleop sequence:

1. Idle → roller holds 3000 RPM
2. Shoot pressed → roller drops to 0 V (`holdRollerStopped` / `setVoltageAfterWait`)
3. Feeders release → roller goes to agitate voltage (3 V)
4. Shoot released → roller returns to 3000 RPM closed loop

Watch for a current spike at step 4 — re-engaging closed loop from a dead stop
commands full Motion Magic ramp. If the spike hits the stator limit, that is
expected on the ~0.5 s ramp and is bounded by the 60 A limit.

---

## Record the result

Log the final gains and the numbers behind them here:

| Gain | Start | Final | Evidence |
|---|---|---|---|
| KS | 1.5 | | |
| KV | 0.18 | | steady-state current at 3000 RPM = ___ A |
| KP | 2.0 | | recovery time = ___ s |
| KI | 0 | | |
| KD | 0 | | |

Free-spin: ____ RPM at ____ A
Loaded: ____ RPM at ____ A, droop ____ RPM

Then update `intakeRollerConstants.PID` and note in the commit that the gains are
now hardware-validated rather than log-derived.
