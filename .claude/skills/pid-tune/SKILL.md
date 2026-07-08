---
name: pid-tune
description: Run the PID/feedforward tuning loop for a mechanism — change gains, run simulation, read the resulting logs, evaluate, and iterate until criteria are met. Use when tuning any closed-loop mechanism (flywheel, hood, pivot, drive path-following). Sim-first; real-robot gain changes always require explicit user confirmation.
---

# PID Tuning Loop

You are running the tuning loop a human tuner would run — change gains, run,
read the response, adjust — but in simulation, without burning robot time.

**Hard rule:** gains applied to the real robot are a Hard Stop item. Everything
in this skill happens in simulation or in a sim-based test. The final output is
a *proposed* gain set with evidence; the user confirms before anything is
deployed. Never present sim-validated gains as field-validated.

## Step 1 — Define the target

Get from the user (ask if missing):

1. **Mechanism** — which subsystem / controller (e.g., flywheel velocity, hood
   position, PathPlanner translation PID)
2. **Success criteria, with numbers** — e.g., "settle within ±50 RPM in
   ≤ 1.0 s, overshoot < 5%", or "path cross-track error < 5 cm"
3. **Test motion** — what input to drive: step to a setpoint, a specific
   trajectory, a sweep of setpoints
4. **Current gains and where they live** — find them yourself if not given
   (`[Subsystem]Constants`, IO implementation, or `COMP_TunerConstants` for
   swerve — the last is generated and must not be edited here)

If no numeric criteria are given, propose reasonable ones and get agreement
before tuning — "looks smooth" is not a criterion.

## Step 2 — Build the harness

Prefer a **JUnit simulation sweep test** over interactive sim — it's
repeatable, automatic, and becomes a permanent asset.
`src/test/java/.../PathFollowingGainSweepTest` is the house example: it swept
PathPlanner gains and found the retune values without touching the robot.

The harness must:
- Drive the mechanism's `IOSim` (or the physics sim) through the test motion
- Record setpoint vs. measurement over time
- Compute the metrics: rise time, settling time, overshoot %, steady-state
  error, oscillation (count zero-crossings of the error after first reach)

If the mechanism's `IOSim` is stubs with no physics, say so — tuning against
stubs is meaningless. Offer to add a simple physics model (e.g., WPILib
`FlywheelSim` / `SingleJointedArmSim`) as a separate reviewed change first.

## Step 3 — Iterate

Run the loop. For each iteration, log: gains tried → metrics observed → what
you changed and why. Standard search order:

1. **Feedforward first** (velocity mechanisms): get Kv/Ks so open-loop tracks
   roughly right; PID then only corrects error.
2. **Kp up** until the response meets rise time or begins to oscillate; back
   off ~30% from the oscillation point.
3. **Kd** to damp overshoot if needed. **Ki last and reluctantly** — only for
   persistent steady-state error, and watch for windup.
4. Sweep, don't guess: when the landscape is unclear, run a parameterized
   sweep over a grid and report the frontier of gains meeting criteria.

Stop when criteria are met, or after a reasonable number of iterations —
if criteria can't be met, report the best achieved and the limiting factor
(actuator limits, sim model fidelity, conflicting criteria).

## Step 4 — Report and hand off

Deliver:

1. **Proposed gains** — old → new, per constant, with file/line locations
2. **Evidence** — the metrics table across iterations; plots or logged data
   the user can open in AdvantageScope if applicable
3. **Behavioral consequence** — in plain words: "stiffer response, settles
   0.4 s faster, ~3% overshoot vs none before"
4. **Failure mode if wrong** — per `.claude/rules/00-safety.md`
5. **The sweep test committed** as a regression asset, so next season's
   retune starts from a working harness

Then **ask for explicit confirmation** before writing the new gains into the
constants (High risk per the rules). After confirmation, apply via the
`tune-constants` discipline: change only the listed constants, compile, and
state the exact lines changed.

**Always note:** sim-validated only. First hardware test at reduced speed /
low voltage, at drive practice, before trusting the gains in a match.
