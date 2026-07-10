# AI Development Handbook

**Guerin Robotics — How We Write Robot Code with AI**

*The companion to the [AI-Assisted Robot Development Playbook](ai-development-playbook.md).
The playbook is the **why**. This is the **how**. If you are a new programming
student — freshman or senior — this is the document you learn from. It is a
living document: we update it every season and every time something goes wrong
that a rule here could have prevented.*

> **A note on "Claude":** This handbook says "Claude" throughout because at the
> time of writing, Claude is the best AI model available for this work. That
> will change. Everything in this document — specs, context, review, testing,
> the two tracks — applies to whatever AI tool we use. If the team switches
> models someday, the process stays; only the name changes.

---

## Table of Contents

1. [The one idea behind everything](#1-the-one-idea)
2. [The AI knows the code, not the robot](#2-the-ai-knows-the-code-not-the-robot)
3. [Two tracks: Full Loop and Fast Path](#3-two-tracks)
4. [The Full Loop — building new code](#4-the-full-loop)
5. [The Fast Path — tweaks, tuning, and bug fixes](#5-the-fast-path)
6. [The season arc — spec, autos, tune](#6-the-season-arc)
7. [Giving Claude context](#7-giving-claude-context)
8. [Testing — Claude does it, you check it](#8-testing)
9. [Skills and prompts — the toolkit](#9-skills-and-prompts)
10. [The IO layer pattern](#10-the-io-layer-pattern)
11. [Cheat sheet](#11-cheat-sheet)

---

## 1. The One Idea

Software teams everywhere are converging on the same lifecycle for AI-assisted
development — the industry version is called the
[AI-SDLC](https://www.aisdlc.io/). Its core findings, translated to our team:

- **AI helps at every stage** — writing specs, generating code, reviewing
  diffs, generating tests, diagnosing failures, and analyzing logs after a
  match. Not just the typing part.
- **Guardrails come before speed.** We wrote our rules (`CLAUDE.md`,
  `.claude/rules/`) *first*, so Claude can move fast *safely*. Never remove a
  guardrail to go faster.
- **AI writes, humans decide.** Claude generates the code and runs the tests.
  A human reads every diff and approves every merge. The AI is the fast hands;
  you are the judgment.
- **Match the process to the risk.** A new subsystem and a one-line constant
  change do not deserve the same ceremony. That's why we have two tracks (§3).

One sentence to remember: **Claude writes and debugs the code; we describe the
robot, review the changes, and own the result.**

---

## 2. The AI Knows the Code, Not the Robot

This is the most important thing to understand before your first session.

Claude Code is powerful for the software side of the robot, but **it does not
automatically understand the physical robot. It only knows what we tell it.**
It sees the code on your computer — it did not see what the robot just did on
the field. It doesn't know the intake got bent in quarterfinals, that the new
battery sags, or that the flywheel makes a grinding noise above 3000 RPM.

Before asking Claude to write, debug, or improve robot code, give it clear
context about the real machine:

- **How the robot is built** — what each mechanism is, what it's for
- **What motors and sensors are on it** — types, gear ratios, what's plugged in where
- **What the robot is expected to do** — the behavior you want, with numbers
- **What actually happened** — what you saw with your eyes, not just "it's broken"

The better we explain the real robot, the better Claude writes code that
actually works on the field. Every AI session that goes sideways goes sideways
because the human assumed Claude knew something only the human knew.

§7 covers exactly how to supply this context.

---

## 3. Two Tracks

Not every change deserves the full process. The Full Loop exists because
**building the initial solution is the hard part** — new subsystems, new
sequences, new autos have to be designed, structured, and verified from
scratch. Once the robot's code exists and works, most changes are small:
add a button binding, adjust a PID gain, fix a bug, raise a current limit,
tweak a timeout. Those take the Fast Path.

| | **Full Loop** (§4) | **Fast Path** (§5) |
|---|---|---|
| **What** | New subsystems, new command sequences, new autos, structural changes | Constant changes, button bindings, small bug fixes, tuning tweaks |
| **Steps** | Spec → Plan → Build → Review → Test → PR → Merge | Ask → Change → Verify → Commit |
| **Plan mode?** | Yes | No |
| **Written spec?** | Yes | One clear sentence is enough |
| **Pull request?** | Always | Yes, but small PRs can bundle related tweaks |

Rule of thumb: **if Claude is creating something, use the Full Loop. If Claude
is adjusting something that already works, use the Fast Path.** When you're
not sure, use the Full Loop — the extra ceremony is cheap; a broken robot is
not.

Either track, the safety rules never turn off. Claude still asks before
touching gains, CAN IDs, offsets, inversions, current limits, or timeouts —
that's enforced by the rules, not by which track you picked.

---

## 4. The Full Loop

For building new code. Seven steps, ending in a pull request.

```
┌────────┐  ┌────────┐  ┌────────┐  ┌────────┐  ┌────────┐  ┌────────┐  ┌────────┐
│ 1 SPEC │─▶│ 2 PLAN │─▶│ 3 BUILD│─▶│ 4 REVIEW─▶│ 5 TEST │─▶│ 6 PR   │─▶│ 7 MERGE│
│ (you)  │  │(you+AI)│  │  (AI)  │  │ (you)  │  │  (AI)  │  │(you+AI)│  │ (team) │
└────────┘  └────────┘  └────────┘  └────────┘  └────────┘  └────────┘  └────────┘
```

### Step 1 — Spec: say what you want

The more specific the detail, the better the result. A good spec answers:

- **Behavior** — what should happen, in order, with numbers.
  *"Spin up the flywheel, wait up to 1.5 s for it to reach setpoint, then feed."*
- **Interfaces** — what it connects to.
  *"Reads pose from RobotState. Bound to operator right bumper."*
- **Done** — how you'll know it works.
  *"Command activates/deactivates correctly in sim; tests pass."*

**Constraints are optional.** The built-in rules (`CLAUDE.md` +
`.claude/rules/`) already tell Claude what must not change — hard stops, risk
tiers, architecture patterns. You only need to add a constraint when it's
specific to this task (*"don't touch the pivot while doing this"*).

Vague specs produce vague code. **"Make the shooter better" is not a spec.**
"The shooter overshoots by ~200 RPM when spinning up from idle; reduce
overshoot without slowing settling past 1.5 s" is a spec.

For recurring tasks, don't write the spec from scratch — use the templates in
`.claude/prompts/` (§9).

### Step 2 — Plan: plan mode before code

Start Claude in **plan mode** (`shift+tab` to cycle modes, or say "make a plan
first"). Claude reads the code and proposes an approach but **cannot edit
anything** until you approve.

Plan mode is also where you pressure-test your own spec. Ask: *"What's unclear
about this request? What could go wrong?"* It finds the holes in your spec
faster than the compiler will.

### Step 3 — Build: Claude implements, one logical change at a time

Claude writes the code under the governance rules — they load automatically
every session. What they guarantee:

- It won't touch CAN IDs, PID gains, encoder offsets, inversions, current
  limits, or safety timeouts without your explicit confirmation.
- It states the behavioral consequence of every Medium+ risk change.
- One logical change per response. If it starts "also cleaning up" things you
  didn't ask for — stop it.

**Your job during this step:** read what Claude *says*, not just the code.
When it says *"this means the robot fires 0.2 s sooner if alignment isn't
achieved"* — that sentence is the review. Decide if that's what you meant.

### Step 4 — Review: a human reads every diff

You are looking for (full list in `docs/review-checklist.md`):

1. **Did it change only what was asked?** Extra "improvements" get reverted.
2. **Do the numbers match the spec?** Timeouts, tolerances, setpoints, units.
3. **Hard-stop items untouched?** CAN IDs, gains, offsets, inversions, limits.
4. **Every `waitUntil` has a `.withTimeout()`.** A hung robot scores zero.
5. **Commands are named** and hardware stays behind the IO interface.

If you can't explain a line, ask Claude to explain it — that's half the point
of reviewing. **Never approve code you don't understand.**

### Step 5 — Test: Claude verifies, you spot-check

Claude runs the whole verification ladder itself — build, unit tests, sim —
and writes unit tests for the new logic automatically. Details in §8. Your
part: watch the change work in AdvantageScope, in sim, with your own eyes.

### Step 6 — Pull request

Every Full Loop change goes to `main` through a pull request:

- Work on a feature branch: `feature/...`, `fix/...`, `tune/...`
- Run `/code-review` on the diff before opening the PR — it catches wrong
  units, missing limits, and logic bugs for free.
- Claude drafts the PR description; you read it — it's part of what you're
  approving. The description says **why**, what the behavioral change is, and
  how it was verified.
- CI builds and format-checks every PR automatically. A red PR does not merge.

### Step 7 — Merge

Another team member (or a lead) reviews the PR before merge. Two humans have
now read the change: the author and the reviewer. That's the standard.

---

## 5. The Fast Path

For adjusting code that already works: constant changes, button bindings,
small bug fixes, current limits, timeouts, and tuning tweaks. AI writes code
fast and debugs well — the Fast Path gets out of its way.

```
ASK  ─▶  CHANGE  ─▶  VERIFY  ─▶  COMMIT (small PR)
```

1. **Ask** — one clear sentence with numbers.
   *"Change the flywheel idle speed from 500 to 800 RPM — we measured spinup
   taking too long from a dead stop."* No written spec, no plan mode.
2. **Change** — Claude makes the edit. If the change touches a High-risk item
   (gains, limits, timeouts), Claude will ask you to confirm and state the
   failure mode — answer it; that's the process working.
3. **Verify** — Claude still compiles, formats, and runs the tests. For a
   behavior change it also checks in sim. "It's just a constant" is not an
   excuse to skip verification — constants have types and consequences.
4. **Commit** — one logical change, message says why. Related small tweaks
   from one session (e.g., a tuning session's constant changes) can share a
   branch and go up as one small PR.

What makes the Fast Path safe isn't skipping steps — it's that the steps are
small enough for Claude to run them in minutes and for you to review the diff
in thirty seconds.

**Escalate to the Full Loop** if a "small fix" turns out to touch more than a
couple of files, changes a sequence's structure, or makes you say "while we're
in here…" — that's a new feature wearing a bug fix's clothes.

---

## 6. The Season Arc

The two tracks map onto how a season actually goes. Build season is Full Loop
territory; competition season is mostly Fast Path.

### Phase 1 — Initial robot spec (kickoff → robot exists)

Write the spec for the whole robot: every mechanism, every motor, every
sensor, every behavior. This is the biggest Full Loop work of the year, and
it's where thoroughness pays off most — you are building the initial solution
that everything else adjusts.

**Include values you haven't tested yet.** Gear ratios from CAD, setpoints
from napkin math, gains copied from last year, speeds from a game-piece video.
Educated guesses and vibed-out numbers are fine — **you have to start with
something**, and every guessed value is a one-line Fast Path fix later. Mark
them: *"UNTESTED — estimate."*

**Keep current limits low at first.** When testing brand-new code on a
brand-new robot, low current limits mean a code bug stalls a mechanism
harmlessly instead of breaking it. Raise limits deliberately, later, when the
mechanism is proven — never as a first resort when something seems weak.

### Phase 2 — Auto specs (robot drives → first competition)

Spec each autonomous routine the same way: starting pose, path, actions at
each point, time budget, what happens if a shot or pickup fails. Autos are
Full Loop — they're new sequences, and a bad auto can foul or score zero.
Verify every auto in sim (watch the pose trace on the field view) before it
ever runs on carpet.

### Phase 3 — Tuning and tweaking (forever)

Everything after the robot works is mostly Fast Path: PID tuning, threshold
adjustments, binding changes, bug fixes from match logs. The two skills built
for this phase — `/pid-tune` and `/debug-match-log` (§9) — exist because
tuning and log analysis were the two things we did most, all season.

---

## 7. Giving Claude Context

The highest-leverage skill in this document. Claude's output quality tracks
the quality of the context it gets.

### What Claude already knows (automatic)

| Source | What it provides |
|---|---|
| `CLAUDE.md` + `.claude/rules/` | Hard stops, risk tiers, architecture rules, build gates. Loads every session. |
| `graphify` knowledge graph | A queryable map of the codebase — "what talks to what" without grepping. |
| Persistent memory | Notes across sessions — ongoing tuning efforts, known issues, past decisions. |
| The code itself | Claude reads any file it needs. Don't paste code it can just open. |

### What YOU must supply (see §2 — the AI never sees the robot)

1. **What physically happened.** *"The robot oscillated left-right during
   auto-align at about 2 Hz"* beats *"auto-align is broken"* by a mile.
2. **What changed recently.** New battery? Mechanical repair? Different
   carpet? Claude sees the code history, not the pit.
3. **The log file.** For any on-robot problem, the AdvantageKit log is worth
   ten paragraphs. Run `/debug-match-log` with the log path and match time.
4. **Intent and priority.** *"20 minutes before our next match, I need the
   safest possible fix"* changes what Claude proposes. Say it.
5. **Hardware ground truth.** Which robot (COMP vs ALPHA), what's plugged in,
   what the mechanism looks like. A photo works — Claude reads images.

### Context anti-patterns

- **The mind-reader.** "Fix the intake." Which behavior? When? Instead of what?
- **The novel.** Backstory burying the request. Lead with the ask.
- **The stale session.** Hours across five topics — the vision debug bleeds
  into the flywheel tune. One task per session (`/clear`).
- **Screenshots of code.** Give file paths; Claude opens files itself.
- **Paraphrased errors.** Paste the actual build output, verbatim.

### The prompt pattern that covers 80% of requests

```
[WHAT]     Add jam detection to the intake roller.
[BEHAVIOR] If stator current exceeds 60 A for more than 250 ms while the
           intake command is active, stop the roller and log a warning.
[WHERE]    IntakeRoller subsystem. Threshold constants in HardwareConstants.
[VERIFY]   Show me it working in sim by faking a current spike in IOSim.
```

Four lines. Constraints only if this task needs one beyond the built-in rules.

---

## 8. Testing

**Claude does the testing and verification. You spot-check the result.**
"It compiles" is the floor, not the goal.

### The ladder Claude runs on every change

```bash
./gradlew compileJava        # 1. It compiles. Always.
./gradlew spotlessApply      # 2. Formatting (CI rejects unformatted code)
./gradlew build              # 3. Full build + all unit tests
./gradlew simulateJava       # 4. Simulation, for anything that moves or schedules
```

Claude runs all four itself before reporting a task complete, simulates the
change, tries values, and reports what it observed — you should never have to
ask "did you build it?"

### Unit tests are automatic

**Every piece of hardware-independent logic gets a unit test, and Claude
writes it as part of the change — you don't have to ask.** Calculators,
interpolation, zone math, alliance flipping, command sequencing, timeout
behavior. And **every bug that ever bit us gets a regression test**: the fix
isn't done until a test exists that fails on the old code and passes on the
new. That's how a bug can only cost us once, ever, across all future seasons.

Your job in testing is the part that needs game knowledge: the known-correct
cases. *"At 3.0 m the shot table says 2400 RPM and 34°"* comes from real
measurement — Claude turns it into a test. Review tests like code: a test
that can't fail is not a test (ask Claude: *"show me this test failing"*).

Tests live in `src/test/java`, never touch hardware, and run in CI on every
PR.

### Simulation — your spot-check

Open **AdvantageScope**, connect to the sim, and watch the thing you changed
actually happen:

- **Commands** schedule and end when they should, with the right names
  (this is why `.withName()` is mandatory — unnamed commands are invisible).
- **Setpoints vs. measurements** on the same plot — oscillation and overshoot
  are visible in sim before they're dangerous on carpet.
- **Autos** — full routine in sim; a path that clips a wall in sim clips a
  wall on the field.

For driver-facing changes, sim with a real controller plugged in is a
ten-minute test that has caught binding mistakes every single time.

### Log replay — the superpower

AdvantageKit records every input on the robot, so any match bug can be
**replayed deterministically on a laptop** — same inputs, same code path,
same bug, every time. Run `/debug-match-log` (§9): Claude reads the log,
correlates inputs, outputs, and command state around the event, and proposes
a root cause you can check against the replay.

This is how the vision pose-jump bug got root-caused — from log data, not
guessing. This capability is why `Logger.processInputs()` is a hard stop.

### What each level catches

| Level | Catches | Misses |
|---|---|---|
| Build | Type errors, formatting | Everything about behavior |
| Unit tests | Wrong math, wrong units, regressions | Timing, integration |
| Simulation | Command logic, sequencing, gross tuning, auto paths | Real friction, latency, batteries |
| Log replay | Real-world bugs, exactly as they happened | Nothing — but only after the fact |
| Drive practice | Everything else | Nothing — which is why it's last, not first |

**Hardware time is the scarcest resource we have.** Every bug caught in sim is
drive-practice time we get back.

---

## 9. Skills and Prompts

The toolkit stays **small and curated on purpose.** A library of thirty
templates nobody remembers is worse than eight that everyone knows. Before
adding a new skill or prompt: does an existing one cover it? Once a season,
the leads prune anything that hasn't been used.

### Skills (slash commands — Claude runs the whole task)

| Skill | What it does | When |
|---|---|---|
| `/debug-match-log` | Analyzes an AdvantageKit log: correlates signals around an anomaly, proposes root cause, sets up replay | Anything went wrong on the field |
| `/pid-tune` | Runs the tuning loop: change gains → run sim → read the logs → evaluate → iterate; proposes final values with evidence | Tuning any closed-loop mechanism |
| `/code-review` | Automated bug-hunting review of the current diff | Before every PR |
| `/simplify` | Cleanup pass — reuse and simplification, no bug hunting | After a feature works, before PR |
| `/verify` | Drives the change end-to-end (build + sim) instead of assuming | Nontrivial changes |
| `/graphify` | Rebuild/query the codebase knowledge graph | After merging big changes |

`/pid-tune` exists because PID tuning was the single task we repeated most.
It sweeps or steps gains **in simulation**, reads the resulting logs itself,
and iterates — the same loop a human tuner runs, without burning robot time.
Real-robot gain changes still require your explicit confirmation, always.

### Prompt templates (`.claude/prompts/` — you fill in brackets)

| Template | Use when |
|---|---|
| `add-subsystem.md` | Scaffolding a new mechanism (Full Loop) |
| `new-auto.md` | Creating or modifying an autonomous routine |
| `tune-constants.md` | Applying **measured** values safely (Fast Path) |
| `review-change.md` | Pre-merge safety review of a diff |
| `write-test.md` | Locking in a behavior or bug as a JUnit test |

**How to use one:** open the file, fill every `[BRACKET]`, delete lines that
don't apply, paste. Don't freestyle a task that has a template — the template
encodes every lesson from every time that task went wrong before.

**When you repeat a task that has no template, that's a signal.** Write the
template (fifteen minutes), or promote it to a skill if Claude can run the
whole thing itself. Then tell a lead so it gets into this document. This is
how the toolkit compounds year over year.

---

## 10. The IO Layer Pattern

Adding a subsystem is the biggest Full Loop task and has the strictest
structure. **Every mechanism follows this exact shape — no exceptions, no
creativity.** The shape is what makes log replay, simulation, and AI
generation all work.

```
subsystems/flywheel/
├── io/
│   ├── FlywheelIO.java        ← interface + @AutoLog inputs class
│   ├── FlywheelIOReal.java    ← TalonFX code lives HERE and only here
│   └── FlywheelIOSim.java     ← physics sim (or stubs), same interface
├── Flywheel.java              ← logic only, zero hardware imports
commands/
└── FlywheelCommands.java      ← static command factories, all .withName()'d
```

**The one rule that matters:** hardware objects (`TalonFX`, `CANcoder`,
`Pigeon2`…) exist ONLY inside `IOReal` classes. A motor import in a subsystem
file breaks log replay for the whole robot — and log replay is how we've
caught real match bugs.

```java
@Override
public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);   // NEVER remove this line
    // ... logic reads from `inputs`, never from hardware
}
```

### The recipe

1. Get the hardware facts from whoever wired it: motor type, CAN ID, bus,
   follower (and opposition), encoder, control mode.
2. Fill in `.claude/prompts/add-subsystem.md` and paste. Claude generates all
   five files from `template/`, adds the CAN ID to
   `HardwareConstants.CanIds`, wires real/sim in `RobotContainer`, compiles,
   and writes the unit tests.
3. Review: CAN ID matches the wiring sheet; correct bus; config via
   `PhoenixUtil.tryUntilOk(...)`; current limits set **low** (§6) and
   enabled; `IOSim` works; inputs appear in AdvantageScope.
4. First hardware test at low voltage before any closed-loop control.

> [!IMPORTANT]
> **Time cost: about 30 minutes including review.** Hand-writing the same five
> files correctly used to take a day — and the sim implementation usually
> didn't get written at all. This is the single clearest payoff of the whole
> approach: the AI makes the *right structure* cheaper than the shortcut.

### Changing an IO interface later

Adding a field to an `@AutoLog` inputs class changes the log schema:
**real + sim must both change in the same PR**, and the build regenerates the
`*AutoLogged` class (`./gradlew clean generateSources` if it gets confused).
Claude knows this rule — the reviewer checks both files changed.

---

## 11. Cheat Sheet

**Two tracks:**
- **Building new code?** Full Loop: Spec → Plan → Build → Review → Test → **PR** → Merge.
- **Adjusting working code?** Fast Path: Ask → Change → Verify → Commit (small PR).
- Not sure? Full Loop.

**The spec pattern:**
```
[WHAT]     one sentence
[BEHAVIOR] what happens, in order, with numbers
[WHERE]    files/subsystems; where constants live
[VERIFY]   how we'll prove it works
```
Constraints optional — the built-in rules cover the standard ones.

**Claude runs the verification itself:** compile → spotless → build + unit
tests → sim. It writes unit tests for new logic automatically, and a
regression test for every bug fix. Your job: spot-check in AdvantageScope and
supply the known-correct numbers.

**The season arc:** ① initial robot spec (guessed values OK — mark them
UNTESTED; keep current limits LOW) → ② auto specs (sim-verify every path) →
③ tune and tweak (Fast Path + `/pid-tune` + `/debug-match-log`).

**Skills:** `/debug-match-log` (field problems), `/pid-tune` (tuning loop),
`/code-review` (before every PR), `/verify`, `/simplify`, `/graphify`.
**Templates:** `.claude/prompts/` — add-subsystem, new-auto, tune-constants,
review-change, write-test. Fill brackets, don't freestyle.

**Never without explicit confirmation:** CAN IDs, PID/FF gains, encoder
offsets, inversions, current limits, safety timeouts,
`Logger.processInputs()`, `.auto` files.

**Always:** every `waitUntil` gets `.withTimeout()`. Every command gets
`.withName()`. Hardware only in `IOReal`. Every fixed bug gets a regression
test. Every Full Loop change gets a PR.

**Remember:** the AI knows the code, not the robot. Tell it what the robot
is, what it did, and what you want — with numbers.

**When in doubt:** treat it as one risk tier higher, use plan mode, and ask.

---

*Maintained by the programming leads alongside the playbook. This document
outlives any one AI model — update the tools, keep the process. Last updated
July 2026.*
