# AI-Assisted Robot Development Playbook

**Guerin Robotics — How We Program, Now and in Future Seasons**

*A living document. Update it every postseason. Share it with anyone who asks "why do you let AI write your code?"*

---

## Executive Summary (read this page if you read nothing else)

Our team programs robots the way modern engineering organizations build products: **humans decide what the robot should do and why; AI writes the implementation; humans review, simulate, and verify before anything touches the robot.**

This is not a shortcut and it is not autopilot. It is a top-down engineering process — the same one used in aerospace and automotive: leadership writes specifications, implementation is delegated, and everything passes through review gates before it ships. The difference is that our "implementation team" is Claude, and it works in minutes instead of weeks.

**Why this matters for us specifically:**

- **Time is our scarcest resource.** An FRC season gives us roughly six weeks to build a competitive robot. Every hour a student spends hand-typing boilerplate subsystem code is an hour not spent on drive practice, strategy, mechanical iteration, or scouting. AI-assisted development compresses days of coding into hours — and gives those hours back to the things that actually win matches.
- **We already know top-down engineering works — we lived it this year.** Our design side adopted the top-down methodology taught at [frcdesign.org](https://frcdesign.org/) — master layout first, subsystems inherit from it — and it made us dramatically better, fast. This playbook is the identical move applied to software: spec first, implementation derived from it, verification gates before anything ships. Software is the last place we're still building bottom-up.
- **It is safer than unsupervised hand-written code, not less safe.** Our AI operates under a written governance framework with hard stops (it cannot change CAN IDs, PID gains, encoder offsets, or safety timeouts without explicit human confirmation), mandatory build verification, and human review of every change. Most student code has none of that.
- **This is where the industry already is.** Anthropic, Google, Microsoft, and virtually every serious software organization now develop this way. Students who learn to specify, direct, review, and verify AI-generated systems are learning the actual job of a modern engineer. Students who only learn to hand-type Java are training for a version of the job that is disappearing.

**The ask:** adopt this playbook as our standard programming process, teach it to every new student, and improve it every season. This is not a plan for one good year — it is the operating system of a program that stays at the top as students come and go, because the rules, templates, and documentation compound instead of graduating. Teams that adopt this will out-iterate teams that don't.

---

## 1. The Core Idea: Top-Down, Specification-Driven Engineering

Traditional student programming is bottom-up: open the editor, start typing, debug until it works. It's slow, the quality depends entirely on who typed it, and knowledge leaves when that student graduates.

We've already seen what happens when you flip this. This year our design side adopted the top-down workflow taught at [frcdesign.org](https://frcdesign.org/) — the community design handbook used by top FRC teams: define the robot's architecture in a master layout sketch first, then let every subsystem inherit from it instead of CADing parts in isolation and hoping they fit. It transformed how we build — fewer integration surprises, faster iteration, a much better robot. Nobody on this team argues with top-down design anymore, and nobody calls it cheating or says it takes the fun out of engineering.

**This playbook is the exact same move, applied to software.** The parallel is not a metaphor — it's the identical discipline, step for step:

| Top-down CAD (frcdesign.org) | Top-down software (this playbook) |
|---|---|
| Master layout sketch defines the robot | Written spec defines the behavior |
| Subsystems inherit from the layout | Code is generated from the spec |
| Interference checks in the assembly | Build checks + simulation before deploy |
| Design reviews before manufacturing | Human review of every diff |
| Part libraries reused every season | Templates, rules & skills reused every season |

We already proved on this team, this year, that flipping from bottom-up to top-down makes us dramatically better. Software is simply the last place we're still building bottom-up. Fixing that is the same win we already banked on the design side.

The process has five layers, and **humans own four of them:**

```
┌─────────────────────────────────────────────────────────┐
│ 1. STRATEGY (humans)                                    │
│    What does the robot need to do to win matches?       │
├─────────────────────────────────────────────────────────┤
│ 2. SPECIFICATION (humans)                               │
│    Written spec: behaviors, constraints, tolerances,    │
│    failure modes, what "done" means                     │
├─────────────────────────────────────────────────────────┤
│ 3. IMPLEMENTATION (Claude)                              │
│    Code written to the spec, under governance rules,    │
│    following our architecture                           │
├─────────────────────────────────────────────────────────┤
│ 4. REVIEW & VERIFICATION (humans + tools)               │
│    Human code review, compile checks, simulation,       │
│    log replay — nothing skips the gates                 │
├─────────────────────────────────────────────────────────┤
│ 5. ROBOT TESTING (humans)                               │
│    Real hardware, drive practice, match validation      │
└─────────────────────────────────────────────────────────┘
```

The skill we cultivate is **layer 2**: writing specifications precise enough that the implementation is almost mechanical. That is a harder and more valuable skill than typing code. A student who can write "the shooter must reach setpoint within 1.5 seconds, and if alignment isn't achieved within the remaining budget, fire anyway — a robot that hangs mid-sequence scores zero points" understands the system. A student who can type a `PIDController` constructor from memory understands syntax.

**We direct. Claude types. We verify.** That's the whole model.

And "Claude types" undersells it. Claude isn't only the layer-3 implementer we visit once — it's an **agent of the team we direct across the whole process.** In **plan mode** it helps pressure-test strategy and sharpen a spec before a line of code is written; it drafts the implementation; it explains and helps review the resulting diff; it debugs from logs. Humans still *own* four of the five layers — every decision is ours — but the AI is a working member of the team we steer at every step, not a code vending machine.

### What a spec looks like

A spec is not an essay. It's a structured request:

- **Behavior:** what should happen, in sequence, with numbers ("spin up flywheel, wait for alignment up to 0.9s, then feed")
- **Constraints:** what must not change ("do not touch the drive subsystem; timeout constants live in `CompConstants.Waits`")
- **Interfaces:** what it connects to ("reads pose from `RobotState`, triggered by the operator's right bumper")
- **Verification:** how we'll know it works ("must show correct command activation in sim before deploy")

We keep reusable spec templates in `.claude/prompts/` — add a subsystem, tune constants, build an auto, debug a match log, review a change. Filling one out takes ten minutes. The implementation comes back in minutes more.

---

## 2. The Case: Time, Capability, and Competitiveness

### Time

The math is blunt. A traditional FRC software timeline looks like: weeks 1–3 writing subsystems, weeks 4–5 getting autonomous barely working, week 6 panic. Software is chronically the bottleneck — the mechanical robot sits idle waiting for code.

With this process, subsystem scaffolding takes hours, not weeks. That moves the bottleneck off software entirely. The season's schedule becomes: robot code ready when the robot is, then **weeks of drive practice and autonomous refinement** — which is where matches are actually won. Ask any veteran team what they'd trade for three extra weeks of drive practice.

### Capability — what a team working this way can do

These are the kinds of work AI-assisted engineering delivers today. This is the ceiling we'd be raising:

| Capability | Typical team | With this process |
|---|---|---|
| Scaffold every subsystem from the spec while the robot is still in CAD — code-complete in simulation before the robot exists | Weeks 1–3 of the season | Week 1 |
| Convert an entire autonomous library between path-planning frameworks, physics config and all | Days to weeks of tedious, error-prone manual conversion | One session |
| Root-cause a vision or odometry bug by replaying match logs deterministically on a laptop | Often never solved — teams live with "the robot teleports sometimes" | One debugging session |
| Extract real drivetrain dynamics from match telemetry and tune auto-aim with proper feedforward — actual system identification | Most teams guess at gains | Measured, sim-validated |
| Build ambitious features like shoot-on-the-move that veteran teams spend seasons chasing | Out of reach | On the table |
| Run a full-codebase review that catches latent bugs — wrong units, missing limits — before they cost a match | Rarely done at all | Routine |
| Debrief every match from its log and fix anomalies before the next one | "It felt weird out there" | Data by next match |

Notice what these have in common: the AI doesn't just type faster — it brings **depth** (system identification, log-replay debugging, whole-codebase review) that a typical student team simply doesn't have access to at any price. It's like having a professional controls engineer and a senior code reviewer on call, 24/7, during build season.

### Institutional memory

Student teams lose their best programmer every year to graduation. Our process captures knowledge in files that persist: the governance rules, the architecture docs, the spec templates, a knowledge graph of the entire codebase, and this playbook. A new student in a future season inherits not just code, but the *documented reasoning* behind it — and an AI that has read all of it and can answer questions about any part of the robot. The bus factor of a traditional team is one student. Ours is zero.

---

## 3. Answering the Concerns — Honestly

These objections deserve real answers, not dismissal. Here they are.

### "The code won't be accurate or reliable."

This is the most important concern, and the answer is: **reliability comes from process, not from who typed the code.** Hand-written student code is not reliable by default either — every team has stories of the auto that drove into a wall because of a sign error at midnight.

Our reliability comes from gates that apply to *all* code regardless of author:

1. **Written governance.** Claude operates under a rules file (`CLAUDE.md` plus safety rules) that it reads before every session. It is *prohibited* from touching CAN IDs, encoder offsets, PID gains, motor inversions, current limits, or safety timeouts without explicit human confirmation. It must classify every change by risk level and state the behavioral consequence and failure mode of risky changes before making them.
2. **Everything compiles and builds** before it's delivered. Automated formatting and build checks are mandatory steps, not suggestions.
3. **Human review of every diff.** A human reads and approves every change. The AI is required to explain what changed and why in plain language, which makes review *easier* than reviewing a teammate's uncommented 2 a.m. code.
4. **Simulation before hardware.** Command logic and state changes get verified in simulation with full logging before deploy.
5. **Log replay.** Our architecture (AdvantageKit) records every input; any match bug can be replayed deterministically on a laptop. When something goes wrong, we don't guess — we reproduce it exactly and fix it.

Compare that to the traditional baseline and ask which process you'd trust on a 120-pound robot.

One more honest point: yes, AI makes mistakes. So do students and mentors. The question was never "is the author infallible" — it's "does the process catch mistakes before the match." Ours does, by design.

### "Students won't learn anything. This takes the fun out of it."

This assumes the valuable skill is typing code. It isn't — and it hasn't been for a while.

What students learn in this process:

- **Systems thinking** — you cannot write a good spec without understanding how the flywheel, hood, transport, vision, and drivetrain interact. Spec-writing forces deeper understanding than copy-pasting a subsystem from last year ever did.
- **Reading and reviewing code** — students review every AI-generated diff. Code *reading* is a rarer and more valuable skill than code writing, and reviewing well-structured, explained code is one of the fastest ways to learn what good code looks like. The AI will explain any line, at any depth, with infinite patience. It is the best programming tutor a student has ever had access to.
- **Verification engineering** — designing the test that proves it works, running the sim, reading the logs.
- **Directing AI** — which, bluntly, is the defining skill of the next twenty years of engineering careers. FIRST's own mission is preparing students for real STEM careers. Real STEM careers now look like this.

And the fun: ask a student whether the fun of robotics is typing `private final TalonFX motor;` for the fortieth time — or watching a shoot-on-the-move feature they specified actually work at drive practice three days after they thought of it. AI removes the *drudgery*, not the engineering. The ideas, the strategy, the "what if the robot could..." — that all stays human, and there's dramatically more time for it.

**Also, nobody is banned from writing code.** A student who wants to hand-write a subsystem to learn is welcome to — and can have the AI review it, explain it, and pair with them. The policy is about how the *team ships*, not about forbidding anyone from learning however they want.

### "It's cheating / against the spirit of FIRST."

FRC rules permit any development tools, including AI assistance — the same as teams using vendor libraries, CAD generative design, or example code from Chief Delphi (which every team does). The code is team-owned, team-reviewed, team-understood, and team-maintained. What FIRST actually celebrates is *Gracious Professionalism* and real-world engineering practice — and this **is** real-world engineering practice, as of right now, at essentially every major software company.

There's also a leadership angle: a first-year team demonstrating a disciplined, documented, safety-governed AI development process is exactly the kind of innovation that awards judges and other teams take notice of. This playbook itself is shareable — helping other teams adopt this well *is* the spirit of FIRST.

### "We'll become dependent on it and helpless without it."

We are "dependent" on WPILib, vendor motor libraries, CAD software, and calculators too. The relevant question is whether the team *understands its robot* — and our process forces more understanding, not less, because every change requires a human-written spec going in and a human review coming out. The knowledge lives in our documented architecture, our rules files, and our students' heads. If AI vanished tomorrow, we'd have an exceptionally well-documented codebase and students who deeply understand its structure — better positioned than most teams, not worse.

### "What if it does something dangerous to the robot?"

This is exactly why the governance framework exists, and it's worth reading (`CLAUDE.md` and `.claude/rules/` in the repository). The hard-stop list was written from a catalog of real failure modes: wrong CAN ID controls the wrong motor, wrong inversion makes swerve fight itself, missing timeout hangs the robot mid-match. The AI is required to refuse those changes without explicit confirmation, escalate anything uncertain to a higher risk tier, and state failure modes out loud before high-risk edits. **No mentor has ever imposed this level of discipline on student-written code.** The AI-governed process is the most safety-conscious programming process this team could run.

### "AI code is unmaintainable spaghetti."

Not under this playbook, because the governance rules enforce our architecture: hardware isolated behind IO interfaces, a single state singleton, named command factories, one logical change per commit, no opportunistic refactors. The AI follows written rules more consistently than humans do — it never gets tired at 11 p.m. and hard-codes a CAN ID "just for now." Spaghetti comes from having no standards; this process is *made of* standards.

---

## 4. The Guardrails (why you can trust this)

For skeptics, this section is the whole ballgame. Our AI does not free-wheel. Everything runs inside a written, version-controlled governance framework that lives in the repository itself:

- **`CLAUDE.md`** — the constitution. Identity ("preserve working behavior; skepticism about changes is correct"), hard stops, and a five-tier risk classification (Safe → Low → Medium → High → Blocked) with escalating requirements. When uncertain, the AI must treat a change as one tier riskier.
- **`.claude/rules/00-safety.md`** — the failure-mode catalog and the absolute hard stops (CAN config, control gains, current limits, safety interlocks, logging integrity).
- **`.claude/rules/01–05`** — architecture rules, hardware/CAN rules, command patterns, mandatory build verification, git discipline. These encode *why* past decisions were made so they don't get silently reversed.
- **Review checklist and change classification docs** — so human reviewers know exactly what to look for at each risk tier.
- **Tested against the spec.** Automated code review, security review, and end-to-end verification run against the written spec before a human ever looks — more testing and verification than this team ever did by hand.
- **One logical change per response; small diffs; no scope creep.** Enforced in the rules.

Every future season, the first postseason task is updating these files with what we learned. The governance framework is itself a year-over-year asset — arguably our most valuable one.

---

## 5. The Toolkit

### What we run today

| Tool | What it does for us |
|---|---|
| **Claude Code** | The core agent: reads the whole codebase, writes code, runs builds, analyzes logs, executes multi-step engineering tasks under our governance rules |
| **Governance framework** (`CLAUDE.md` + rules) | Safety rails, architecture enforcement, risk classification — see §4 |
| **Prompt templates** (`.claude/prompts/`) | Reusable specs: add-subsystem, new-auto, tune-constants, review-change, write-test. Standardizes how we direct the AI |
| **Team skills** (`/debug-match-log`, `/pid-tune`) | The two most-repeated tasks, fully automated: match-log root-cause analysis and the sim-based PID tuning loop |
| **graphify knowledge graph** | A queryable map of the entire codebase — components, relationships, architecture. Lets the AI (and students) answer "what talks to what" instantly instead of grepping |
| **Code review skills** (`/code-review`, `/security-review`, `/simplify`) | On-demand automated review passes — including multi-agent deep review of a whole branch before merge |
| **Verification skill** (`/verify`) | Drives a change end-to-end (build, sim) instead of assuming it works |
| **Subagents** | Parallel workers for big tasks — one can explore the codebase while another plans an implementation |
| **AdvantageKit + log replay** | Not an AI tool per se, but the foundation that lets the AI debug matches deterministically from logs |

### What we can add next

- **Custom team skills** — e.g., a `/new-auto` skill that takes a path description and produces a Choreo trajectory plus registered named commands; a `/match-debrief` skill that ingests a match log and produces a findings report automatically after every match.
- **CI integration** — every pull request automatically built, formatted, and AI-reviewed on GitHub before a human ever looks at it.
- **Scheduled agents** — nightly automated codebase health review during build season; automatic post-practice log analysis.
- **MCP integrations** — connect the AI to our Slack (post build results to the team channel), scouting data, or match schedules.
- **Vision/CAD assistance** — AI-assisted analysis of camera calibration, field layouts, and mechanism geometry from images.
- **A student onboarding skill** — new programmer joins, runs one command, gets a guided interactive tour of the robot code.

The toolkit compounds. Every skill we build this year is inherited by every future season.

---

## 6. The Year-Over-Year Season Layout

This is the repeatable process. Each season, run this loop:

### Preseason (fall)
1. **Update the governance files** with last season's lessons (new hard stops, new failure modes, architecture decisions).
2. **Train new students** on the process: how to write a spec, how to review a diff, how to run sim verification. Target: every programming student can independently spec-and-ship a low-risk change by kickoff.
3. **Refresh the toolkit** — update Claude Code, rebuild the knowledge graph, add the skills identified last postseason.
4. **Dry run**: rebuild a past-season subsystem from spec alone, as a training exercise.

### Kickoff → Week 1
1. Strategy team defines robot behaviors (human work — the most important week of the year).
2. Programming leads write the **robot software spec**: subsystem list, state machine, autonomous priorities, interfaces to hardware.
3. Claude scaffolds every subsystem from the spec + template while mechanical design is still in CAD. **Software is code-complete in simulation before the robot exists.**

### Build Season (weeks 2–6)
The loop, repeated per feature: **Spec → Implement (AI) → Review (human) → Simulate → Deploy → Drive-test.** One logical change at a time, every change through the gates. Autonomous routines developed and tuned in simulation continuously, not crammed into week 5.

### Competition
- Match logs pulled after every match; AI-assisted debrief finds anomalies before the next match.
- Fixes at competition follow the same gates, compressed — spec can be verbal, but review and build verification never skip. Revert discipline per the git rules.

### Postseason
1. Full AI-assisted codebase review to catch latent bugs while there's time to fix them calmly.
2. **Post-mortem into the governance files** — every mistake becomes a rule so it can't recur.
3. Update this playbook.
4. Archive the season's knowledge graph and docs as the starting context for next year.

---

## 7. How This Fits GRIDS

GRIDS is our own framework — **G**oals, **R**ecruitment, **I**ntegration, **D**evelopment, **S**uccession. This playbook isn't a side project bolted onto the team; it puts a spec-driven software engine behind every pillar.

| Pillar | How the playbook advances it |
|---|---|
| **Goals** — world-class robots, competing at the highest level | AI-assisted, spec-driven development is the largest legal time multiplier available to reach that level — and to keep reaching it, year after year. |
| **Recruitment** — inspire and attract the next generation | A disciplined, documented, safety-governed AI process is exactly what awards judges and prospective members notice. Sharing this playbook with other teams is the spirit of FIRST. |
| **Integration** — meaningful work from day one | A new member paired with a veteran can spec-and-ship a real low-risk change in their first week. The rules and templates make the codebase approachable instead of intimidating. |
| **Development** — grow students into engineers | The skill we cultivate is specifying systems — behaviors, numbers, constraints, what "done" means. That is real engineering judgment, and it transfers to CAD, electrical, and strategy. |
| **Succession** — leave the team stronger than we found it | Most teams reset every June when their best programmer graduates. Here the rules, templates, skills, and docs *are* the program — they compound, and each season starts where the last one ended. |

The through-line: this is not a tool one talented student uses. It's an institution the whole team owns, and it makes every pillar of GRIDS stronger the longer we run it.

---

## 8. What This Means for Us

Time is our biggest enemy. Every team gets the same six weeks; the teams that win are the ones that spend those weeks iterating on the robot instead of fighting their tools. AI-assisted development is the single largest time multiplier available to an FRC team today, it is fully legal, and almost nobody is using it with real discipline yet. That is a competitive window, and it will not stay open — in three years this will simply be how good teams work, the way swerve went from exotic to mandatory.

We can be early and disciplined, build the playbook, win with it, and teach it — or we can be late and copy someone else's version of this document.

And this is not about one season. Most teams reset every June when their best programmer graduates and the knowledge walks out the door. Under this playbook nothing resets: the governance rules, the spec templates, the skills, and the documentation *are* the program, and they compound. Top-down design already transformed how we build the robot — we watched it happen this year. This finishes the job on the software side, and it's how a first-year team becomes a top team that stays on top.

The framework is written. The tools are installed. The only missing piece is **us** — the whole team choosing to adopt it, learn it, and make it how we build. Full buy-in is the last component, and it's the one only we can supply.

---

*Maintained by the programming leads. Last updated July 2026. Questions and pushback welcome — this document gets stronger when it has to answer them, and so does the process.*
