---
name: External Tools
description: Vendored external tools for log analysis (ClaudeScope) and simulation/WPILOG workflows (wpilib-agent-tools). Use these instead of writing custom analysis scripts.
---

# External Tools

This repo vendors two community tools under `tools/`. Use them for any log analysis, simulation, or NT4 task before writing custom code.

| Tool | Path | Purpose |
|---|---|---|
| ClaudeScope | [tools/ClaudeScope/](../../tools/ClaudeScope/) | Read `.wpilog` files + query live NT4; gives tuning recommendations |
| wpilib-agent-tools | [tools/wpilib-agent-tools/](../../tools/wpilib-agent-tools/) | Python CLI for sim sandboxes, WPILOG analysis, graphs, math/stats |

---

## ClaudeScope (`/scope`, `/simulate`)

A Claude Code plugin + Go CLI. The plugin provides two skills the agent invokes automatically.

### Install (one-time, per machine)

**Plugin (skills):**
```
/plugin marketplace add rylero/TheFRCSuite
/plugin install thefrc-suite@rylero/TheFRCSuite
```

**Binary (CLI):** Download from [github.com/rylero/TheFRCSuite/releases](https://github.com/rylero/TheFRCSuite/releases) and put on PATH:

| Platform | Binary |
|---|---|
| macOS (Apple Silicon) | `ClaudeScope-darwin-arm64` → rename to `ClaudeScope` |
| macOS (Intel) | `ClaudeScope-darwin-amd64` → rename to `ClaudeScope` |
| Linux | `ClaudeScope-linux-amd64` → rename to `ClaudeScope` |
| Windows | `ClaudeScope-windows-amd64.exe` → rename to `ClaudeScope.exe` |

Verify: `ClaudeScope version`

### Usage

The plugin defines two slash commands the user types in Claude Code:

- `/scope <question or file path>` — analyzes a `.wpilog` or live NT4 data; the agent uses ClaudeScope CLI to query, then synthesizes findings
- `/simulate <goal>` — launches headless sim, connects ClaudeScope, runs a goal-driven investigation autonomously

Example:
```
/scope analyze akit_26-03-21_15-19-00_incol_q4.wpilog and tell me if the flywheel was over- or under-tuned
```

### Workflow (from ClaudeScope SKILL.md)

```
1. Load the log (or connect to NT) → get session_id
2. Run queries using --session <id>
3. Disconnect when done
```

Timestamps are microseconds since log start. Negative start/end values are offsets from end of log.

Reference: [tools/ClaudeScope/skills/scope/SKILL.md](../../tools/ClaudeScope/skills/scope/SKILL.md)

---

## wpilib-agent-tools

A Python CLI for closed-loop sim experiments, WPILOG inspection, and NT4 recording. Bounded output (no context dumps from large logs).

### Install (one-time, per machine)

```bash
cd tools/wpilib-agent-tools
./scripts/install_all.sh --workspace ../.. --harnesses claude
```

What this does:
- Creates a `.venv` with `robotpy-wpiutil`, `pyntcore`, `numpy`, `matplotlib`, `sympy`
- Adds `.wpilib-agent-tools/run_cli.sh` shim at the repo root
- Adds a managed block to [CLAUDE.md](../../CLAUDE.md) and a slash command in [.claude/commands/](../../.claude/commands/)

> Heads-up: this modifies CLAUDE.md. Review the diff before committing — our hand-written safety hierarchy must stay intact.

If you skip the installer, you can still call the CLI directly:
```bash
cd tools/wpilib-agent-tools
python3 -m venv .venv && source .venv/bin/activate
pip install -e agent/
wpilib-agent-tools --help
```

### Capabilities

For `.wpilog` analysis:
- Graphing/visualization via Matplotlib
- NT4 key listing/querying
- Math helpers: derivatives, integrals, min/max, averages, std dev, RMS, thresholds, settling metrics
- **Output gating** — JSON modes, summaries, limits to prevent dumping huge logs into context

For simulation iteration:
- Auto-managed sandboxes (bounded changes outside the real workspace)
- Switch repo into sim mode → run → record NT4 → analyze → modify → repeat
- Best with AdvantageKit-style repos (which we are)

### Typical Workflow

```
1. Create sandbox
2. Apply bounded change
3. Run sim, record NT4
4. Inspect evidence
5. Review patch
6. Apply only what survives review
```

The agent shouldn't blindly edit robot code. The CLI makes verification faster and more inspectable. Patches still go through the [safety hierarchy](../../CLAUDE.md).

### Example Prompts

> Using wpilib-agent-tools, analyze akit_26-03-21_15-19-00_incol_q4.wpilog and tell me which subsystems are not meeting commanded setpoints.

> Using wpilib-agent-tools, create a sandbox and diagnose why the shoot sequence sometimes fires before alignment.

---

## When to Use Which

| Task | Tool |
|---|---|
| "Why was the flywheel oscillating in match Q4?" | ClaudeScope `/scope` |
| "Replay the log and check if my fix changed alignment timing" | AdvantageKit replay mode (see [replay-testing.md](replay-testing.md)) |
| "Run sim with this change and tell me if it brownouts" | wpilib-agent-tools sandbox |
| "Extract every BatteryLogger key and graph total current" | wpilib-agent-tools graph CLI |
| "Live query NT4 while the robot is running on the bench" | ClaudeScope NT mode |

Don't write custom Python WPILOG parsers — these tools already do it correctly.
