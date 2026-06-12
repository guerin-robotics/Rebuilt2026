# wpilib-agent-tools

`wpilib-agent-tools` is an experimental, repo-first toolkit for FRC teams exploring agentic coding workflows around WPILib simulation, NT4 recording, and WPILOG analysis.

The tooling is meant to help agents work more effectively with WPILib simulation, NT4 data, and `.wpilog` analysis. It gives models a focused set of CLI commands and markdown guidance for gathering evidence, checking behavior, and iterating on robot code without immediately mutating the real workspace.

It packages two related pieces:

- a Python CLI for simulation, log, graph, math, sandbox, and NT4 workflows in [agent/](agent/)
- shared workspace harness support for **Codex**, **Claude Code**, and **Cursor**

This is not a polished universal AI coding platform. It is a practical experiment for gathering better evidence during robot-code iteration before any patch is applied to a real workspace.

## At a Glance

| Area | What this repo provides |
| --- | --- |
| CLI workflows | WPILib sim, NT4 recording, WPILOG inspection, graphing, math checks, and sandbox utilities |
| Agent harnesses | One shared installer path for Codex, Claude Code, and Cursor workspace support |
| Validation loop | Bounded changes in a sandbox, followed by sim/log evidence and patch review |
| Distribution | GitHub-first source, install scripts, docs, tagged releases, and release artifacts |

## Quick Start

Clone the repo, then install the CLI plus workspace harness support into a robot project:

```bash
git clone https://github.com/edanliahovetsky/wpilib-agent-tools.git
cd wpilib-agent-tools
./scripts/install_all.sh --workspace /path/to/robot-repo --harnesses all
```

That command:

- bootstraps the local CLI into `./.venv`
- validates the Codex skill bundle structure
- runs smoke checks
- installs shared harness support into the target workspace
- creates a consistent workspace entry path for Codex, Claude Code, and Cursor

## Core Workflow

The useful loop is intentionally evidence-driven:

1. Make a bounded change in a sandbox.
2. Run simulation or log analysis.
3. Inspect concrete evidence from the run.
4. Review the resulting patch.
5. Apply only the changes that survive normal engineering review.

The goal is not to let a model blindly edit robot code. The goal is to make verification faster, more repeatable, and easier to inspect.

## Agent-Oriented Capabilities

At a high level, this toolkit enables agents to analyze robot behavior with purpose-built commands and to run closed-loop simulation experiments in isolated workspaces.

For `.wpilog` analysis, it supports:

- graphing and visualization via Matplotlib
- NT4 key listing and querying
- basic math and statistics helpers, including derivatives, integrals, min/max, averages, standard deviations, RMS values, thresholds, and settling metrics
- output gating through limits, summaries, JSON modes, and compact JSON modes, which helps avoid dumping huge logs into model context

For WPILib simulation iteration, it supports:

- automatically creating and managing sandboxes
- switching a robot repo into simulation mode
- running simulation, recording NT4 output, analyzing the resulting logs, modifying code, and repeating the loop
- workflows that currently work best with AdvantageKit-style repos

There is also support for recording NT4 from a live robot source through the same pipeline. That path exists, but it is less tested than the simulation and log-analysis workflow.

## What It Helps With

| Workflow | Examples |
| --- | --- |
| Sandbox-first experiments | Try robot-code changes without mutating the real workspace first |
| WPILib simulation | Run bounded sim checks and collect evidence from generated outputs |
| NT4 recording | Capture NetworkTables data into WPILOG files |
| WPILOG analysis | Inspect keys, query values, calculate stats, and generate graphs |
| Math checks | Run symbolic or numeric checks while debugging control logic |
| Agent setup | Install consistent project guidance for Codex, Claude Code, and Cursor |

## Example Agent Prompts

These are the kinds of tasks this tooling is intended to support:

> Using wpilib-agent-tools, create a new sandbox and diagnose/fix this superstructure behavior. I require functionality xyz and observed abc. Iterate until completion.

> Using wpilib-agent-tools, create a new sandbox and find the root cause of symptom xyz in my auto routine. I require functionality xyz and observed abc. Iterate until completion.

> Using wpilib-agent-tools, analyze this match log and check whether all subsystems are meeting their commanded setpoints throughout the match. I require functionality xyz and observed abc.

## Install Options

### Recommended: shared installer

Use the repo-first installer for the full CLI plus workspace harness setup:

```bash
./scripts/install_all.sh --workspace /path/to/robot-repo --harnesses all
```

This is the primary setup path for the current public snapshot because it keeps the code, docs, install scripts, and harness support together.

### CLI only via `pipx` from GitHub

```bash
pipx install "git+https://github.com/edanliahovetsky/wpilib-agent-tools.git#subdirectory=agent"
```

This is convenient when you mainly want the Python CLI. It does not replace the shared workspace installer for Codex, Claude Code, and Cursor.

### Harness support only

```bash
./scripts/install_harness_support.sh --workspace /path/to/robot-repo --harnesses all
```

Use this when the CLI is already available and you only need to provision workspace guidance and harness assets.

## Documentation

| Document | Purpose |
| --- | --- |
| [INSTALL.md](INSTALL.md) | Onboarding, install options, and agent-specific setup notes |
| [agent/README.md](agent/README.md) | Full CLI reference |
| [docs/DISTRIBUTION.md](docs/DISTRIBUTION.md) | Repo-first distribution strategy |
| [docs/VALIDATION_STATUS.md](docs/VALIDATION_STATUS.md) | Current validation coverage and known gaps |
| [CHANGELOG.md](CHANGELOG.md) | Release history |

## Current Status

This project is **experimental**.

It was developed near the beginning of build season mostly out of curiosity, then used for a few weeks during early competition-season robot-code development. In practice, focused CLI commands plus well-scoped markdown guidance made modern LLMs more effective in the FRC ecosystem than expected.

The core tooling is functional, but the packaging and distribution are still experimental. It is useful enough to try if you are already exploring agentic FRC workflows, but it still has rough edges:

- best results tend to come from stronger lead/orchestrator models and higher-reasoning modes
- the model needs enough long-horizon capability to read the robot repo, understand the tooling instructions, run the CLI, inspect evidence, and iterate without losing the thread
- encouraging the model to use subagents for broad search or parallel investigation can help with speed and token cost
- token usage, usage-based pricing, and subagent usage still deserve care
- model outputs still need normal engineering review
- hallucinated fixes, misunderstood logs, and weak math are realistic failure modes
- the live-robot NT4 recording path is less tested than the simulation and log-analysis workflow
- there are likely still bugs and rough edges, including possible bugs in the CLI itself
- Cursor support is installer-validated, but full headless/noninteractive Cursor agent validation is still a known gap

For teams using this with students, treat it as a tool for investigation, verification, and learning. It should make evidence easier to gather, not remove the need to understand the robot code.

Like with all agentic tooling, a skilled driver is still needed. The tools can make evidence gathering and iteration much faster, but they do not remove the need to review changes, manage context, prevent runaway behavior, and make sure the agent is actually aligned with the intended robot behavior.

Issues and forks are welcome.

## Distribution Model

For this public snapshot, GitHub is the source of truth for:

- code
- install instructions
- supported setup flow
- tagged releases
- issue tracking

The primary distribution model is repo-first:

```bash
./scripts/install_all.sh --workspace /path/to/robot-repo --harnesses all
```

This project is **not MCP-based**. That is a scope decision for this experiment: the current implementation is a CLI plus packaged workspace guidance and harness assets. MCP may be a good direction for adjacent FRC tooling, but this repo currently optimizes for a simple, inspectable, source-first setup.

Claude Code and Codex both have strong skill/project setup workflows, and the models should generally be capable of setting up `wpilib-agent-tools` from the repo URL with a little direction if anything goes wrong.

## Migration Note

The old top-level source path `skills/wpilib-agent-tools/` is gone.

If you previously relied on that repo path directly, migrate to the supported entrypoints instead:

- `./scripts/sync_skill.sh`
- `./scripts/install_all.sh`
- `./scripts/install_harness_support.sh`

The canonical in-repo Codex skill source now lives under:

```text
agent/src/wpilib_agent_tools/integrations/codex/skill_bundle
```

## Common Commands

| Command | Purpose |
| --- | --- |
| `make test` | Run the Python test suite |
| `make skill-validate` | Validate the packaged Codex skill bundle |
| `make smoke` | Run repo smoke checks |
| `make validate-2026` | Run the configured 2026 robot-repo validation flow |
| `make release-check` | Run the release readiness checks |

Repo automation prefers local code so validation reflects latest edits:

1. `WPILIB_AGENT_TOOLS_CLI` override, if set
2. `./.venv/bin/wpilib-agent-tools`
3. `wpilib-agent-tools` from `PATH`
4. `python3 -m wpilib_agent_tools` via repo `agent/src`

## Release Hygiene

Before tagging a release:

```bash
make release-check
```

Then push a version tag to publish a GitHub Release with built artifacts:

```bash
git tag v0.2.0
git push origin v0.2.0
```

Tag pushes matching `v*` trigger [.github/workflows/release.yml](.github/workflows/release.yml), which runs checks, builds `sdist`/`wheel`, and uploads assets to the release.

Update [CHANGELOG.md](CHANGELOG.md) before tagging.
