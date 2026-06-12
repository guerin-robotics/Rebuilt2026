# Install and Onboarding

This file is the recommended setup guide for the current experimental public snapshot of `wpilib-agent-tools`.

## Who this is for

This project is a good fit if you are:

- already using agentic coding workflows for FRC
- interested in closed-loop sim validation, NT4 recording, or WPILOG analysis
- using Codex, Claude Code, or Cursor
- comfortable with a repo/package-first install flow

This is probably **not** the right tool if you want a polished, universal integration across every coding assistant right away. The supported harnesses for this snapshot are **Codex, Claude Code, and Cursor**.

## Recommended path: clone the repo and run the installer

```bash
git clone https://github.com/edanliahovetsky/wpilib-agent-tools.git
cd wpilib-agent-tools
./scripts/install_all.sh --workspace /path/to/robot-repo --harnesses all
```

This is the recommended path because it keeps:

- the code
- the install scripts
- the docs
- the shared harness installer

all in one place.

## What `install_all.sh` does

With `--workspace`, it:

1. installs the CLI locally into `./.venv`
2. validates the skill structure
3. runs smoke checks
4. installs the shared workspace harness support

The shared workspace installer creates:

- `./.wpilib-agent-tools/run_cli.sh` inside the target workspace
- a managed Codex block in `AGENTS.md`
- a managed Claude Code block in `CLAUDE.md`
- a Claude Code slash-command file in `.claude/commands/`
- Cursor rules in `.cursor/rules/`

Useful examples:

```bash
# Local CLI + install all three harnesses into a robot repo
./scripts/install_all.sh --workspace /path/to/robot-repo --harnesses all

# Global CLI via pipx, then install all three harnesses into a robot repo
./scripts/install_all.sh --cli-mode pipx --workspace /path/to/robot-repo --harnesses all

# Install only Codex + Claude Code harness support
./scripts/install_all.sh --workspace /path/to/robot-repo --harnesses codex,claude
```

## Agent-specific notes

### Codex

Codex support is provided through the shared workspace installer. It manages a Codex block inside the target repo’s `AGENTS.md` and points it at the workspace-local runner:

```bash
./scripts/install_harness_support.sh --workspace /path/to/robot-repo --harnesses codex
```

The repo still ships a Codex skill bundle, but it now lives alongside the other integration assets in the package-centered `integrations/` tree. The primary distribution story is the shared installer rather than a Codex-only setup path.

### Claude Code

Claude Code support is also provided through the shared workspace installer. It manages:

- `CLAUDE.md`
- `.claude/commands/wpilib-agent-tools-validate.md`

Example:

```bash
./scripts/install_harness_support.sh --workspace /path/to/robot-repo --harnesses claude
```

### Cursor

Cursor support is provided through the same shared installer and still writes the packaged rule templates into `.cursor/rules/`.

Example:

```bash
./scripts/install_harness_support.sh --workspace /path/to/robot-repo --harnesses cursor --cursor-mode core
```

## Alternate install paths

### `pipx` from GitHub

If you mainly want the CLI installed globally first:

```bash
pipx install "git+https://github.com/edanliahovetsky/wpilib-agent-tools.git#subdirectory=agent"
```

What `pipx` means:

- it installs a Python CLI app into its own isolated virtual environment
- then exposes the command globally on your PATH

This is cleaner than manual venv setup for CLI tools, but you should still use the shared harness installer to provision Codex / Claude Code / Cursor support into a workspace.

### PyPI

PyPI is the Python package registry. If the project is published there in the future, the CLI could be installed with:

```bash
pipx install wpilib-agent-tools
```

or

```bash
pip install wpilib-agent-tools
```

For this snapshot, PyPI is intentionally **not** the main story. The main story is still one shared installer for the three supported harnesses.

### Manual component install

If you want complete control, you can install pieces separately:

```bash
# local CLI
./scripts/install_cli.sh --mode local

# or pipx CLI
./scripts/install_cli.sh --mode pipx

# shared harness installer
./scripts/install_harness_support.sh --workspace /path/to/robot-repo --harnesses all
```

## Recommended mental model for this snapshot

Think of `wpilib-agent-tools` as:

- a CLI plus a shared workspace installer
- one maintainable integration path for Codex, Claude Code, and Cursor
- a repo/package where the docs and install scripts remain the source of truth

That is the most honest distribution story for the current experimental state of the project.

## Validation status

The current validation state is documented in:

- [docs/VALIDATION_STATUS.md](docs/VALIDATION_STATUS.md)

Short version:

- repo-contained checks are green
- install paths have been exercised
- Codex and Claude Code were live-tested against real generated guidance
- Cursor install/rules were validated, but full headless/noninteractive `cursor agent` validation is still an explicit remaining gap

## Migration note for existing users

If you previously depended on the old repo source path:

- `skills/wpilib-agent-tools/`

that path has been removed as part of the architecture cleanup.

Use the supported scripts instead:

- `./scripts/sync_skill.sh`
- `./scripts/install_all.sh`
- `./scripts/install_harness_support.sh`

The canonical in-repo Codex skill source now lives at:

- `agent/src/wpilib_agent_tools/integrations/codex/skill_bundle`
