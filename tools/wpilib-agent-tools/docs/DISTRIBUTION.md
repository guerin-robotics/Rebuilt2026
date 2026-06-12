# Distribution Notes

This document explains the distribution strategy for the current experimental public snapshot of `wpilib-agent-tools`.

## Short version

For this snapshot, the project should be distributed as a **repo-first, GitHub-canonical** tool.

That means GitHub is the source of truth for:

- the code
- the docs
- the install scripts
- the supported setup flow
- releases / tags

The supported harnesses for this release are:

- Codex
- Claude Code
- Cursor

The parity goal is **one shared installer path** that supports all three cleanly.

The canonical source-of-truth integration assets now live in the package-centered tree:

- `agent/src/wpilib_agent_tools/integrations/codex/`
- `agent/src/wpilib_agent_tools/integrations/claude/`
- `agent/src/wpilib_agent_tools/integrations/cursor/`

The former top-level source path:

- `skills/wpilib-agent-tools/`

has been removed. Migration should happen through supported scripts rather than by restoring a second source tree.

## Why not center everything around `pipx` or PyPI?

Because this project is really two things:

1. a Python CLI
2. an agent setup / skill / workflow package

`pipx` and PyPI are good tools for distributing **Python CLIs**.
They do **not** automatically solve the full setup story for:

- Codex workspace instructions
- Claude Code project/command files
- Cursor rules
- repo-local workflow docs

So if the project were advertised mainly as a `pipx` or PyPI package right now, it would make the CLI feel cleaner while leaving the overall onboarding story fragmented.

## Distribution options

### 1. Repo-first / source-first

Users clone the repo and run:

```bash
./scripts/install_all.sh --workspace /path/to/robot-repo --harnesses all
```

### Pros

- lowest maintenance burden
- matches current project structure
- easiest to debug
- most honest for an experimental but useful tool

### Cons

- more friction than a one-line package install
- still assumes users are comfortable with a repo/workspace integration model

### 2. `pipx` from GitHub

Users install the CLI directly from the repo:

```bash
pipx install "git+https://github.com/edanliahovetsky/wpilib-agent-tools.git#subdirectory=agent"
```

### Pros

- clean CLI experience
- no PyPI publishing needed
- nice convenience path for power users

### Cons

- only solves the CLI cleanly
- still needs the shared harness installer for Codex / Claude Code / Cursor
- not ideal as the single headline onboarding story

### 3. PyPI

If the CLI is published to PyPI, users could install it with:

```bash
pipx install wpilib-agent-tools
```

### Pros

- best Python CLI UX
- familiar for many users
- good long-term path if the CLI becomes independently valuable

### Cons

- adds package publishing/release overhead
- still does not solve the shared harness/setup half by itself
- can over-signal polish if the rest of the workflow is still repo-driven

### 4. GitHub Releases

The repo can publish tagged releases with built artifacts.

### Pros

- versioned artifacts
- cleaner than “clone main”
- useful for release history and reproducibility

### Cons

- still needs repo docs to explain setup
- adds some release management overhead

### 5. Three unrelated agent-specific installers

One possible design would be separate installers for:

- Codex
- Claude Code
- Cursor
- better UX per tool in isolation
- potentially more native-feeling installs

### Cons

- highest maintenance burden
- easy to fragment the docs
- violates the shared-installer goal

## Recommendation for this snapshot

Use this structure:

### Primary

- GitHub repo
- `INSTALL.md`
- `./scripts/install_all.sh --workspace /path/to/robot-repo --harnesses all`
- `./scripts/install_harness_support.sh --workspace /path/to/robot-repo --harnesses all`

### Secondary

- `pipx` from GitHub for CLI installation
- manual component install for advanced users

### Deferred

- PyPI
- polished multi-agent packaging
- more opinionated platform-specific installers

## Why this is the right tradeoff now

It matches the real state of the project:

- useful
- real
- worth trying
- still experimental, with rough edges

It also keeps the public story honest:

- there is one shared installer story for Codex, Claude Code, and Cursor
- alternate paths exist, but are not competing “official” stories
- future packaging improvements can happen after real user feedback

For the latest tested coverage and remaining gaps, see:

- [VALIDATION_STATUS.md](VALIDATION_STATUS.md)
