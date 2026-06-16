# Validation Status

This document summarizes the current validation snapshot for the repo.

## Scope of the pass

The pass covered:

- repo-contained validation
- install-path validation
- package/build validation
- live harness validation where possible
- safe remediation of issues found during validation

## Repo-contained validation

Verified successfully:

- `make release-check`
- `make test`
- shell syntax for `scripts/*.sh`
- `scripts/validate_skill.py` against the canonical Codex skill bundle
- package/wheel contents include the expected integration assets
- docs/install/release surfaces were reviewed for consistency

Latest evidence from the pass:

- `make release-check` passed
- `make test` passed
- total test count: **117 passed**

## Install-path validation

Verified successfully:

- `./scripts/install_cli.sh --mode local`
- `./scripts/install_cli.sh --mode pipx`
- `./scripts/install_harness_support.sh`
- `./scripts/install_cursor_rules.sh`
- `./scripts/sync_skill.sh --mode copy --codex-home <tmp>`
- `./scripts/install_all.sh --workspace ... --harnesses all --skip-checks`

## Live harness validation

### Codex

Validated successfully:

- installed workspace support
- read generated `AGENTS.md`
- extracted the expected runner path
- ran:
  - `./.wpilib-agent-tools/run_cli.sh --version`
  - `./.wpilib-agent-tools/run_cli.sh math --mode simplify --expr 'sin(x)**2 + cos(x)**2' --json`

### Claude Code

Validated successfully:

- installed workspace support
- read generated `CLAUDE.md`
- extracted the expected runner path
- ran:
  - `./.wpilib-agent-tools/run_cli.sh --version`
  - `./.wpilib-agent-tools/run_cli.sh math --mode simplify --expr 'sin(x)**2 + cos(x)**2' --json`

Note:
- in the validation environment, actual command execution required `--dangerously-skip-permissions`

### Cursor

Partially validated:

- Cursor app present
- Cursor CLI present
- version check succeeded
- workspace rules install succeeded

Not yet validated end-to-end:

- usable headless/noninteractive `cursor agent` flow

Observed blocker during the pass:

- `cursor agent --help` timed out
- direct `cursor agent ...` attempts timed out

## Issue found and fixed during validation

### `scripts/validate_skill.py` portability bug

Problem:
- running it with system `python3` could fail if `PyYAML` was not installed

Fix:
- replaced the script's PyYAML dependency with a small built-in parser for the limited YAML subset it actually uses
- added regression coverage in:
  - `agent/tests/test_validate_skill_script.py`

Commit:
- `31e70e7` — `Make skill validation independent of optional PyYAML`

## Current confidence level

The repo has been validated for these paths:

- repo-contained checks
- install-path validation
- package/build validation
- live Codex validation
- live Claude Code validation

The main remaining explicit gap is:

- full live headless/noninteractive Cursor agent validation

## Recommended interpretation for sharing

It is reasonable to present the repo as:

- well-tested locally
- install paths validated
- Codex and Claude Code live-tested
- Cursor support present and installer-validated, with headless live-agent validation still pending
