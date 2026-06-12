# Changelog

All notable changes to this project are documented in this file.

The format is based on Keep a Changelog and this project follows semantic versioning.

## [Unreleased]

## [0.2.0] - 2026-04-28

### Added
- One-command installer `scripts/install_all.sh` for Codex, Claude Code, and Cursor setup.
- Dedicated installers:
  - `scripts/install_cli.sh`
  - `scripts/install_cursor_rules.sh`
  - `scripts/install_harness_support.sh`
- Release readiness script `scripts/release_check.sh`.
- Make targets for install workflows and release checks.
- Tag-triggered GitHub release workflow at `.github/workflows/release.yml`.
- Top-level `LICENSE` for public repository sharing.
- Canonical install guide at `INSTALL.md`.
- Distribution notes at `docs/DISTRIBUTION.md`.
- Validation status report at `docs/VALIDATION_STATUS.md`.
- Shared `harness install` CLI support for Codex, Claude Code, and Cursor.

### Changed
- Root documentation now includes copy-paste onboarding for local development and shared harness setup.
- Root README now frames the project as an experimental repo-first public snapshot with one recommended install path and clearer maturity/distribution guidance.
- Install/docs now center a shared multi-harness installer path for Codex, Claude Code, and Cursor.
- The old top-level `skills/wpilib-agent-tools/` source path has been removed; the canonical Codex skill source is now package-centered and migration should happen through the supported scripts.

## [0.1.0] - 2026-02-19

### Added
- Initial `wpilib-agent-tools` CLI with sandbox-first sim/log/query workflows.
- Codex skill bundle for agent-facing workflow integration.
- Validation and smoke tooling for local workflow verification.
