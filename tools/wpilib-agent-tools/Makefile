SHELL := /bin/bash

REPO_ROOT := $(CURDIR)
SKILL_DIR := agent/src/wpilib_agent_tools/integrations/codex/skill_bundle
CURSOR_MODE ?= core
WORKSPACE ?=

.PHONY: bootstrap \
	test \
	skill-validate \
	skill-sync-local \
	skill-sync-copy \
	smoke \
	validate-2026 \
	install-all \
	install-codex \
	install-harness \
	install-cursor \
	install-cli-pipx \
	release-check

bootstrap:
	@./scripts/bootstrap.sh

test:
	@if [[ -x .venv/bin/pytest ]]; then \
		cd agent && PYTHONPATH=src ../.venv/bin/pytest -q; \
	else \
		cd agent && PYTHONPATH=src python3 -m pytest -q; \
	fi

skill-validate:
	@if [[ -x .venv/bin/python ]]; then \
		.venv/bin/python scripts/validate_skill.py $(SKILL_DIR); \
	else \
		python3 scripts/validate_skill.py $(SKILL_DIR); \
	fi

skill-sync-local:
	@./scripts/sync_skill.sh --mode symlink

skill-sync-copy:
	@./scripts/sync_skill.sh --mode copy

smoke:
	@./scripts/smoke.sh

validate-2026:
	@./scripts/validate_robot_repo.sh \
		--repo $(HOME)/FRC/2026-Robot-Code \
		--branch comp-dev \
		--profile 2026-robot-code \
		--keep-sandbox-on-fail

install-all:
	@./scripts/install_all.sh

install-codex:
	@./scripts/sync_skill.sh --mode symlink

install-harness:
	@if [[ -z "$(WORKSPACE)" ]]; then \
		echo "Set WORKSPACE=/path/to/robot-repo"; \
		exit 2; \
	fi
	@./scripts/install_harness_support.sh --workspace "$(WORKSPACE)" --harnesses all --cursor-mode "$(CURSOR_MODE)"

install-cursor:
	@if [[ -z "$(WORKSPACE)" ]]; then \
		echo "Set WORKSPACE=/path/to/robot-repo"; \
		exit 2; \
	fi
	@./scripts/install_cursor_rules.sh --workspace "$(WORKSPACE)" --mode "$(CURSOR_MODE)"

install-cli-pipx:
	@./scripts/install_cli.sh --mode pipx

release-check:
	@./scripts/release_check.sh
