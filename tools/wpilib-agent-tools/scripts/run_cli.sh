#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOCAL_VENV_CLI="${REPO_ROOT}/.venv/bin/wpilib-agent-tools"
CLI_OVERRIDE="${WPILIB_AGENT_TOOLS_CLI:-}"

if [[ -n "${CLI_OVERRIDE}" ]]; then
  if command -v "${CLI_OVERRIDE}" >/dev/null 2>&1; then
    exec "${CLI_OVERRIDE}" "$@"
  fi
  echo "WPILIB_AGENT_TOOLS_CLI is set but not executable: ${CLI_OVERRIDE}" >&2
  exit 127
fi

# Prefer the repo-local editable install when available.
if [[ -x "${LOCAL_VENV_CLI}" ]]; then
  exec "${LOCAL_VENV_CLI}" "$@"
fi

if command -v wpilib-agent-tools >/dev/null 2>&1; then
  exec wpilib-agent-tools "$@"
fi

if [[ -d "${REPO_ROOT}/agent/src" ]]; then
  export PYTHONPATH="${REPO_ROOT}/agent/src:${PYTHONPATH:-}"
  exec python3 -m wpilib_agent_tools "$@"
fi

echo "wpilib-agent-tools CLI not found. Run scripts/bootstrap.sh first." >&2
exit 127
