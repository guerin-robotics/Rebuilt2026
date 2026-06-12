#!/usr/bin/env bash
set -euo pipefail

CLI_BIN="${WPILIB_AGENT_TOOLS_CLI:-wpilib-agent-tools}"

find_repo_root() {
  local start_dir current
  start_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  current="${start_dir}"

  while [[ "${current}" != "/" ]]; do
    if [[ -d "${current}/agent/src" ]]; then
      printf '%s\n' "${current}"
      return 0
    fi
    current="$(cd "${current}/.." && pwd)"
  done

  return 1
}

# Prefer explicitly provided source repo when available.
if [[ -n "${WPILIB_AGENT_TOOLS_REPO:-}" ]]; then
  if [[ -d "${WPILIB_AGENT_TOOLS_REPO}/agent/src" ]]; then
    export PYTHONPATH="${WPILIB_AGENT_TOOLS_REPO}/agent/src:${PYTHONPATH:-}"
    exec python3 -m wpilib_agent_tools "$@"
  fi
fi

# If this skill is symlinked inside the source repo, use that checkout.
if CANDIDATE_REPO="$(find_repo_root)"; then
  export PYTHONPATH="${CANDIDATE_REPO}/agent/src:${PYTHONPATH:-}"
  exec python3 -m wpilib_agent_tools "$@"
fi

if command -v "${CLI_BIN}" >/dev/null 2>&1; then
  exec "${CLI_BIN}" "$@"
fi

echo "wpilib-agent-tools CLI not found. Install it or set WPILIB_AGENT_TOOLS_REPO." >&2
exit 127
