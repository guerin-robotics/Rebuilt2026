#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/install_cli.sh [--mode local|pipx] [--pipx-spec <spec>] [--force]

Modes:
  local  Install editable CLI into repo virtualenv via scripts/bootstrap.sh (default)
  pipx   Install globally via pipx

Examples:
  scripts/install_cli.sh
  scripts/install_cli.sh --mode pipx
  scripts/install_cli.sh --mode pipx --force
  scripts/install_cli.sh --mode pipx --pipx-spec "git+https://github.com/edanliahovetsky/wpilib-agent-tools.git#subdirectory=agent"
EOF
}

MODE="local"
FORCE=0
PIPX_SPEC="${WPILIB_AGENT_TOOLS_PIPX_SPEC:-git+https://github.com/edanliahovetsky/wpilib-agent-tools.git#subdirectory=agent}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      MODE="$2"
      shift 2
      ;;
    --pipx-spec)
      PIPX_SPEC="$2"
      shift 2
      ;;
    --force)
      FORCE=1
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

case "${MODE}" in
  local)
    "${REPO_ROOT}/scripts/bootstrap.sh"
    ;;
  pipx)
    if ! command -v pipx >/dev/null 2>&1; then
      echo "pipx is required for --mode pipx. Install pipx first." >&2
      exit 1
    fi
    if pipx list --short 2>/dev/null | grep -q '^wpilib-agent-tools '; then
      if [[ "${FORCE}" -eq 1 ]]; then
        pipx reinstall wpilib-agent-tools
      else
        pipx upgrade --include-injected wpilib-agent-tools
      fi
    else
      if [[ "${FORCE}" -eq 1 ]]; then
        pipx install --force "${PIPX_SPEC}"
      else
        pipx install "${PIPX_SPEC}"
      fi
    fi
    ;;
  *)
    echo "--mode must be one of: local, pipx" >&2
    exit 2
    ;;
esac

echo "CLI install complete (mode=${MODE})."
