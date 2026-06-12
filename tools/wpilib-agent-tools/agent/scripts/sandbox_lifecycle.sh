#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  sandbox_lifecycle.sh create --name <id> --source <workspace|branch:x|rev:y> [--force] [--json]
  sandbox_lifecycle.sh delete --name <id> [--force] [--json]
  sandbox_lifecycle.sh list [--json]
  sandbox_lifecycle.sh clean --all [--older-than <hours>] [--force] [--json]

Notes:
  - This script is a non-interactive wrapper over `wpilib-agent-tools sandbox ...`.
  - It never mutates the user workspace directly.
EOF
}

CLI_BIN="${WPILIB_AGENT_TOOLS_CLI:-wpilib-agent-tools}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

run_cli() {
  if command -v "${CLI_BIN}" >/dev/null 2>&1; then
    "${CLI_BIN}" "$@"
    return
  fi
  PYTHONPATH="${PROJECT_ROOT}/src:${PYTHONPATH:-}" python3 -m wpilib_agent_tools "$@"
}

if [[ $# -lt 1 ]]; then
  usage
  exit 2
fi

action="$1"
shift

case "${action}" in
  create)
    run_cli sandbox create "$@"
    ;;
  delete)
    run_cli sandbox clean "$@"
    ;;
  list)
    run_cli sandbox list "$@"
    ;;
  clean)
    run_cli sandbox clean "$@"
    ;;
  *)
    echo "Unknown action: ${action}" >&2
    usage
    exit 2
    ;;
esac
