#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export WPILIB_AGENT_TOOLS_REPO="${REPO_ROOT}"
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  echo "Migration note: this wrapper now resolves through the package-centered integrations tree."
  echo "Do not depend on the old repo source path skills/wpilib-agent-tools/."
fi
exec python3 "${REPO_ROOT}/agent/src/wpilib_agent_tools/integrations/codex/skill_bundle/scripts/validate_robot_repo.py" "$@"
