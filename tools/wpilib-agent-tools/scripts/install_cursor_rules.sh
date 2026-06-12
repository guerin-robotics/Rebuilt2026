#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/install_cursor_rules.sh --workspace <repo-path> [--mode core|all] [--force]

Examples:
  scripts/install_cursor_rules.sh --workspace ~/FRC/2026-Robot-Code
  scripts/install_cursor_rules.sh --workspace ~/FRC/2026-Robot-Code --mode all --force
EOF
}

WORKSPACE=""
MODE="core"
FORCE=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace)
      WORKSPACE="$2"
      shift 2
      ;;
    --mode)
      MODE="$2"
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

if [[ -z "${WORKSPACE}" ]]; then
  echo "--workspace is required." >&2
  usage >&2
  exit 2
fi

if [[ "${MODE}" != "core" && "${MODE}" != "all" ]]; then
  echo "--mode must be one of: core, all" >&2
  exit 2
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_PATH="$(cd "${WORKSPACE}" && pwd)"

CMD=(
  "${REPO_ROOT}/scripts/install_harness_support.sh"
  --workspace "${WORKSPACE_PATH}"
  --harnesses cursor
  --cursor-mode "${MODE}"
)
if [[ "${FORCE}" -eq 1 ]]; then
  CMD+=(--force)
fi

"${CMD[@]}"
echo "Cursor rules installed at ${WORKSPACE_PATH}/.cursor/rules (mode=${MODE})."
