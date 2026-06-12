#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  scripts/install_harness_support.sh --workspace <repo-path> [options]

Options:
  --workspace <path>              Target robot/workspace repo
  --harnesses <all|list>          Harnesses to install (default: all)
  --cursor-mode <core|all>        Cursor rule mode (default: core)
  --force                         Overwrite generated files
  --help                          Show this message

Examples:
  scripts/install_harness_support.sh --workspace ~/FRC/2026-Robot-Code
  scripts/install_harness_support.sh --workspace ~/FRC/2026-Robot-Code --harnesses codex,claude
  scripts/install_harness_support.sh --workspace ~/FRC/2026-Robot-Code --cursor-mode all --force
USAGE
}

WORKSPACE=""
HARNESSES="all"
CURSOR_MODE="core"
FORCE=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace)
      WORKSPACE="$2"
      shift 2
      ;;
    --harnesses)
      HARNESSES="$2"
      shift 2
      ;;
    --cursor-mode)
      CURSOR_MODE="$2"
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

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_CLI="${REPO_ROOT}/scripts/run_cli.sh"
WORKSPACE_PATH="$(cd "${WORKSPACE}" && pwd)"

CMD=(
  "${RUN_CLI}"
  harness
  install
  --workspace "${WORKSPACE_PATH}"
  --harnesses "${HARNESSES}"
  --cursor-mode "${CURSOR_MODE}"
  --fallback-runner "${RUN_CLI}"
)
if [[ "${FORCE}" -eq 1 ]]; then
  CMD+=(--force)
fi

"${CMD[@]}"
echo "Harness support installed at ${WORKSPACE_PATH} (harnesses=${HARNESSES})."
