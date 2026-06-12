#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/install_all.sh [options]

Options:
  --cli-mode <local|pipx>     CLI install mode (default: local)
  --pipx-spec <spec>          pipx install spec when --cli-mode pipx
  --skill-mode <symlink|copy> Codex skill sync mode (default: symlink)
  --codex-home <path>         Codex home path (default: $CODEX_HOME or ~/.codex)
  --skip-codex                Skip Codex skill sync
  --skip-checks               Skip skill validation + smoke checks
  --workspace <path>          Install shared harness support into a target workspace
  --harnesses <all|list>      Harnesses to install (default: all)
  --cursor-workspace <path>   Backward-compatible alias for --workspace with Cursor-only install
  --cursor-mode <core|all>    Cursor rule mode within shared harness install (default: core)
  --cursor-force              Overwrite existing generated harness/rule files
  --help                      Show this message

Migration:
  The old top-level repo source path `skills/wpilib-agent-tools/` has been removed.
  Use this script or scripts/sync_skill.sh instead of hardcoding source paths.

Examples:
  scripts/install_all.sh
  scripts/install_all.sh --cli-mode pipx --skill-mode copy
  scripts/install_all.sh --workspace ~/FRC/2026-Robot-Code --harnesses all
  scripts/install_all.sh --cursor-workspace ~/FRC/2026-Robot-Code --cursor-mode core
EOF
}

CLI_MODE="local"
PIPX_SPEC=""
SKILL_MODE="symlink"
CODEX_HOME="${CODEX_HOME:-${HOME}/.codex}"
SKIP_CODEX=0
SKIP_CHECKS=0
WORKSPACE=""
HARNESSES="all"
CURSOR_WORKSPACE=""
CURSOR_MODE="core"
CURSOR_FORCE=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --cli-mode)
      CLI_MODE="$2"
      shift 2
      ;;
    --pipx-spec)
      PIPX_SPEC="$2"
      shift 2
      ;;
    --skill-mode)
      SKILL_MODE="$2"
      shift 2
      ;;
    --codex-home)
      CODEX_HOME="$2"
      shift 2
      ;;
    --skip-codex)
      SKIP_CODEX=1
      shift
      ;;
    --skip-checks)
      SKIP_CHECKS=1
      shift
      ;;
    --workspace)
      WORKSPACE="$2"
      shift 2
      ;;
    --harnesses)
      HARNESSES="$2"
      shift 2
      ;;
    --cursor-workspace)
      CURSOR_WORKSPACE="$2"
      shift 2
      ;;
    --cursor-mode)
      CURSOR_MODE="$2"
      shift 2
      ;;
    --cursor-force)
      CURSOR_FORCE=1
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

if [[ "${CLI_MODE}" != "local" && "${CLI_MODE}" != "pipx" ]]; then
  echo "--cli-mode must be one of: local, pipx" >&2
  exit 2
fi
if [[ "${SKILL_MODE}" != "symlink" && "${SKILL_MODE}" != "copy" ]]; then
  echo "--skill-mode must be one of: symlink, copy" >&2
  exit 2
fi
if [[ "${CURSOR_MODE}" != "core" && "${CURSOR_MODE}" != "all" ]]; then
  echo "--cursor-mode must be one of: core, all" >&2
  exit 2
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -n "${CURSOR_WORKSPACE}" ]]; then
  WORKSPACE="${CURSOR_WORKSPACE}"
  HARNESSES="cursor"
fi

AUTO_SKIP_CODEX=0
if [[ -n "${WORKSPACE}" && "${SKIP_CODEX}" -eq 0 ]]; then
  case ",${HARNESSES}," in
    *,all,*|*,codex,*)
      ;;
    *)
      AUTO_SKIP_CODEX=1
      ;;
  esac
fi

echo "[1/4] Installing CLI (${CLI_MODE})"
CLI_CMD=("${REPO_ROOT}/scripts/install_cli.sh" --mode "${CLI_MODE}")
if [[ -n "${PIPX_SPEC}" ]]; then
  CLI_CMD+=(--pipx-spec "${PIPX_SPEC}")
fi
"${CLI_CMD[@]}"

if [[ "${SKIP_CODEX}" -eq 0 && "${AUTO_SKIP_CODEX}" -eq 0 ]]; then
  echo "[2/4] Syncing Codex skill (${SKILL_MODE})"
  "${REPO_ROOT}/scripts/sync_skill.sh" --mode "${SKILL_MODE}" --codex-home "${CODEX_HOME}"
elif [[ "${AUTO_SKIP_CODEX}" -eq 1 ]]; then
  echo "[2/4] Skipping Codex skill sync (codex not selected in --harnesses)"
else
  echo "[2/4] Skipping Codex skill sync"
fi

if [[ "${SKIP_CHECKS}" -eq 0 ]]; then
  echo "[3/4] Running validation checks"
  if [[ -x "${REPO_ROOT}/.venv/bin/python" ]]; then
    "${REPO_ROOT}/.venv/bin/python" "${REPO_ROOT}/scripts/validate_skill.py" "${REPO_ROOT}/agent/src/wpilib_agent_tools/integrations/codex/skill_bundle"
  else
    python3 "${REPO_ROOT}/scripts/validate_skill.py" "${REPO_ROOT}/agent/src/wpilib_agent_tools/integrations/codex/skill_bundle"
  fi
  "${REPO_ROOT}/scripts/smoke.sh"
else
  echo "[3/4] Skipping validation checks"
fi

if [[ -n "${WORKSPACE}" ]]; then
  echo "[4/4] Installing workspace harness support (${HARNESSES})"
  HARNESS_CMD=(
    "${REPO_ROOT}/scripts/install_harness_support.sh"
    --workspace "${WORKSPACE}"
    --harnesses "${HARNESSES}"
    --cursor-mode "${CURSOR_MODE}"
  )
  if [[ "${CURSOR_FORCE}" -eq 1 ]]; then
    HARNESS_CMD+=(--force)
  fi
  "${HARNESS_CMD[@]}"
else
  echo "[4/4] Workspace harness setup skipped (no --workspace provided)"
fi

echo "Install complete."
