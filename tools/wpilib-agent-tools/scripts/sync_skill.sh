#!/usr/bin/env bash
set -euo pipefail

MODE="symlink"
CODEX_HOME="${CODEX_HOME:-${HOME}/.codex}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      MODE="$2"
      shift 2
      ;;
    --codex-home)
      CODEX_HOME="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

if [[ "${MODE}" != "symlink" && "${MODE}" != "copy" ]]; then
  echo "--mode must be one of: symlink, copy" >&2
  exit 2
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SOURCE_DIR="${REPO_ROOT}/agent/src/wpilib_agent_tools/integrations/codex/skill_bundle"
TARGET_DIR="${CODEX_HOME}/skills/wpilib-agent-tools"

if [[ ! -d "${SOURCE_DIR}" ]]; then
  echo "Skill source directory not found: ${SOURCE_DIR}" >&2
  exit 1
fi

mkdir -p "${CODEX_HOME}/skills"
if [[ -L "${TARGET_DIR}" || -d "${TARGET_DIR}" ]]; then
  rm -rf "${TARGET_DIR}"
fi

if [[ "${MODE}" == "symlink" ]]; then
  ln -s "${SOURCE_DIR}" "${TARGET_DIR}"
else
  cp -R "${SOURCE_DIR}" "${TARGET_DIR}"
fi

echo "Synced skill to ${TARGET_DIR} (${MODE})."
echo "Canonical repo source: ${SOURCE_DIR}"
echo "Migration note: the old repo path skills/wpilib-agent-tools/ has been removed; use scripts/sync_skill.sh instead of hardcoding source paths."
