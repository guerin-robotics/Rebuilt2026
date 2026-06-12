#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_PATH="${REPO_ROOT}/.venv"
PYTHON_BIN="${PYTHON:-python3}"

if [[ ! -d "${VENV_PATH}" ]]; then
  "${PYTHON_BIN}" -m venv "${VENV_PATH}"
fi

"${VENV_PATH}/bin/pip" install --upgrade pip
"${VENV_PATH}/bin/pip" install -e "${REPO_ROOT}/agent" pytest pyyaml

"${REPO_ROOT}/scripts/smoke.sh"

echo "Bootstrap complete."
