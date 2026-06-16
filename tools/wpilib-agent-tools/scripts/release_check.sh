#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "[1/4] Unit tests"
make -C "${REPO_ROOT}" test

echo "[2/4] Skill validation"
make -C "${REPO_ROOT}" skill-validate

echo "[3/4] Smoke checks"
make -C "${REPO_ROOT}" smoke

echo "[4/4] Packaging check"
if [[ -x "${REPO_ROOT}/.venv/bin/python" ]]; then
  "${REPO_ROOT}/.venv/bin/python" -m pip install --quiet build
  "${REPO_ROOT}/.venv/bin/python" -m build "${REPO_ROOT}/agent" --sdist --wheel --outdir "${REPO_ROOT}/dist"
else
  python3 -m pip install --quiet build
  python3 -m build "${REPO_ROOT}/agent" --sdist --wheel --outdir "${REPO_ROOT}/dist"
fi

echo "Release check passed."
