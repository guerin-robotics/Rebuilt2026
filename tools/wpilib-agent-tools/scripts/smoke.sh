#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_CLI="${REPO_ROOT}/scripts/run_cli.sh"

"${RUN_CLI}" --version >/dev/null
"${RUN_CLI}" --help >/dev/null

JSON_OUT="$("${RUN_CLI}" math --mode simplify --expr "sin(x)**2 + cos(x)**2" --json)"
python3 -c 'import json,sys; payload=json.loads(sys.stdin.read()); assert payload.get("result") == "1", payload' <<<"${JSON_OUT}"

TMP_WORKSPACE="$(mktemp -d)"
trap 'rm -rf "${TMP_WORKSPACE}"' EXIT
"${RUN_CLI}" harness install --workspace "${TMP_WORKSPACE}" --json >/dev/null
test -f "${TMP_WORKSPACE}/AGENTS.md"
test -f "${TMP_WORKSPACE}/CLAUDE.md"
test -f "${TMP_WORKSPACE}/.claude/commands/wpilib-agent-tools-validate.md"
test -f "${TMP_WORKSPACE}/.cursor/rules/wpilib-agent-tools-core.mdc"

echo "Smoke checks passed."
