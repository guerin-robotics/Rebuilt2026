#!/usr/bin/env bash
set -euo pipefail

# Kill any existing tool/sim processes so validation runs are single-instance.
targets=(
  "wpilib-agent-tools"
  "python.*-m wpilib_agent_tools"
  "gradlew .*simulateJava"
  "sim_wrapper.py"
)

self_pid="$$"
parent_pid="$PPID"
declare -a to_kill=()

for target in "${targets[@]}"; do
  while IFS= read -r pid; do
    [[ -z "${pid}" ]] && continue
    [[ "${pid}" == "${self_pid}" || "${pid}" == "${parent_pid}" ]] && continue
    to_kill+=("${pid}")
  done < <(pgrep -f "${target}" || true)
done

if [[ "${#to_kill[@]}" -eq 0 ]]; then
  echo "No matching wpilib-agent-tools/sim processes found."
  exit 0
fi

unique_pids="$(printf "%s\n" "${to_kill[@]}" | sort -u)"
echo "Stopping existing processes: ${unique_pids//$'\n'/ }"
kill -TERM ${unique_pids} || true
sleep 1
for pid in ${unique_pids}; do
  if kill -0 "${pid}" 2>/dev/null; then
    kill -KILL "${pid}" || true
  fi
done
