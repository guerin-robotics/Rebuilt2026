#!/usr/bin/env bash
#
# sync-logs.sh — pull new WPILib logs off the roboRIO over SFTP and upload them
# to Google Drive, without ever mounting the robot's USB stick on this computer.
#
# Requires two rclone remotes (see README.md for the one-time setup):
#   roborio:  SFTP to the roboRIO  (host + user configured inside the remote)
#   gdrive:   Google Drive         (OAuth configured inside the remote)
#
# rclone only transfers files the destination is missing, so re-running this is
# cheap and "the newest logs are always uploaded" is automatic.
#
# Usage:
#   ./sync-logs.sh            # sync once and exit
#   ./sync-logs.sh --watch    # sync forever, every $INTERVAL seconds
#
set -euo pipefail

# ---- Config (override via environment) --------------------------------------
SRC_REMOTE="${SRC_REMOTE:-roborio}"     # rclone remote name for the roboRIO
SRC_PATH="${SRC_PATH:-/U/logs}"          # log directory on the roboRIO USB
DST_REMOTE="${DST_REMOTE:-gdrive}"       # rclone remote name for Google Drive
DST_PATH="${DST_PATH:-FRC-Logs}"         # destination folder in Google Drive
INCLUDE="${INCLUDE:-*.wpilog}"           # also add *.hoot if you want CTRE logs
INTERVAL="${INTERVAL:-120}"              # seconds between syncs in --watch mode
# -----------------------------------------------------------------------------

if ! command -v rclone >/dev/null 2>&1; then
  echo "error: rclone is not installed. See tools/log-sync/README.md" >&2
  exit 1
fi

# Single-instance lock so an overlapping run (e.g. a slow upload) can't stack up.
LOCK="${TMPDIR:-/tmp}/guerin-log-sync.lock"

sync_once() {
  # copy (never delete on Drive) + --update (skip files already there).
  # --low-level-retries keeps a flaky radio link from aborting the whole run.
  if rclone copy "${SRC_REMOTE}:${SRC_PATH}" "${DST_REMOTE}:${DST_PATH}" \
        --include "${INCLUDE}" \
        --update \
        --low-level-retries 3 \
        --stats-one-line \
        --log-level INFO; then
    echo "[$(date '+%H:%M:%S')] sync ok"
  else
    # Robot unreachable / no internet is expected sometimes — don't hard-fail
    # the watch loop over it; just report and try again next tick.
    echo "[$(date '+%H:%M:%S')] sync failed (robot or internet unreachable?)" >&2
  fi
}

run_locked() {
  # flock isn't on macOS by default; fall back to a mkdir-based lock.
  if ! mkdir "${LOCK}" 2>/dev/null; then
    echo "another sync is already running (${LOCK}); skipping" >&2
    return 0
  fi
  trap 'rmdir "${LOCK}" 2>/dev/null || true' RETURN
  sync_once
}

if [[ "${1:-}" == "--watch" ]]; then
  echo "watching: ${SRC_REMOTE}:${SRC_PATH} -> ${DST_REMOTE}:${DST_PATH} every ${INTERVAL}s"
  while true; do
    run_locked
    sleep "${INTERVAL}"
  done
else
  run_locked
fi
