#!/usr/bin/env bash
# Collect a small batch of accepted upstream episodes and append them to data/episodes/.
#
# Flow:
#   1. Run batch preflight into /tmp staging (safe: failed runs do not touch archive)
#   2. On PASS, import episode_*/ directories into the persistent archive
#
# Examples:
#   # One red-box episode today (default)
#   ./scripts/collect_daily_episodes.sh
#
#   # Three objects, one episode each
#   COLLECT_OBJECTS="object_red_box object_blue_cylinder object_green_sphere" ./scripts/collect_daily_episodes.sh
#
#   # Portfolio: 10 usable demos in one session
#   COLLECT_TARGET=10 BATCH_PREFLIGHT_EPISODES=10 ./scripts/collect_daily_episodes.sh
#
#   # Import an existing /tmp batch output without re-running sim
#   ./scripts/collect_daily_episodes.sh --import-only /tmp/ros2_arm_batch_preflight_xxx

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARCHIVE_ROOT="${EPISODE_ARCHIVE_ROOT:-${ROOT_DIR}/data/episodes}"
IMPORT_ONLY=""
STAGING_ROOT=""

usage() {
  cat <<'EOF'
Usage:
  collect_daily_episodes.sh
  collect_daily_episodes.sh --import-only <batch_output_root>

Environment:
  COLLECT_OBJECTS          Objects to collect (default: object_red_box)
  BATCH_PREFLIGHT_EPISODES Episodes per object in one sim session (default: 1)
  COLLECT_TARGET           Stop after archive has this many episodes (default: 1)
  EPISODE_ARCHIVE_ROOT     Persistent archive root (default: data/episodes)
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --import-only)
      IMPORT_ONLY="${2:-}"
      if [[ -z "${IMPORT_ONLY}" ]]; then
        echo "--import-only requires a batch output path" >&2
        exit 2
      fi
      shift 2
      ;;
    -h|--help)
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

log() { echo "[collect-daily] $*"; }

episode_count() {
  python3 "${ROOT_DIR}/scripts/episode_archive.py" status --root "${ARCHIVE_ROOT}" 2>/dev/null \
    | awk '/^Episodes:/{print $2; exit}'
}

if [[ -n "${IMPORT_ONLY}" ]]; then
  STAGING_ROOT="${IMPORT_ONLY}"
  log "import-only mode: ${STAGING_ROOT}"
else
  TARGET_EPISODES="${COLLECT_TARGET:-1}"
  CURRENT_EPISODES="$(episode_count 2>/dev/null || echo 0)"
  if (( CURRENT_EPISODES >= TARGET_EPISODES )); then
    log "archive already has ${CURRENT_EPISODES} episode(s) (target ${TARGET_EPISODES})"
    exit 0
  fi

  export BATCH_PREFLIGHT_OBJECTS="${COLLECT_OBJECTS:-object_red_box}"
  NEED=$((TARGET_EPISODES - CURRENT_EPISODES))
  export BATCH_PREFLIGHT_EPISODES="${BATCH_PREFLIGHT_EPISODES:-${NEED}}"
  STAMP="$(date +%Y%m%d_%H%M%S)"
  STAGING_ROOT="${BATCH_PREFLIGHT_OUTPUT_ROOT:-${ROOT_DIR}/data/.staging/batch_${STAMP}}"
  export BATCH_PREFLIGHT_OUTPUT_ROOT="${STAGING_ROOT}"
  log "collecting objects: ${BATCH_PREFLIGHT_OBJECTS}"
  log "episodes per object this session: ${BATCH_PREFLIGHT_EPISODES}"
  log "archive target: ${TARGET_EPISODES} (currently ${CURRENT_EPISODES})"
  log "staging root: ${STAGING_ROOT}"
  bash "${ROOT_DIR}/scripts/run_batch_preflight_smoke.sh"
  if [[ ! -d "${STAGING_ROOT}" ]]; then
    echo "Staging output not found: ${STAGING_ROOT}" >&2
    exit 1
  fi
  log "staging output: ${STAGING_ROOT}"
fi

python3 "${ROOT_DIR}/scripts/quality_gate_episode.py" "${STAGING_ROOT}" || {
  echo "[collect-daily] staging failed physics quality gate; not importing" >&2
  exit 1
}

python3 "${ROOT_DIR}/scripts/episode_archive.py" status --root "${ARCHIVE_ROOT}" || true
log "importing into LeRobot v2.1 archive: ${ARCHIVE_ROOT}"
python3 "${ROOT_DIR}/scripts/episode_archive.py" import "${STAGING_ROOT}" --dest "${ARCHIVE_ROOT}"

log "transcode any remaining legacy Arrow episodes"
python3 "${ROOT_DIR}/scripts/transcode_episode_to_video.py" "${ARCHIVE_ROOT}" || true

log "physics quality gate on archive"
python3 "${ROOT_DIR}/scripts/quality_gate_episode.py" "${ARCHIVE_ROOT}" || {
  echo "[collect-daily] imported archive failed physics quality gate" >&2
  exit 1
}

log "archive status:"
python3 "${ROOT_DIR}/scripts/episode_archive.py" status --root "${ARCHIVE_ROOT}"

log "validate full archive:"
python3 "${ROOT_DIR}/scripts/validate_dataset.py" "${ARCHIVE_ROOT}" --min-frames 5 --json \
  | tee "${ARCHIVE_ROOT}/latest_validate_dataset.json"

log "PASS: episodes appended to ${ARCHIVE_ROOT}"
