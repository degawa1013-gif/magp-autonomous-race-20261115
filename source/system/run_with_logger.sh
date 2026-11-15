#!/usr/bin/env bash
set -euo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

# Calibration observes the existing receiver service without starting driving.
if [ "${1:-}" = "--rc-calibrate" ]; then
  shift
  exec python3 rc_calibrate.py "$@"
fi

for argument in "$@"; do
  if [ "${argument}" = "--rc-calibrate" ]; then
    echo "--rc-calibrate must be the first argument" >&2
    exit 2
  fi
done

for argument in "$@"; do
  case "${argument}" in
    --rc-shadow|--rc-input-check|-h|--help)
      exec python3 robust_lane_failsafe.py "$@"
      ;;
  esac
done

# Claim the whole launch before creating the logger or touching services. This
# is separate from Python's actuator lock, and remains held through cleanup.
# Keep the file in place so concurrent launches always lock the same inode.
exec 9>>"/tmp/magp-driving-wrapper.lock"
if ! flock --nonblock 9; then
  echo "走行システムの起動処理はすでに実行中です。既存の動作と記録を継続します。" >&2
  exit 1
fi

if ! pgrep -x pigpiod >/dev/null 2>&1; then
  sudo pigpiod || true
fi

LOGGER_PID=""
WIFI_MONITOR_PID=""
cleanup() {
  if [ -n "${LOGGER_PID}" ] && kill -0 "${LOGGER_PID}" >/dev/null 2>&1; then
    kill "${LOGGER_PID}" >/dev/null 2>&1 || true
    wait "${LOGGER_PID}" >/dev/null 2>&1 || true
  fi
  LOGGER_PID=""
  if [ -n "${WIFI_MONITOR_PID}" ] && kill -0 "${WIFI_MONITOR_PID}" >/dev/null 2>&1; then
    kill "${WIFI_MONITOR_PID}" >/dev/null 2>&1 || true
    wait "${WIFI_MONITOR_PID}" >/dev/null 2>&1 || true
  fi
  WIFI_MONITOR_PID=""
}
trap cleanup EXIT INT TERM

python3 status_csv_logger.py &
LOGGER_PID="$!"

if [ -f "wifi_monitor.py" ]; then
  mkdir -p output/wifi_capture
  # This optional observer never owns the driving lock or vehicle output.
  # Closing fd 9 prevents a surviving observer from blocking a later launch.
  python3 wifi_monitor.py --output-dir output/wifi_capture --interval 2 \
    --exit-after-unavailable 60 9>&- >>output/wifi_capture/runtime.log 2>&1 &
  WIFI_MONITOR_PID="$!"
fi

python3 robust_lane_failsafe.py "$@"
cleanup

LATEST_RUN_DIR="$(python3 - <<'PY'
from pathlib import Path
runs = (p for p in Path('output').glob('run_*') if p.is_dir())
latest = max(runs, key=lambda p: p.stat().st_mtime, default=None)
print(latest or '')
PY
)"
if [ -n "${LATEST_RUN_DIR}" ]; then
  RUN_NAME="$(basename "${LATEST_RUN_DIR}")"
  if [ -f "${LATEST_RUN_DIR}/control_log.csv" ] && [ -f "${LATEST_RUN_DIR}/mask_log.mp4" ]; then
    (
      cd "${LATEST_RUN_DIR}"
      python3 -m zipfile -c "../${RUN_NAME}.zip" control_log.csv mask_log.mp4
    )
    echo "Final archive: ${PWD}/output/${RUN_NAME}.zip"
  fi
fi
