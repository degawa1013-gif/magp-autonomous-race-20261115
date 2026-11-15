#!/usr/bin/env bash
set -euo pipefail
MAGP_G2_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'HELP'
使い方: bash ./run_with_g2.sh
走行制御・DualSense入力・Web操作・G2状態表示を一緒に起動します。
Wi-Fi・Bluetooth診断も自動記録します（output/wifi_capture）。
起動時は停止・Nです。起動だけでは走行を開始しません。
iPhoneのEvenアプリで、表示されたG2用URLのQRを開いてください。
HELP
  exit 0
fi
if [[ $# -ne 0 ]]; then
  printf '%s\n' 'この起動コマンドに追加引数は不要です。--helpで使い方を表示します。' >&2
  exit 2
fi
if [[ ! -r "${MAGP_G2_DIR}/even_hud_dist/g2.html" ]]; then
  printf '%s\n' 'G2用画面が見つかりません。' >&2
  exit 2
fi

# Display the current address and reuse the existing vehicle process. This
# check never changes its selected mode, gear or output. The logger launcher
# retains the process lock if another launch wins the race after this check.
if python3 - "${MAGP_G2_DIR}" <<'PY'
import ipaddress
import json
from pathlib import Path
import subprocess
import sys
import urllib.request

host = "raspberrypi.local"
try:
    result = subprocess.run(["ip", "-j", "-4", "route", "get", "1.1.1.1"],
                            capture_output=True, text=True, timeout=1, check=True)
    route = json.loads(result.stdout)[0]
    candidate = route.get("prefsrc") or route.get("src")
    host = str(ipaddress.IPv4Address(candidate))
except (OSError, ValueError, IndexError, subprocess.SubprocessError):
    pass
print(f"操作画面: http://{host}:8000/", flush=True)
print(f"G2用URL: http://{host}:8000/g2.html?v=controller-status-2", flush=True)
print("iPhoneのEvenアプリでMAGPを開くとG2へ接続します。", flush=True)
try:
    with urllib.request.urlopen("http://127.0.0.1:8000/status.json", timeout=1) as response:
        status = json.load(response)
    if (status.get("system_variant") == "original_20260523_direct_pi"
            and isinstance(status.get("vehicle_control"), dict)
            and status.get("actuator_output", {}).get("transport") == "pi_direct_pwm"):
        control = status["vehicle_control"]
        root = Path(sys.argv[1])
        monitor = root / "wifi_monitor.py"
        if monitor.is_file():
            try:
                destination = root / "output" / "wifi_capture"
                destination.mkdir(parents=True, exist_ok=True)
                with (destination / "runtime.log").open("ab") as log:
                    subprocess.Popen(
                        [sys.executable, str(monitor), "--output-dir", str(destination),
                         "--interval", "2", "--exit-after-unavailable", "60"],
                        cwd=root, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                        start_new_session=True, close_fds=True)
                print(f"Wi-Fi・Bluetooth診断ログ: {destination}", flush=True)
            except OSError as error:
                print(f"Wi-Fi診断の起動を確認してください: {error}", file=sys.stderr, flush=True)
        print(f"すでに同時起動中です。現在の状態を継続します: {control.get('mode')} / {control.get('gear')}", flush=True)
        sys.exit(0)
except (OSError, ValueError, TypeError):
    pass
sys.exit(1)
PY
then
  exit 0
fi

exec bash "${MAGP_G2_DIR}/run_with_logger.sh"
