#!/usr/bin/env python3
"""Extract only aggregate radio metrics needed for the paper (no live I/O).

Uses a whitelist; raw SSID/BSSID/MAC/address fields are never copied.
Input defaults to the frozen analysis_report_20261115 files.
"""
import argparse
import hashlib
import json
from pathlib import Path

KEYS = ["wifi_signal_dbm", "gateway_rtt_ms", "bt_lq_raw_first_link", "afh_enabled_count_first_link"]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--base", type=Path, default=Path(__file__).resolve().parents[2])
    p.add_argument("--out", type=Path)
    args = p.parse_args()
    base = args.base.resolve()
    source = base / "pico_radio_monitor_20261115/analysis_report_20261115"
    ep = source / "inputs/event_report.json"
    qp = source / "build/quant_findings.json"
    event = json.loads(ep.read_text())
    quant = json.loads(qp.read_text())
    coverage = {}
    for observer in ["pico", "pi"]:
        c = quant["coverage"][observer]
        coverage[observer] = {k: c[k] for k in ["records", "observed_minutes", "first_source_time_jst", "last_source_time_jst", "minutes_inclusive_between_first_and_last", "missing_minutes_between_first_and_last", "missing_runs_between_first_and_last"]}
    minutes = []
    for m in event["minutes"]:
        if not 591 <= m["minute"] <= 947:
            continue
        pico, pi = m["pico"], m["pi"]
        minutes.append({"minute_of_day_jst": m["minute"], "time_jst": m["time_jst"],
                        "pico_scan_count": pico["scans"], "pico_ap_count": pico.get("AP_count"),
                        "pi_diagnostic_samples": pi["diagnostic_samples"],
                        "pi_metrics": {k: pi["metrics"].get(k, {"n": 0, "mean": None, "min": None, "max": None}) for k in KEYS}})
    summary = {
        "schema": "race-radio-event-summary-v1", "date_jst": "2026-11-15",
        "frozen_report_created_jst": quant["source"]["created_jst"],
        "source_files": {
            "event_report": {"relative_path": str(ep.relative_to(base)), "sha256": hashlib.sha256(ep.read_bytes()).hexdigest()},
            "quant_findings": {"relative_path": str(qp.relative_to(base)), "sha256": hashlib.sha256(qp.read_bytes()).hexdigest()},
        },
        "observation_scope": {
            "pico": "ノートPCに接続したPico 2 Wによる周囲APのスキャン。車載Piとは別の観測点。",
            "pi": "車載Piによる接続先Wi-FiのRSSI、ゲートウェイ往復RTT、Bluetoothコントローラ診断。",
        },
        "coverage": coverage, "pico_overall": quant["Pico_overall"],
        "pi_metric_aggregates": {k: quant["Pi_metrics"][k] for k in KEYS},
        "bt_cache_age_ms": quant["Pi_metrics"]["bt_observation_age_ms"],
        "gateway_cache_age_ms": quant["Pi_metrics"]["gateway_probe_cache_age_ms"],
        "limitations": [
            "AP検出数は電波占有率・混線量・通信量の実測ではない。",
            "RSSIは受信電力であり距離を測定していない。",
            "RTTとBluetoothはキャッシュ値を含む診断スナップショットの分集計。同一観測を繰り返し含み、独立試行数ではない。",
            "Bluetooth LQはコントローラ定義の0–255指標であり、パケット成功率ではない。",
            "欠測を0として補わず、線を接続しない。Piの09:51–15:45内の欠測は102分。",
            "Pico測定開始は12:01であり、11:32の予選接触時には同地点を含むPicoデータがない。",
            "SSID・BSSID・MACアドレス・接続先の私的IPはこの要約に含めない。",
        ],
        "minutes": minutes,
    }
    assert sum(m["pico_scan_count"] for m in minutes) == 1360
    assert sum(m["pi_diagnostic_samples"] for m in minutes) == 7297
    out = args.out or base / "reports/data/radio_event_summary.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    print(str(out))


if __name__ == "__main__":
    main()
