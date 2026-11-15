#!/usr/bin/env python3
"""Recalculate paper metrics from saved race captures; never access the vehicle.

Python >=3.9, standard library. Pillow is optional for verifying saved mask PNGs.
Example: python3 reports/scripts/analyze_runs.py --base /path/to/magp-recovery
New event inventory: ... --reviews saved_review_dir --inventory-only --out /tmp/new-event
The caller may relocate --out. Source JSONL/ZIP/CSV files are read only.
"""
import argparse
import collections
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import statistics
from datetime import datetime, timezone, timedelta
import zipfile

JST = timezone(timedelta(hours=9))
CAPTURE = re.compile(r"magp-capture-(\d{8}-\d{6}-[a-z0-9]+)-s(\d+)")
REVIEWS = (
    "qualifying_review_20261115_1137", "oscillation_review_20261115_101513",
    "regression_review_20261115_095920", "wobble_review_20261115_0926",
    "drive_review_20261114_1123", "hairpin_review_20261114_1152",
    "stability_review_20261114_1254", "wobble_review_20261114_1521",
    "anticipation_review_20261114", "right_angle_review_20261114_1702",
    "straight_pace_review_20261115",
)
QUALIFYING = "20261115-112838-1628279c"


def sha(data):
    return hashlib.sha256(data).hexdigest()


def jst(milliseconds):
    return datetime.fromtimestamp(milliseconds / 1000, JST).isoformat(timespec="milliseconds")


def finite(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def q(vals, probability):
    """Linear interpolation at (n-1)*p, matching numpy percentile default."""
    vals = sorted(v for v in vals if finite(v))
    if not vals:
        return None
    i = (len(vals) - 1) * probability
    lo, hi = math.floor(i), math.ceil(i)
    return vals[lo] * (hi - i) + vals[hi] * (i - lo) if lo != hi else vals[lo]


def stats(vals):
    vals = [v for v in vals if finite(v)]
    if not vals:
        return {"count": 0}
    return dict(count=len(vals), minimum=min(vals), mean=statistics.fmean(vals),
                median=q(vals, .5), p95=q(vals, .95), maximum=max(vals))


def get(d, path, default=None):
    for k in path.split("."):
        if not isinstance(d, dict) or k not in d:
            return default
        d = d[k]
    return d


def reduced(r):
    m = r["metadata"]
    a = m.get("actuator_output") or {}
    return {
        "source": r["_source"], "source_line": r["_line"],
        "segment": r["_segment"], "frame": r.get("frame"),
        "frame_index": m.get("frame_index"),
        "time_unix_ms": r["capture_time_unix_ms"],
        "time_jst": jst(r["capture_time_unix_ms"]),
        "vehicle_mode": get(m, "vehicle_control.mode", r.get("mode")),
        "capture_mode": r.get("mode"), "perception_mode": m.get("mode"),
        "reason": m.get("reason"), "confidence": m.get("confidence"),
        "point_count": len(m.get("points") or []),
        "image_error_px": m.get("raw_center_error"),
        "steering_command_us": get(a, "pi_steering.commanded_pulse_us"),
        "throttle_command_us": get(a, "pi_throttle.commanded_pulse_us"),
        "steering_logical_duty": m.get("output_steer"),
        "throttle_logical_duty": m.get("output_speed"),
        "requested_forward_percent": m.get("auto_output_setting_percent"),
        "forward_command_percent": get(m, "corner_pace.applied_percent"),
        "encoder_speed_kph": m.get("encoder_speed_kph"),
        "encoder_count": m.get("encoder_pulse_count"),
        "camera_to_output_ms": get(m, "loop_timing.camera_to_output_ms"),
        "vision_ms": get(m, "loop_timing.vision_ms"),
        "frame_interval_ms": get(m, "loop_timing.frame_interval_ms"),
        "plan_to_decision_ms": get(m, "fast_control_output.plan_to_decision_ms"),
        "fast_output_error": get(m, "fast_control_output.error"),
        "preview_active": get(m, "steering_preview.active"),
        "preview_delta_duty": get(m, "steering_preview.extra_duty"),
        "return_delta_duty": get(m, "return_damping.delta_duty"),
        "mask_ratio": m.get("mask_ratio"),
        "source_sha256": get(m, "build.source_sha256"),
    }


def summarize(rows):
    times = [r["time_unix_ms"] for r in rows]
    counts = collections.Counter(r["perception_mode"] for r in rows)
    intervals = [(b - a) / 1000 for a, b in zip(times, times[1:])]
    # The last sample receives zero duration. Gaps >1 s are explicitly excluded.
    dwell = collections.Counter()
    gaps = []
    for r, delta in zip(rows, intervals):
        if 0 < delta <= 1:
            dwell[r["perception_mode"]] += delta
        elif delta > 1:
            gaps.append(delta)
    observed_duration = sum(dwell.values())
    n = len(rows)
    track = [r for r in rows if r["perception_mode"] == "TRACK"]
    result = {
        "samples": n, "start_jst": jst(times[0]), "end_jst": jst(times[-1]),
        "saved_span_s": (times[-1] - times[0]) / 1000,
        "perception_count": dict(counts),
        "perception_sample_percent": {k: 100 * v / n for k, v in counts.items()},
        "observed_interval_s": observed_duration,
        "perception_estimated_dwell_s": dict(dwell),
        "perception_estimated_time_percent": {k: 100 * v / observed_duration for k, v in dwell.items()} if observed_duration else {},
        "excluded_recording_gaps_over_1s": gaps,
        "vehicle_mode_count": dict(collections.Counter(r["vehicle_mode"] for r in rows)),
        "sample_interval_ms": stats([d * 1000 for d in intervals]),
        "track_absolute_image_error_px": stats([abs(r["image_error_px"]) for r in track if finite(r["image_error_px"])]),
        "source_sha256": sorted({r["source_sha256"] for r in rows if r["source_sha256"]}),
        "image_error_scope": "TRACK samples only; control-point image error in transformed pixel coordinates, not physical lateral tracking error",
    }
    for key in ("camera_to_output_ms", "vision_ms", "frame_interval_ms", "plan_to_decision_ms", "encoder_speed_kph", "steering_command_us", "throttle_command_us", "requested_forward_percent", "forward_command_percent"):
        result[key] = stats([r[key] for r in rows])
    result["software_frame_rate_from_median_interval_hz"] = (1000 / result["frame_interval_ms"]["median"] if result["frame_interval_ms"].get("median", 0) else None)
    result["recorded_frame_rate_from_median_interval_hz"] = (1000 / result["sample_interval_ms"]["median"] if result["sample_interval_ms"].get("median", 0) else None)
    return result


def read_sources(base, reviews=REVIEWS):
    sources, raw, skipped, conflicts = [], {}, [], []
    candidates = []
    for review in reviews:
        root = base / review
        for p in sorted(root.glob("*.zip")):
            with zipfile.ZipFile(p) as z:
                for member in z.namelist():
                    if member.endswith("frames.jsonl"):
                        candidates.append((p, member, z.read(member)))
        for p in sorted(root.rglob("frames.jsonl")):
            if "replay" in str(p.relative_to(root)):
                continue
            candidates.append((p, None, p.read_bytes()))
    # Archive originals are processed before metadata aliases in each review.
    for p, member, data in candidates:
        rel = str(p.relative_to(base)) + ("!" + member if member else "")
        match = CAPTURE.search(str(p))
        segment = match.group(0) if match else None
        parsed = [json.loads(line) for line in data.splitlines() if line.strip()]
        if not parsed or "metadata" not in parsed[0] or "capture_time_unix_ms" not in parsed[0]:
            skipped.append(rel)
            continue
        source = {"path": rel, "frames_jsonl_sha256": sha(data), "rows": len(parsed), "unique_rows_added": 0}
        if member:
            source["archive_sha256"] = sha(p.read_bytes())
        sources.append(source)
        for i, row in enumerate(parsed, 1):
            key = (row["capture_time_unix_ms"], row["metadata"].get("frame_index"))
            if key in raw:
                # Recording encode/queue metadata may be recomputed independently;
                # compare the raw captured observation and control metadata only.
                before = raw[key]
                if row["metadata"] != before["metadata"]:
                    conflicts.append({"first": before["_source"], "alias": rel, "line": i})
                continue
            if segment is None:
                # Standalone JSONL with no capture identifier is retained under its
                # review name, unless its rows were already in an archive.
                seg = "unlabelled:" + str(p.parent.relative_to(base))
            else:
                seg = segment
            row.update(_source=rel, _line=i, _segment=seg)
            raw[key] = row
            source["unique_rows_added"] += 1
    rows = [reduced(r) for r in raw.values()]
    grouped = collections.defaultdict(list)
    for r in rows:
        match = CAPTURE.search(r["segment"])
        session = match.group(1) if match else r["segment"]
        grouped[session].append(r)
    for values in grouped.values():
        values.sort(key=lambda r: r["time_unix_ms"])
    return sources, grouped, skipped, conflicts


def csv_auto_interval(path):
    with path.open() as f:
        rows = list(csv.DictReader(f))
    intervals, active = [], []
    for i, r in enumerate(rows):
        if r.get("enabled") == "true" and r.get("manual_mode") == "false":
            active.append((i + 2, r))
        elif active:
            intervals.append((active, (i + 2, r)))
            active = []
    if active:
        intervals.append((active, None))
    seq, after = max(intervals, key=lambda item: float(item[0][-1][1]["time"]) - float(item[0][0][1]["time"]))
    first_line, first = seq[0]
    last_line, last = seq[-1]
    return {
        "path": str(path), "sha256": sha(path.read_bytes()), "csv_rows": len(rows),
        "main_auto_rows": len(seq), "first_auto_line": first_line,
        "first_auto_jst": jst(float(first["time"]) * 1000),
        "last_auto_line": last_line, "last_auto_jst": jst(float(last["time"]) * 1000),
        "first_non_auto_after_line": after[0] if after else None,
        "first_non_auto_after_jst": jst(float(after[1]["time"]) * 1000) if after else None,
        "first_auto_to_first_non_auto_s": float(after[1]["time"]) - float(first["time"]) if after else None,
        "sampling_note": "Decimated CSV; first enabled=true is not the exact physical start time.",
    }


def write_csv(path, rows):
    if not rows:
        return
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def saved_software_benchmarks(base):
    """Summarize already-run Pi pipe/fake-sink benchmarks, not live hardware."""
    root = base / "manual_event_latency_review_20261115"
    results = []
    sources = []
    for condition, suffix in (("no_perception_load", ""), ("perception_load", "_load")):
        for version in ("before", "after"):
            path = root / ("benchmark_" + version + suffix + "_pi.json")
            if not path.exists():
                continue
            item = json.loads(path.read_text())
            assert item["actual_radio_or_gpio_measured"] is False
            sources.append({"path": str(path.relative_to(base)), "sha256": sha(path.read_bytes())})
            p = item["pipe_send_to_fake_output_ms"]
            a = item["accepted_input_to_fake_output_ms"]
            results.append(dict(condition=condition, version=version, samples=p["count"],
                                pipe_to_fake_output_median_ms=p["median"],
                                pipe_to_fake_output_p95_ms=p["p95"],
                                accepted_to_fake_output_median_ms=a["median"],
                                accepted_to_fake_output_p95_ms=a["p95"],
                                actual_radio_or_gpio_measured=False))
    return {"scope": "Previously performed Pi software-only evdev pipe to fake-output benchmark; no Bluetooth air time, actual GPIO/PWM or mechanics included", "sources": sources, "rows": results}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--out", type=Path)
    parser.add_argument("--reviews", nargs="+", default=list(REVIEWS),
                        help="Review directories below --base; source archives and metadata only")
    parser.add_argument("--inventory-only", action="store_true",
                        help="Generic session metrics, without this event's manually annotated qualifying case")
    args = parser.parse_args()
    base = args.base.resolve()
    out = (args.out or base / "reports" / "data").resolve()
    out.mkdir(parents=True, exist_ok=True)
    sources, grouped, skipped, conflicts = read_sources(base, args.reviews)
    summaries = {k: summarize(v) for k, v in sorted(grouped.items())}
    if args.inventory_only:
        report = {"schema": "autonomous-race-run-metrics-v1", "base": str(base),
                  "scope": "Generic retained-session inventory; no competition-stage or contact annotation inferred",
                  "sources": sources, "skipped_sources": skipped,
                  "duplicate_metadata_conflicts": conflicts, "sessions": summaries}
        (out / "run_metrics.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        table = [{"session": k, "samples": s["samples"], "saved_span_s": s["saved_span_s"],
                  "start_jst": s["start_jst"], "track_sample_percent": s["perception_sample_percent"].get("TRACK", 0),
                  "track_estimated_time_percent": s["perception_estimated_time_percent"].get("TRACK", 0)} for k, s in summaries.items()]
        write_csv(out / "session_metrics.csv", table)
        print(json.dumps({"sessions": len(summaries), "samples": sum(s["samples"] for s in summaries.values()),
                          "out": str(out), "stage_inferred": False}))
        return
    qrows = grouped[QUALIFYING]
    t0 = qrows[0]["time_unix_ms"]
    for r in qrows:
        r["elapsed_s"] = (r["time_unix_ms"] - t0) / 1000
    by_frame = {(r["segment"].split("-")[-1], r["frame"]): r for r in qrows}
    selected_numbers = [331, 338, 339, 340, 341, 342, 344, 345, 346, 348, 349, 361, 418, 419, 425, 638]
    selected = [dict(by_frame[("s003", f)]) for f in selected_numbers]
    # Count pixels of retained, original masks rather than treating metadata's
    # ratios (which may refer to another mask space) as physical area.
    try:
        from PIL import Image
        for r in selected:
            mask = base / "qualifying_review_20261115_1137" / r["segment"] / "masks" / ("%06d.png" % r["frame"])
            if mask.exists():
                im = Image.open(mask).convert("L")
                r["saved_camera_mask_nonzero_pixels"] = sum(v != 0 for v in im.getdata())
    except ImportError:
        pass
    loss = by_frame[("s003", 341)]
    contact = by_frame[("s003", 346)]
    stationary_end = by_frame[("s003", 418)]
    recovery = by_frame[("s003", 425)]
    first_contact_window = [by_frame[("s003", 344)]["elapsed_s"], contact["elapsed_s"]]
    freeze = [r for r in qrows if contact["time_unix_ms"] <= r["time_unix_ms"] <= stationary_end["time_unix_ms"]]
    assert len(qrows) == 1973
    assert summaries[QUALIFYING]["perception_count"] == {"TRACK": 1866, "LOST": 28, "DEGRADED": 3, "STOP": 76}
    assert {r["encoder_count"] for r in freeze} == {40391}
    assert 221.8 < contact["elapsed_s"] < 222.0
    periods = {}
    for lo, hi, label in [(0, 60, "first60s"), (60, 120, "second60s"), (120, 180, "third60s"), (180, 210, "180to210s"), (contact["elapsed_s"]-60, contact["elapsed_s"], "last60s_before_contact"), (contact["elapsed_s"]-3, contact["elapsed_s"], "last3s_before_contact")]:
        chosen = [r for r in qrows if lo <= r["elapsed_s"] < hi]
        periods[label] = dict(window_start_s=lo, window_end_s=hi, **summarize(chosen))
    bins = []
    for lo in range(0, math.ceil(qrows[-1]["elapsed_s"]), 30):
        chosen = [r for r in qrows if lo <= r["elapsed_s"] < lo + 30]
        summary = summarize(chosen)
        bins.append({"start_s": lo, "end_s": min(lo+30, qrows[-1]["elapsed_s"]), "samples": len(chosen),
                     "track_sample_percent": summary["perception_sample_percent"].get("TRACK", 0),
                     "track_time_percent": summary["perception_estimated_time_percent"].get("TRACK", 0),
                     "track_abs_image_error_p95_px": summary["track_absolute_image_error_px"].get("p95"),
                     "wheel_speed_median_kph": summary["encoder_speed_kph"].get("median"),
                     "camera_to_output_median_ms": summary["camera_to_output_ms"].get("median"),
                     "camera_to_output_p95_ms": summary["camera_to_output_ms"].get("p95")})
    report = {
        "schema": "autonomous-race-run-metrics-v1", "base": str(base),
        "method": {"sample_percentage": "state sample count / retained samples * 100", "time_percentage": "left endpoint state held until next retained sample, excluding gaps >1 s; last sample has zero duration", "percentile": "linear interpolation at (n-1)*p", "deduplication": "capture_time_unix_ms + metadata.frame_index; raw archives and metadata aliases not counted twice", "scope": "Saved captures from selected review directories, not all real trials; replay outputs excluded"},
        "sources": sources, "skipped_sources": skipped, "duplicate_metadata_conflicts": conflicts,
        "sessions": summaries,
        "saved_software_latency_benchmark": saved_software_benchmarks(base),
        "qualifying_detail": {
            "session": QUALIFYING, "stage": "qualifying (予選), not final tournament",
            "first_saved_auto_jst": qrows[0]["time_jst"], "csv_auto": csv_auto_interval(base / "qualifying_review_20261115_1137" / "run_control_log.csv"),
            "visual_contact": {"selected_prior_visual_evidence": "qualifying_review_20261115_1137/macro_visual_audit.json", "visual_confirmation_image": "qualifying_review_20261115_1137/vision_contact_confirmation.jpg", "inferred_contact_elapsed_window_s": first_contact_window, "barrier_fills_view_by_elapsed_s": contact["elapsed_s"], "barrier_frame": 346, "exact_force_sensor_contact_time": None},
            "recognition_loss_elapsed_s": loss["elapsed_s"],
            "count_freeze": {"start_elapsed_s": contact["elapsed_s"], "end_elapsed_s": stationary_end["elapsed_s"], "duration_s": (stationary_end["time_unix_ms"] - contact["time_unix_ms"])/1000, "samples": len(freeze), "count": 40391},
            "track_recovery_elapsed_s": recovery["elapsed_s"], "continuous_non_track_until_recovery_s": (recovery["time_unix_ms"] - loss["time_unix_ms"])/1000,
            "selected_frames": selected, "periods": periods, "bins30s": bins,
            "source_visual_audit_scope": "This script recomputes metadata and timing; visual contact annotation relies on retained images and explicitly identified prior visual audit, not a computed collision detector.",
        },
        "unavailable_evaluations": ["final tournament run-to-log correspondence", "independent world-frame trajectory or lateral tracking error", "measured steering angle", "contact force or exact collision instant", "exact laps / official lap time from these captures", "trial denominator for run success rate", "actuator mechanical response latency", "one-way radio latency or radio interference cause of contact"],
    }
    (out / "run_metrics.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    write_csv(out / "qualifying_timeline.csv", qrows)
    write_csv(out / "qualifying_30s_bins.csv", bins)
    write_csv(out / "manual_software_latency_benchmark.csv", report["saved_software_latency_benchmark"]["rows"])
    compact = []
    for session, s in summaries.items():
        compact.append({"session": session, "samples": s["samples"], "saved_span_s": s["saved_span_s"], "start_jst": s["start_jst"], "track_sample_percent": s["perception_sample_percent"].get("TRACK",0), "track_estimated_time_percent": s["perception_estimated_time_percent"].get("TRACK",0), "camera_to_output_median_ms": s["camera_to_output_ms"].get("median"), "camera_to_output_p95_ms": s["camera_to_output_ms"].get("p95"), "source_sha256": ";".join(s["source_sha256"])})
    write_csv(out / "session_metrics.csv", compact)
    print(json.dumps({"sessions": len(summaries), "unique_samples": sum(s["samples"] for s in summaries.values()), "source_aliases": len(sources), "duplicate_conflicts": len(conflicts), "qualifying": summaries[QUALIFYING], "contact_elapsed_s": contact["elapsed_s"], "out": str(out)}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
