#!/usr/bin/env python3
"""Verify the frozen report against retained CSV/JSON, source hashes and PDF text.

Standard library only. No vehicle, network, package installer or source mutation.
The receipt distinguishes cached consistency from raw recalculation and visual QA.
"""
from __future__ import annotations

import argparse
import ast
from collections import Counter, defaultdict
import csv
from datetime import datetime, timezone
from decimal import Decimal, ROUND_HALF_UP
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys
import unicodedata


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def relative(root, value):
    path = (root / value).resolve()
    require(path.is_relative_to(root), "Report path escapes its root: " + value)
    return path


def load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def table(path):
    with Path(path).open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def percentile(values, p):
    values = sorted(values)
    require(bool(values), "No values for percentile")
    x = (len(values) - 1) * p
    a, b = math.floor(x), math.ceil(x)
    return values[a] if a == b else values[a] * (b - x) + values[b] * (x - a)


def near(a, b, label, tolerance=1e-7):
    require(math.isfinite(float(a)) and math.isfinite(float(b)), label + " is not finite")
    require(math.isclose(float(a), float(b), abs_tol=tolerance, rel_tol=1e-9),
            f"{label}: recalculated {a}, stored {b}")


def values(rows, key, absolute=False):
    result = [float(r[key]) for r in rows if r.get(key) not in (None, "")]
    require(all(math.isfinite(v) for v in result), "Nonfinite CSV value: " + key)
    return [abs(v) for v in result] if absolute else result


def check_stats(observed, stored, label):
    require(len(observed) == stored["count"], label + " count differs")
    if not observed:
        return
    for key, val in {"minimum": min(observed), "maximum": max(observed),
                     "mean": sum(observed) / len(observed), "median": percentile(observed, .5),
                     "p95": percentile(observed, .95)}.items():
        near(val, stored[key], label + "." + key)


def dwell(rows):
    result = defaultdict(float)
    gaps = []
    for a, b in zip(rows, rows[1:]):
        delta = (float(b["time_unix_ms"]) - float(a["time_unix_ms"])) / 1000
        if 0 < delta <= 1:
            result[a["perception_mode"]] += delta
        elif delta > 1:
            gaps.append(delta)
    return dict(result), gaps


def display(value, digits=2):
    return str(Decimal(str(value)).quantize(Decimal(1).scaleb(-digits), rounding=ROUND_HALF_UP))


def find_source_code(root, cfg, explicit):
    candidates = []
    if explicit:
        supplied = Path(explicit).resolve()
        candidates += [supplied / "controller_hud_candidate", supplied / "source/system", supplied]
    else:
        candidates += [(root / p).resolve() for p in cfg["source_code"]["relative_candidate_dirs"]]
    for path in candidates:
        if (path / "robust_lane_failsafe.py").is_file():
            return path
    raise ValueError("Qualifying source snapshot missing. Include package source/system or supply --source-root. No absolute historical path was used.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reports", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--config", type=Path)
    parser.add_argument("--source-root", type=Path, help="Preserved raw/source root, or the qualifying source snapshot directory")
    parser.add_argument("--skip-pdf", action="store_true", help="Verify data, text and figures during PDF authoring; do not attest the PDF")
    parser.add_argument("--mode", choices=("cached", "raw"), default="cached", help="Receipt label; raw calculation itself is performed by rebuild.py")
    parser.add_argument("--out", type=Path, help="Alternative verification receipt path")
    args = parser.parse_args()
    root = args.reports.resolve()
    config_path = (args.config or root / "report_config.json").resolve()
    cfg = load(config_path)
    checks, warnings = [], []

    def check(name, fn):
        try:
            detail = fn()
            checks.append({"name": name, "status": "passed", "detail": detail})
        except Exception as exc:
            checks.append({"name": name, "status": "failed", "detail": str(exc)})

    paths = {key: relative(root, path) for key, path in cfg["paths"].items()}
    metrics = load(paths["metrics"])
    rows = table(paths["timeline"])
    detail = metrics["qualifying_detail"]
    summary = metrics["sessions"][cfg["event"]["qualifying_session"]]
    expected = cfg["expected"]
    paper = paths["paper"].read_text(encoding="utf-8")
    body = paper.split("<!-- REFERENCES:GENERATED -->")[0]

    def config_check():
        require(cfg["schema"] == "autonomous-race-report-build-v1", "Unsupported build config")
        require(cfg["event"]["date_jst"] == "2026-11-15", "Frozen report date must remain the user-specified 2026-11-15")
        require(cfg["event"]["timezone"] == "Asia/Tokyo", "Event timezone mismatch")
        for rel in cfg["cached_inputs"]:
            require(relative(root, rel).is_file(), "Missing cached input: " + rel)
        return {"date_jst": cfg["event"]["date_jst"], "cached_inputs": len(cfg["cached_inputs"])}

    def run_check():
        require(len(rows) == summary["samples"] == expected["qualifying_samples"], "Qualifying sample count mismatch")
        require(len(metrics["sessions"]) == expected["sessions"], "Session count mismatch")
        require(sum(s["samples"] for s in metrics["sessions"].values()) == expected["retained_observations"], "Retained observation total mismatch")
        require(not metrics["duplicate_metadata_conflicts"], "Conflicting duplicated metadata")
        require(detail["session"] == cfg["event"]["qualifying_session"], "Wrong qualifying session")
        require({r["vehicle_mode"] for r in rows} == {"AUTO"}, "Non-AUTO vehicle modes in the detailed interval")
        times = values(rows, "time_unix_ms")
        require(all(b > a for a, b in zip(times, times[1:])), "Timeline is not strictly chronological")
        require(len({(r["time_unix_ms"], r["frame_index"]) for r in rows}) == len(rows), "Duplicated retained observation")
        near((times[-1] - times[0]) / 1000, summary["saved_span_s"], "Saved span")
        for row in rows:
            near((float(row["time_unix_ms"]) - times[0]) / 1000, row["elapsed_s"], "Elapsed time")
        counts = dict(Counter(r["perception_mode"] for r in rows))
        require(counts == summary["perception_count"] == expected["perception_counts"], "Perception counts differ")
        held, gaps = dwell(rows)
        near(sum(held.values()), summary["observed_interval_s"], "Observed time denominator")
        require(len(gaps) == len(summary["excluded_recording_gaps_over_1s"]), "Recording gap count differs")
        for state, count in counts.items():
            near(100 * count / len(rows), summary["perception_sample_percent"][state], "Sample percentage " + state)
            near(held[state], summary["perception_estimated_dwell_s"][state], "State duration " + state)
            near(100 * held[state] / sum(held.values()), summary["perception_estimated_time_percent"][state], "Time percentage " + state)
        for key in ["camera_to_output_ms", "vision_ms", "frame_interval_ms", "plan_to_decision_ms", "encoder_speed_kph", "steering_command_us", "throttle_command_us", "requested_forward_percent", "forward_command_percent"]:
            check_stats(values(rows, key), summary[key], key)
        track = [r for r in rows if r["perception_mode"] == "TRACK"]
        check_stats(values(track, "image_error_px", True), summary["track_absolute_image_error_px"], "TRACK image error")
        check_stats([b-a for a,b in zip(times, times[1:])], summary["sample_interval_ms"], "Saved frame interval")
        require({r["source_sha256"] for r in rows} == {cfg["source_code"]["main_sha256"]}, "Runtime source SHA differs")
        return {"samples": len(rows), "sessions": len(metrics["sessions"]), "state_counts": counts,
                "track_sample_percent": summary["perception_sample_percent"]["TRACK"],
                "track_estimated_time_percent": summary["perception_estimated_time_percent"]["TRACK"],
                "camera_to_output_median_ms": summary["camera_to_output_ms"]["median"],
                "camera_to_output_p95_ms": summary["camera_to_output_ms"]["p95"],
                "scope": "Recomputed from retained CSV; not a raw-capture or physical response measurement"}

    def segments_check():
        bins = table(paths["macro_bins"])
        require(len(bins) == len(detail["bins30s"]), "Macro bin count differs")
        for row, stored in zip(bins, detail["bins30s"]):
            for key, value in stored.items():
                near(row[key], value, "Macro CSV/JSON " + key)
            start = float(row["start_s"])
            chosen = [r for r in rows if start <= float(r["elapsed_s"]) < start + 30]
            require(len(chosen) == int(row["samples"]), "Macro sample count differs")
            near(100 * sum(r["perception_mode"] == "TRACK" for r in chosen) / len(chosen), row["track_sample_percent"], "Macro sample rate")
            held, _ = dwell(chosen)
            near(100 * held.get("TRACK", 0) / sum(held.values()), row["track_time_percent"], "Macro time rate")
            near(percentile(values(chosen, "camera_to_output_ms"), .95), row["camera_to_output_p95_ms"], "Macro p95")
        selected = {(r["segment"], int(r["frame"])): r for r in rows}
        for stored in detail["selected_frames"]:
            row = selected[(stored["segment"], int(stored["frame"]))]
            for key in ["time_unix_ms", "elapsed_s", "steering_command_us", "forward_command_percent", "encoder_count"]:
                near(row[key], stored[key], "Selected frame " + key)
            require(row["perception_mode"] == stored["perception_mode"], "Selected perception state differs")
        freeze = detail["count_freeze"]
        chosen = [r for r in rows if freeze["start_elapsed_s"] <= float(r["elapsed_s"]) <= freeze["end_elapsed_s"]]
        require(len(chosen) == freeze["samples"], "Encoder freeze sample count differs")
        require({int(r["encoder_count"]) for r in chosen} == {freeze["count"]}, "Encoder freeze is not constant")
        near(float(chosen[-1]["elapsed_s"]) - float(chosen[0]["elapsed_s"]), freeze["duration_s"], "Freeze duration")
        require(detail["visual_contact"]["exact_force_sensor_contact_time"] is None, "Unmeasured physical collision instant filled in")
        return {"macro_bins": len(bins), "selected_frames": len(detail["selected_frames"]), "freeze_samples": len(chosen)}

    def benchmark_check():
        observed = table(paths["software_benchmark"])
        stored = metrics["saved_software_latency_benchmark"]["rows"]
        require(len(observed) == len(stored) == 4, "Expected four saved benchmark conditions")
        for row, item in zip(observed, stored):
            require(row["condition"] == item["condition"] and row["version"] == item["version"], "Benchmark condition differs")
            require(int(row["samples"]) == item["samples"] == 200, "Benchmark event count differs")
            require(row["actual_radio_or_gpio_measured"] == "False" and item["actual_radio_or_gpio_measured"] is False, "Software benchmark scope changed")
            for key in ["pipe_to_fake_output_median_ms", "pipe_to_fake_output_p95_ms", "accepted_to_fake_output_median_ms", "accepted_to_fake_output_p95_ms"]:
                near(row[key], item[key], "Software benchmark " + key)
        return {"conditions": 4, "samples_per_condition": 200, "radio_gpio_measured": False}

    def official_check():
        official = load(paths["official_results"])
        require(official["team_name"] == cfg["event"]["team"], "Official team differs")
        require(official["combined_MAGP_RR_qualifying_rank"] == expected["official_qualifying_rank"], "Qualifying rank differs")
        for no in (1, 2):
            q = official[f"qualifying{no}"]
            require(q["safe_laps"] == expected[f"official_q{no}_laps"], "Official lap count differs")
            near(q["best_lap_s"], expected[f"official_q{no}_best_lap_s"], "Official lap time")
            pattern = rf"\|予選{no}\|\s*{q['safe_laps']}周\|\s*{re.escape(display(q['best_lap_s']))}秒\|"
            require(re.search(pattern, paper), "Paper official result row differs: Q" + str(no))
        final = official["final_tournament"]
        require(final["official_final_overall_rank"] is None and final["official_round1_score"] is None, "Unverified final data filled in")
        require(final["qualified"] is True and "参加者" in final["result_source"], "Final result provenance differs")
        require("予選順位" in body and "参加者本人の報告" in body, "Rank/final provenance not distinguished in paper")
        return {"team": official["team_name"], "qualifying_rank": official["combined_MAGP_RR_qualifying_rank"], "final_round1_provenance": final["result_source"], "official_final_overall_rank": None}

    def source_check():
        evidence = load(paths["code_evidence"])
        entries = evidence["source_files"]
        require(len(entries) == cfg["source_code"]["expected_files"], "Qualifying source count differs")
        source = find_source_code(root, cfg, args.source_root)
        verified = {}
        for entry in entries:
            name = Path(entry["path"])
            require(not name.is_absolute() and ".." not in name.parts and name.parts[0] == "controller_hud_candidate", "Unexpected source ledger path")
            actual = source.joinpath(*name.parts[1:])
            require(actual.is_file(), "Missing qualifying source: " + str(actual))
            h = sha(actual)
            require(h == entry["sha256"] == entry["manifest_sha256"] and entry["matches_manifest"] is True, "Qualifying source SHA differs: " + str(name))
            verified[entry["path"]] = h
        build_groups = 0
        for recorded in evidence["qualifying_build_matches"]:
            for group in recorded["distinct_builds"]:
                matches = group["hash_matches"]
                require(len(matches) == cfg["source_code"]["expected_recorded_build_hashes"], "Recorded hash count differs from the 25-file source inventory")
                for key, info in matches.items():
                    require(group["build"][key] == verified[info["file"]] and info["matches"] is True, "Runtime build hash mismatch: " + key)
                build_groups += 1
        require(build_groups == cfg["source_code"]["expected_recorded_build_groups"], "Recorded build group count differs")
        return {"source_directory": str(source), "actual_files_rehashed": len(verified), "recorded_hashes_per_build": cfg["source_code"]["expected_recorded_build_hashes"], "build_groups": build_groups}

    def radio_check():
        radio = load(paths["radio_summary"])
        minutes = radio["minutes"]
        require(radio["date_jst"] == cfg["event"]["date_jst"], "Radio date differs")
        require(sum(m["pico_scan_count"] for m in minutes) == radio["coverage"]["pico"]["records"] == expected["pico_scans"], "Pico count differs")
        require(sum(m["pi_diagnostic_samples"] for m in minutes) == radio["coverage"]["pi"]["records"] == expected["pi_radio_snapshots"], "Pi diagnostic count differs")
        populated = [m["minute_of_day_jst"] for m in minutes if m["pi_diagnostic_samples"]]
        missing = max(populated) - min(populated) + 1 - len(populated)
        require(missing == radio["coverage"]["pi"]["missing_minutes_between_first_and_last"] == expected["pi_missing_minutes"], "Pi missing minutes differ")
        rtt = [(m["pi_metrics"]["gateway_rtt_ms"]["mean"], m["minute_of_day_jst"]) for m in minutes if m["pi_metrics"]["gateway_rtt_ms"]["mean"] is not None]
        near(max(rtt)[0], radio["pi_metric_aggregates"]["gateway_rtt_ms"]["minute_mean_max"], "RTT minute maximum")
        pairs = [(m["pico_ap_count"]["mean"], m["pi_metrics"]["gateway_rtt_ms"]["mean"]) for m in minutes if m["pico_ap_count"] and m["pi_metrics"]["gateway_rtt_ms"]["mean"] is not None]
        mx = sum(p[0] for p in pairs) / len(pairs); my = sum(p[1] for p in pairs) / len(pairs)
        corr = sum((x-mx)*(y-my) for x,y in pairs) / math.sqrt(sum((x-mx)**2 for x,y in pairs) * sum((y-my)**2 for x,y in pairs))
        require(len(pairs) == 152 and display(corr, 3) == "0.135", "Exploratory minute correlation differs")
        return {"pico_scans": expected["pico_scans"], "pi_cached_snapshots": expected["pi_radio_snapshots"], "pi_missing_minutes": missing, "matched_minutes": len(pairs), "exploratory_correlation": corr, "independent_radio_trials": False}

    def citation_check():
        refs = load(paths["reference_index"])
        require(len(refs) == expected["reference_count"], "Reference index length differs")
        section = paper.split("<!-- REFERENCES:GENERATED -->", 1)[1].split("## 付録A", 1)[0]
        nums = [int(n) for n in re.findall(r"^\[(\d+)\]", section, re.M)]
        require(nums == list(range(1, len(refs)+1)), "Reference numbering has gaps or duplicates")
        bib = paths["bibliography"].read_text()
        keys = re.findall(r"^@[a-zA-Z]+\{([^,]+),", bib, re.M)
        require(keys == [r["bib_key"] for r in refs] and len(set(keys)) == len(refs), "BibTeX keys/order differ")
        for ref in refs:
            require(ref["url"].startswith("https://") and ref["url"] in section and ref["url"] in bib, "Reference URL absent or invalid")
        cited = set()
        for token in re.findall(r"\[([0-9]+(?:\s*[,、\-–]\s*[0-9]+)*)\]", body):
            for item in re.split(r"[,、]", token):
                ends = re.split(r"[\-–]", item)
                cited.update(range(int(ends[0]), int(ends[-1])+1))
        require(cited == set(nums), "Body citations and bibliography differ")
        return {"references": len(refs), "bibtex_entries": len(keys), "all_citations_resolved": True, "network_recheck_performed": False}

    def paper_check():
        normalized = unicodedata.normalize("NFKC", body)
        for value, label in [(len(rows), "observations"), (len(metrics["sessions"]), "sessions"), (expected["retained_observations"], "retained observations")]:
            require(str(value) in normalized, "Paper missing computed " + label)
        displayed = [display(summary["perception_sample_percent"]["TRACK"]), display(summary["perception_estimated_time_percent"]["TRACK"]), display(summary["camera_to_output_ms"]["median"]), display(summary["camera_to_output_ms"]["p95"]), display(summary["vision_ms"]["median"]), display(summary["vision_ms"]["p95"]), display(summary["frame_interval_ms"]["median"]), display(summary["sample_interval_ms"]["median"]), display(summary["saved_span_s"], 3)]
        for val in displayed:
            require(re.search(r"(?<![\d.])" + re.escape(val) + r"(?![\d.])", normalized), "Paper numeric value differs/missing: " + val)
        require("28/3/76" in normalized.replace("／", "/"), "Paper LOST/DEGRADED/STOP counts differ")
        require(cfg["source_code"]["main_sha256"] in paper, "Paper runtime source identity missing")
        require("25ファイル" in body and "11ハッシュ" in body, "Source inventory and recorded hashes not distinguished")
        require("0.005" in body and "0.30秒" in body, "PD time-clipping condition missing")
        for token in ["知覚状態STOP", "保存された観測", "交絡要因", "実舵角", "車輪換算速度"]:
            require(token in body, "Required measurement limitation missing: " + token)
        return {"checked_display_values": displayed, "pending_confirmation_markers": paper.count("[要確認]"), "scope": "Targeted numeric and limitation checks; no automated proof of all prose claims"}

    def figures_check():
        refs = re.findall(r"!\[[^\]]*\]\(([^)]+)\)", paper)
        require(len(refs) == expected["figure_count"], "Paper figure count differs")
        require({Path(p).stem for p in refs} == set(cfg["figures"]), "Paper figure names differ from generated figure set")
        fm = load(paths["figure_metadata"])
        require(set(fm["figures"]) == set(cfg["figures"]), "Figure metadata set differs")
        for name, entry in fm["figures"].items():
            for ext, info in entry["files"].items():
                path = relative(root, "figures/" + info["path"])
                require(path.is_file() and sha(path) == info["sha256"], "Figure missing or stale: " + str(path))
            require(entry.get("conditions_ja") and entry.get("limitations_ja"), "Figure conditions/units/limits metadata missing")
        for rel in refs:
            require(relative(root, rel).is_file(), "Paper image missing: " + rel)
        for name, h in fm["sources"].items():
            require(sha(relative(root, "data/" + name)) == h, "Figure generated from another data version: " + name)
        require(sha(root / "system_analysis.md") == fm["system_analysis_sha256"], "Architecture figure source document changed")
        manifest = load(root / "data/contact_frames/source_manifest.json")
        source_images = {i["frame"]: i for i in manifest["images"]}
        for image in cfg["contact_images"]:
            actual = relative(root, image["path"])
            require(sha(actual) == image["sha256"] == source_images[image["frame"]]["sha256"], "Contact JPEG changed")
        return {"figures": len(refs), "figure_files_rehashed": sum(len(e["files"]) for e in fm["figures"].values()), "contact_jpegs_rehashed": len(cfg["contact_images"])}

    def cli_check():
        needed = {"rebuild.py": {"--mode", "--source-root", "--reports", "--config", "--skip-figures", "--skip-pdf"}, "verify_report.py": {"--reports", "--config", "--source-root", "--skip-pdf"}, "analyze_runs.py": {"--base", "--reviews", "--inventory-only", "--out"}, "make_figures.py": {"--reports", "--out"}, "build_pdf.py": {"--reports"}, "update_references.py": {"--reports"}, "extract_radio_summary.py": {"--base", "--out"}}
        checked = {}
        for name, flags in needed.items():
            script = root / "scripts" / name
            require(script.is_file(), "Documented script missing: " + name)
            parsed = ast.parse(script.read_text(), filename=str(script))
            declared = {arg.value for n in ast.walk(parsed) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == "add_argument" for arg in n.args if isinstance(arg, ast.Constant) and isinstance(arg.value, str)}
            require(flags <= declared, "Documented but unimplemented CLI in " + name + ": " + str(flags-declared))
            checked[name] = sorted(flags)
        return {"syntax_and_declared_cli": checked, "vehicle_code_imported": False}

    def pdf_check():
        if args.skip_pdf:
            return {"status": "explicitly_skipped", "reason": "--skip-pdf: no claim that PDF or visual QA passed"}
        pdf = paths["pdf"]
        require(pdf.is_file() and pdf.stat().st_size > 10000, "PDF missing or empty")
        def tool(name):
            system = Path("/usr/bin") / name
            return str(system) if system.is_file() else shutil.which(name)
        info, texttool = tool("pdfinfo"), tool("pdftotext")
        require(info and texttool, "PDF inspection requires existing system pdfinfo/pdftotext; no package was installed")
        proc = subprocess.run([info, str(pdf)], text=True, capture_output=True, timeout=30)
        require(proc.returncode == 0, "pdfinfo failed: " + proc.stderr[-300:])
        match = re.search(r"^Pages:\s*(\d+)", proc.stdout, re.M)
        require(match is not None, "PDF page count unavailable")
        pages = int(match.group(1))
        require(pages >= cfg["verification"]["pdf_minimum_pages"], "PDF page count unexpectedly low")
        proc = subprocess.run([texttool, "-layout", str(pdf), "-"], text=True, capture_output=True, timeout=30)
        require(proc.returncode == 0, "pdftotext failed: " + proc.stderr[-300:])
        text = proc.stdout
        require(len(text) >= cfg["verification"]["pdf_minimum_text_characters"], "PDF searchable text unexpectedly short")
        require("\ufffd" not in text, "Replacement glyph in extracted PDF text")
        compact = re.sub(r"\s+", "", unicodedata.normalize("NFKC", text))
        for number in ["1973", "94.58", "60.69", "78.23", "30.88", "25.23"]:
            require(number in compact, "PDF does not contain required result: " + number)
        visual = paths["pdf_visual_review_receipt"]
        visual_status = "not_recorded_or_outdated"
        if visual.is_file():
            receipt = load(visual)
            if receipt.get("pdf_sha256") == sha(pdf) and set(receipt.get("reviewed_pages", [])) == set(range(1, pages+1)) and not receipt.get("open_layout_issues"):
                visual_status = "matching_independent_visual_review_receipt"
        if visual_status != "matching_independent_visual_review_receipt":
            warnings.append("PDF numeric/text checks passed, but a matching all-page visual-review receipt was not found. Root visual inspection remains separate.")
        return {"pages": pages, "searchable_text_characters": len(text), "sha256": sha(pdf), "visual_review": visual_status, "pdfinfo": info, "pdftotext": texttool}

    for name, fn in [("configuration_and_inputs", config_check), ("retained_csv_metrics", run_check), ("macro_and_contact_timing", segments_check), ("software_benchmark", benchmark_check), ("official_result_provenance", official_check), ("qualifying_source_and_runtime_hashes", source_check), ("radio_aggregate_scope", radio_check), ("references_and_citations", citation_check), ("paper_numeric_claims", paper_check), ("figures_and_contact_images", figures_check), ("implemented_scripts_and_cli", cli_check), ("pdf_structure_and_text", pdf_check)]:
        check(name, fn)
    failed = [c for c in checks if c["status"] == "failed"]
    receipt = {"schema": "autonomous-race-report-verification-v1", "created_utc": datetime.now(timezone.utc).isoformat(), "event_date_jst": cfg["event"]["date_jst"], "reports": str(root), "requested_mode": args.mode, "verification_scope": "Cached CSV/JSON/paper/source/figure consistency. Raw recomputation is attested separately by rebuild's executed steps; no physical experiment or automatic visual verdict.", "config_sha256": sha(config_path), "paper_sha256": sha(paths["paper"]), "status": "failed" if failed else "passed", "checks": checks, "warnings": warnings}
    out = (args.out or paths["verification_receipt"]).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(receipt, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"status": receipt["status"], "checks_passed": len(checks)-len(failed), "checks_failed": len(failed), "failures": failed, "warnings": warnings, "receipt": str(out)}, ensure_ascii=False))
    raise SystemExit(1 if failed else 0)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError) as error:
        print(json.dumps({"status": "failed_preflight", "error": str(error)}, ensure_ascii=False), file=sys.stderr)
        raise SystemExit(2)
