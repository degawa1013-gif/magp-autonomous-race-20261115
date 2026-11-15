#!/usr/bin/env python3
"""Rebuild the frozen MAGP paper offline; install nothing and never access a vehicle.

cached: use packaged analysis data, 3 contact JPEGs and the qualifying source snapshot.
raw: require explicit original evidence root; recalculate retained run metrics and
extract the frozen radio aggregate before the same figure/reference/PDF pipeline.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def local_path(root, value):
    path = (root / value).resolve()
    if not path.is_relative_to(root):
        raise ValueError("Configured report path escapes its root: " + value)
    return path


def python_for(cfg, role, modules):
    setting = cfg.get("python", {}).get(role, "auto")
    candidates = [setting] if setting != "auto" else [sys.executable, "/usr/bin/python3", shutil.which("python3")]
    checked = set()
    for candidate in candidates:
        if not candidate:
            continue
        candidate = shutil.which(candidate) or candidate
        if candidate in checked:
            continue
        checked.add(candidate)
        try:
            probe = subprocess.run([candidate, "-c", "import importlib.util,sys; sys.exit(0 if all(importlib.util.find_spec(x) is not None for x in sys.argv[1:]) else 1)", *modules], capture_output=True, text=True, timeout=15)
        except (OSError, subprocess.TimeoutExpired):
            continue
        if probe.returncode == 0:
            return candidate
    raise ValueError(f"No existing Python for {role} with {modules}. Checked {sorted(checked)}. Choose a compatible existing interpreter in report_config.json; no installation was performed.")


def preflight_raw(source, cfg):
    if source is None:
        raise ValueError("--mode raw requires --source-root pointing to preserved original evidence. Cached summaries are not raw inputs.")
    if not source.is_dir():
        raise ValueError("Raw source root is not an accessible directory: " + str(source))
    missing = []
    for review in cfg["raw"]["reviews"]:
        path = source / review
        if not path.is_dir():
            missing.append(review + "/")
        elif not list(path.glob("*.zip")) and next(path.rglob("frames.jsonl"), None) is None:
            missing.append(review + "/ (no original ZIP or frames.jsonl)")
    for rel in cfg["raw"]["required_files"] + cfg["raw"]["radio_required_files"]:
        if not (source / rel).is_file():
            missing.append(rel)
    if missing:
        raise ValueError("Raw recreation cannot proceed; missing original evidence:\n" + "\n".join(missing) + "\nUse --mode cached to regenerate from the packaged retained aggregates; it does not recompute raw evidence.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reports", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--config", type=Path, help="Default: report_config.json inside the selected reports directory")
    parser.add_argument("--mode", choices=("cached", "raw"), default="cached")
    parser.add_argument("--source-root", type=Path, help="Required original evidence root for raw; optional explicit source hash location for cached")
    parser.add_argument("--skip-figures", action="store_true", help="Reuse existing verified figures while iterating the manuscript/PDF")
    parser.add_argument("--skip-pdf", action="store_true", help="Regenerate data/figures/references and verify them during PDF authoring")
    parser.add_argument("--dry-run", action="store_true", help="Validate inputs/interpreters and print the plan without writing outputs")
    args = parser.parse_args()
    root = args.reports.resolve()
    config_path = (args.config or root / "report_config.json").resolve()
    cfg = json.loads(config_path.read_text(encoding="utf-8"))
    if cfg.get("schema") != "autonomous-race-report-build-v1":
        raise ValueError("Unsupported report config schema")
    if cfg["event"]["date_jst"] != "2026-11-15":
        raise ValueError("This frozen-event pipeline uses the client-specified date 2026-11-15. Create a separate event config/adapter for another competition.")
    source = args.source_root.resolve() if args.source_root else None
    if args.mode == "raw":
        preflight_raw(source, cfg)
        raw_outputs = {"data/run_metrics.json", "data/qualifying_timeline.csv", "data/qualifying_30s_bins.csv", "data/manual_software_latency_benchmark.csv", "data/radio_event_summary.json"}
    else:
        raw_outputs = set()
    for rel in cfg["cached_inputs"]:
        if rel not in raw_outputs and not local_path(root, rel).is_file():
            raise ValueError("Missing packaged input: " + rel)
    core = python_for(cfg, "core", [])
    steps = []
    if args.mode == "raw":
        steps += [
            ("raw_run_recalculation", [core, str(root / cfg["raw"]["run_script"]), "--base", str(source), "--reviews", *cfg["raw"]["reviews"], "--out", str(root / "data")]),
            ("frozen_radio_aggregate_extraction", [core, str(root / cfg["raw"]["radio_script"]), "--base", str(source), "--out", str(local_path(root, cfg["paths"]["radio_summary"]))]),
        ]
    if not args.skip_figures:
        plotting = python_for(cfg, "figures", ["numpy", "matplotlib", "PIL"])
        steps.append(("figures_from_retained_data", [plotting, str(root / "scripts/make_figures.py"), "--reports", str(root)]))
    steps.append(("verified_reference_rendering", [core, str(root / "scripts/update_references.py"), "--reports", str(root)]))
    if not args.skip_pdf:
        pdf_python = python_for(cfg, "pdf", ["reportlab"])
        steps.append(("paper_pdf", [pdf_python, str(root / "scripts/build_pdf.py"), "--reports", str(root)]))
    verification = [core, str(root / "scripts/verify_report.py"), "--reports", str(root), "--config", str(config_path), "--mode", args.mode]
    if source:
        verification += ["--source-root", str(source)]
    if args.skip_pdf:
        verification.append("--skip-pdf")
    steps.append(("verification", verification))
    for name, command in steps:
        if not Path(command[1]).is_file():
            raise ValueError("Planned script is absent: " + command[1])
    plan = {"mode": args.mode, "reports": str(root), "event_date_jst": cfg["event"]["date_jst"], "source_root": str(source) if source else None,
            "raw_recomputed": args.mode == "raw", "radio_recomputation_scope": cfg["raw"]["scope"] if args.mode == "raw" else "Packaged frozen summary; no raw radio calculation",
            "steps": [{"name": n, "command": c} for n, c in steps], "skipped_figures": args.skip_figures, "skipped_pdf": args.skip_pdf}
    if args.dry_run:
        print(json.dumps({"dry_run": True, **plan}, ensure_ascii=False, indent=2))
        return
    receipt_path = local_path(root, cfg["paths"]["build_receipt"])
    receipt_path.parent.mkdir(parents=True, exist_ok=True)
    receipt = {"schema": "autonomous-race-report-build-receipt-v1", "created_utc": datetime.now(timezone.utc).isoformat(), "config_sha256": digest(config_path),
               **{k: v for k, v in plan.items() if k != "steps"}, "steps": [], "status": "running",
               "environment_modified": False, "live_vehicle_access": False, "network_access": False,
               "original_source_modified": False, "visual_review": "Separate root visual-review receipt; not implied by this pipeline"}
    for name, command in steps:
        try:
            result = subprocess.run(command, cwd=root, text=True, capture_output=True, timeout=900)
            record = {"name": name, "command": command, "returncode": result.returncode, "stdout": result.stdout[-12000:], "stderr": result.stderr[-4000:]}
        except (OSError, subprocess.TimeoutExpired) as exc:
            record = {"name": name, "command": command, "returncode": -1, "error": str(exc)}
        receipt["steps"].append(record)
        if record["returncode"] != 0:
            receipt["status"] = "failed"
            receipt_path.write_text(json.dumps(receipt, ensure_ascii=False, indent=2) + "\n")
            print(json.dumps({"status": "failed", "step": name, "detail": record, "receipt": str(receipt_path)}, ensure_ascii=False))
            raise SystemExit(1)
        print(json.dumps({"step": name, "status": "passed"}, ensure_ascii=False), flush=True)
    receipt["status"] = "passed"
    receipt["outputs"] = {}
    for key in ["paper", "metrics", "timeline", "radio_summary", "figure_metadata", "bibliography", "verification_receipt"] + ([] if args.skip_pdf else ["pdf"]):
        path = local_path(root, cfg["paths"][key])
        if path.is_file():
            receipt["outputs"][cfg["paths"][key]] = {"bytes": path.stat().st_size, "sha256": digest(path)}
    receipt_path.write_text(json.dumps(receipt, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"status": "passed", "mode": args.mode, "receipt": str(receipt_path), "raw_recomputed": args.mode == "raw", "pdf_rebuilt": not args.skip_pdf}, ensure_ascii=False))


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError) as error:
        print(json.dumps({"status": "failed_preflight", "error": str(error)}, ensure_ascii=False), file=sys.stderr)
        raise SystemExit(2)
