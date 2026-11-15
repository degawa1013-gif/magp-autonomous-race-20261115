"""Finalize one sealed diagnostic segment in an isolated Python process.

Only standard-library archive/filesystem work and the pure diagnostic analyzer
are imported. This helper never imports the driving program, camera or GPIO.
"""
import argparse
import csv
import io
import json
import math
import os
from pathlib import Path
import signal
import sys
import zipfile

from drive_capture_analysis import analyze_rows


def _flatten(value, prefix=""):
    result = {}
    for key, item in value.items():
        name = f"{prefix}.{key}" if prefix else key
        if isinstance(item, dict):
            result.update(_flatten(item, name))
        elif item is None or isinstance(item, (str, bool, int, float)):
            result[name] = item
    return result


def _distribution(values):
    if not values:
        return {"count": 0}
    values = sorted(values)
    def percentile(p):
        at = (len(values) - 1) * p
        low, high = math.floor(at), math.ceil(at)
        return round(values[low] + (values[high] - values[low]) * (at - low), 3)
    return dict(count=len(values), min=round(values[0], 3), p50=percentile(.5),
                p95=percentile(.95), max=round(values[-1], 3))


def _checked_path(root, name, suffix):
    if not isinstance(name, str) or Path(name).name != name or not name.startswith("magp-capture-") or not name.endswith(suffix):
        raise ValueError("invalid finalizer job member")
    path = root / name
    if path.is_symlink():
        raise ValueError("finalizer job members must not be symlinks")
    return path


def _isolate_from_parent(expected_parent):
    """Lower scheduling priority and exit if the owning process disappears."""
    if hasattr(os, "nice"):
        os.nice(10)
    if sys.platform.startswith("linux"):
        # Runs inside the new interpreter, never in Popen preexec/fork hooks.
        import ctypes
        if os.getppid() != expected_parent:
            raise RuntimeError("recording owner exited before finalizer startup")
        libc = ctypes.CDLL(None, use_errno=True)
        if libc.prctl(1, signal.SIGTERM, 0, 0, 0) != 0:
            raise OSError(ctypes.get_errno(), "cannot set finalizer parent-death signal")
        if os.getppid() != expected_parent:
            raise RuntimeError("recording owner exited during finalizer startup")


def finalize_job(job_path):
    job_path = Path(job_path).resolve()
    root = job_path.parent
    job = json.loads(job_path.read_text(encoding="utf-8"))
    if job.get("schema_version") != 1:
        raise ValueError("unsupported finalizer job schema")
    _isolate_from_parent(int(job["parent_pid"]))
    partial = _checked_path(root, job["partial_name"], ".partial.zip")
    rows_path = _checked_path(root, job["rows_name"], ".rows.jsonl")
    final = _checked_path(root, job["final_name"], ".zip")
    rows = [json.loads(line) for line in rows_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not rows or len(rows) != job["summary"]["frames"]:
        raise ValueError("sealed recording row count does not match its job")
    events = job.get("events", [])
    timings, flattened = {}, []
    for row in rows:
        flat = _flatten(row)
        flattened.append(flat)
        for name, value in flat.items():
            if (name.startswith(("metadata.", "recording.")) and name.endswith("_ms")
                    and isinstance(value, (int, float)) and not isinstance(value, bool)
                    and math.isfinite(value)):
                timings.setdefault(name, []).append(float(value))
    summary = dict(job["summary"])
    summary["timings_ms"] = {name: _distribution(values) for name, values in timings.items()}
    summary["finalization"] = {"transport": "isolated_python_process", "pid": os.getpid(),
                               "nice": os.nice(0) if hasattr(os, "nice") else None,
                               "parent_pid": os.getppid()}
    with zipfile.ZipFile(partial, "a", compression=zipfile.ZIP_STORED, allowZip64=True) as archive:
        # Copy the already serialized aligned rows rather than re-encoding them.
        archive.write(rows_path, "frames.jsonl")
        archive.writestr("events.jsonl", "".join(json.dumps(event, ensure_ascii=False, allow_nan=False) + "\n" for event in events).encode("utf-8"))
        archive.writestr("summary.json", json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False).encode("utf-8"))
        archive.writestr("analysis.json", json.dumps(analyze_rows(rows, events), ensure_ascii=False, indent=2, allow_nan=False).encode("utf-8"))
        columns = sorted({key for row in flattened for key in row})
        csv_file = io.StringIO(newline="")
        writer = csv.DictWriter(csv_file, fieldnames=columns)
        writer.writeheader()
        writer.writerows(flattened)
        archive.writestr("frames.csv", csv_file.getvalue().encode("utf-8-sig"))
    partial.replace(final)
    rows_path.unlink()
    job_path.unlink()
    return final


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--job", required=True)
    args = parser.parse_args()
    try:
        finalize_job(args.job)
    except BaseException as error:
        print(f"{type(error).__name__}: {error}"[:2048], file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
