#!/usr/bin/env python3
"""Read a drive-capture ZIP and print its diagnostic analysis as JSON.

Examples:
  python3 analyze_drive_capture.py run.zip
  python3 analyze_drive_capture.py run.zip --output /tmp/magp-review

The source ZIP is opened read-only. --output writes analysis.json and only the
representative originals/masks, retaining their relative archive paths.
"""
import argparse
import json
from pathlib import Path, PurePosixPath
import sys
import zipfile

from drive_capture_analysis import analyze_rows


_MAX_JSONL_BYTES = 32 * 1024 * 1024
_MAX_IMAGE_BYTES = 8 * 1024 * 1024
_MAX_ROWS = 10000


def _read_rows(archive, name, optional=False):
    try:
        entry = archive.getinfo(name)
    except KeyError:
        if optional:
            return [], 0
        raise ValueError(f"archive has no {name}")
    if entry.file_size > _MAX_JSONL_BYTES:
        raise ValueError(f"{name} exceeds 32 MiB read limit")
    rows, malformed = [], 0
    with archive.open(entry, "r") as stream:
        for line in stream:
            if not line.strip():
                continue
            if len(rows) >= _MAX_ROWS:
                raise ValueError(f"{name} exceeds {_MAX_ROWS} rows")
            try:
                rows.append(json.loads(line))
            except (json.JSONDecodeError, UnicodeDecodeError):
                rows.append(None)
                malformed += 1
    return rows, malformed


def _extract_reference(archive, root, relative):
    if not isinstance(relative, str):
        return None
    name = PurePosixPath(relative)
    if (name.is_absolute() or ".." in name.parts or "\\" in relative or "\x00" in relative
            or not name.parts or name.parts[0] not in ("images", "masks", "extra_masks")):
        raise ValueError(f"unsafe representative path: {relative!r}")
    entry = archive.getinfo(relative)
    if entry.is_dir() or entry.file_size > _MAX_IMAGE_BYTES:
        raise ValueError(f"representative file exceeds 8 MiB or is a directory: {relative}")
    target = root.joinpath(*name.parts)
    # Protect repeated runs against an existing output symlink escaping root.
    if not target.resolve().is_relative_to(root.resolve()):
        raise ValueError(f"representative output escapes output directory: {relative}")
    target.parent.mkdir(parents=True, exist_ok=True)
    body = archive.read(entry)
    if target.exists():
        if target.read_bytes() != body:
            raise ValueError(f"output already contains a different file: {relative}")
    else:
        target.write_bytes(body)
    return relative


def analyze_archive(path, output=None):
    """Read a ZIP without changing it; optionally extract up to 12 frame sets."""
    path = Path(path)
    with zipfile.ZipFile(path, "r") as archive:
        rows, malformed = _read_rows(archive, "frames.jsonl")
        events, malformed_events = _read_rows(archive, "events.jsonl", optional=True)
        report = analyze_rows(rows, events)
        report["source_archive"] = path.name
        report["input"]["malformed_frame_jsonl_lines"] = malformed
        report["input"]["malformed_event_jsonl_lines"] = malformed_events
        if output is not None:
            root = Path(output)
            root.mkdir(parents=True, exist_ok=True)
            extracted, missing = [], []
            names = set()
            for representative in report["representative_frames"]:
                for name in (representative.get("image"), representative.get("mask"),
                             *representative.get("extra_masks", {}).values()):
                    if name is not None:
                        names.add(name)
            for name in sorted(names):
                try:
                    extracted.append(_extract_reference(archive, root, name))
                except KeyError:
                    missing.append(name)
            report["extracted_files"] = extracted
            report["missing_representative_files"] = missing
            destination = root / "analysis.json"
            if not destination.resolve().is_relative_to(root.resolve()):
                raise ValueError("analysis output escapes output directory")
            if destination.resolve() == path.resolve():
                raise ValueError("analysis output must not replace source ZIP")
            destination.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("archive", type=Path, help="completed MAGP diagnostic ZIP")
    parser.add_argument("--output", type=Path, help="write analysis.json and representative images/masks here")
    args = parser.parse_args(argv)
    try:
        report = analyze_archive(args.archive, args.output)
        print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    except (OSError, ValueError, KeyError, zipfile.BadZipFile) as exc:
        parser.exit(2, f"analysis failed: {exc}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
