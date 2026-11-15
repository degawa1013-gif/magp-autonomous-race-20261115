#!/usr/bin/env python3
"""Select and anonymize the saved report for a separate repository.

Default: dry-run, without writing to the destination. --apply copies only the
allowlisted report artifacts and generated package documentation. It never
executes vehicle code, installs dependencies, accesses a network, or invokes Git.
The 25 saved competition files are validated, never edited.
"""
from __future__ import annotations

import argparse
import ast
from collections import Counter
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys


TOP_DOCUMENTS = (
    "CREATIVE_DIRECTION.md", "source_inventory.md", "system_analysis.md",
    "competition_analysis.md", "literature_review.md", "paper_outline.md",
    "paper.md", "references.bib", "verification.md", "environment.md",
    "README.md", "report_config.json",
)
DATA = (
    "manual_software_latency_benchmark.csv", "official_competition_results.json",
    "qualifying_30s_bins.csv", "qualifying_timeline.csv", "radio_event_summary.json",
    "reference_index.json", "run_analysis_validation.json", "run_metrics.json",
    "session_metrics.csv", "verification_results.json",
)
RESEARCH = (
    "autonomous_race_skill_validation.json", "code_evidence.json",
    "install_receipt.json", "literature_sources.json",
    "manuscript_technical_review.md", "original_repository_recovery.json",
    "scientific_keyword_audit.json", "scientific_numeric_audit.json",
    "scientific_prose_audit.json", "skill_review.md", "pdf_visual_review.json",
    "raw_integration_validation.json", "rebuild_validation.json",
    "skill_forward_test_summary.json",
    "skill_audit/source_manifest.json", "skill_audit/static_audit.json",
)
REQUIRED = (
    "paper.md", "references.bib", "report_config.json", "data/run_metrics.json",
    "research/code_evidence.json", "scripts/rebuild.py", "scripts/verify_report.py",
    "data/contact_frames/source_manifest.json",
    "data/contact_frames/frame_000338.jpg", "data/contact_frames/frame_000341.jpg",
    "data/contact_frames/frame_000346.jpg",
)
FORBIDDEN_SUFFIXES = {".har", ".pem", ".key", ".jsonl", ".log", ".bag", ".db3",
                      ".mcap", ".mp4", ".mov", ".zip", ".tar", ".tgz", ".pt",
                      ".pth", ".onnx", ".safetensors", ".ckpt", ".pyc"}
FORBIDDEN_PARTS = {".ssh", ".gh", ".aws", ".gnupg", "node_modules", ".venv",
                   "__pycache__", "raw_logs", "raw_data", "models", "weights"}
TEXT_SUFFIXES = {".md", ".bib", ".json", ".csv", ".py", ".sh", ".txt",
                 ".yaml", ".yml", ".toml", ".svg", ".html", ".js", ".css"}
SENSITIVE_KEYS = {"ssid", "bssid", "mac", "macaddress", "peerid", "authorization",
                  "cookie", "setcookie", "accesstoken", "refreshtoken", "password",
                  "passphrase", "privatekey", "secretkey", "apitoken", "apikey"}
IPV4 = re.compile(r"(?<![\w.])(?:\d{1,3}\.){3}\d{1,3}(?![\w.])")
MAC = re.compile(r"(?i)(?<![\w:])(?:[0-9a-f]{2}:){5}[0-9a-f]{2}(?![\w:])")
EMAIL = re.compile(r"(?<![\w])[-.\w+]+@[-.\w]+\.[a-zA-Z]{2,}(?![\w])")
TOKEN_QUERY = re.compile(r"(?i)([?&](?:token|access_token|api_key|auth|signature|sig)=)([^&#\s\"'<>]+)")
KNOWN_TOKEN = re.compile(r"\b(?:gh[pousr]_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{30,}|AKIA[A-Z0-9]{16}|sk-[A-Za-z0-9_-]{24,})\b")
PRIVATE_KEY = re.compile(r"-----BEGIN (?:OPENSSH |RSA |EC |DSA )?PRIVATE KEY-----")
PERSONAL_HOME = re.compile(r"/(?:home|Users)/([^/\s\"'<>`]+)")
PRIVATE_IPV6 = re.compile(r"(?i)(?<![\w:])(?:fe80|fc[0-9a-f]{2}|fd[0-9a-f]{2})(?::[0-9a-f]{0,4}){2,7}(?:%[\w]+)?(?![\w:])")


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def private_ipv4(match):
    try:
        value = ipaddress.ip_address(match.group())
    except ValueError:
        return False
    canonical_networks = {str(ipaddress.ip_address(number)): suffix for number, suffix in (
        (10 << 24, "/8"), ((172 << 24) + (16 << 16), "/12"),
        ((192 << 24) + (168 << 16), "/16"))}
    if (str(value) in canonical_networks and
            match.string[match.end():].startswith(canonical_networks[str(value)])):
        # Public classification constants in the sanitizer are not a measured
        # host/AP address. Redacting them would corrupt the exported program.
        return False
    # Loopback, wildcard binding and documentation addresses are not identities.
    return not value.is_loopback and str(value) != "0.0.0.0" and (
        value in ipaddress.ip_network("10.0.0.0/8") or
        value in ipaddress.ip_network("172.16.0.0/12") or
        value in ipaddress.ip_network("192.168.0.0/16") or value.is_link_local)


def sanitize_text(value: str, replacements: list[tuple[str, str]], counts: Counter) -> str:
    for old, new in replacements:
        if old and old in value:
            counts["personal_path_prefixes"] += value.count(old)
            value = value.replace(old, new)
    def ip_replace(match):
        if private_ipv4(match):
            counts["private_ipv4"] += 1
            return "<private-address>"
        return match.group()
    value = IPV4.sub(ip_replace, value)
    for pattern, label, replacement in (
        (PRIVATE_IPV6, "private_ipv6", "<private-address>"),
        (MAC, "mac_addresses", "<redacted-mac>"),
        (EMAIL, "email_addresses", "<redacted-email>"),
        (KNOWN_TOKEN, "credential_tokens", "<redacted-credential>"),
        (PRIVATE_KEY, "private_key_marker", "<redacted-private-key-marker>"),
    ):
        def replace(match, label=label, replacement=replacement):
            counts[label] += 1
            return replacement
        value = pattern.sub(replace, value)
    def query(match):
        counts["credential_query_values"] += 1
        return match.group(1) + "<redacted>"
    value = TOKEN_QUERY.sub(query, value)
    def home(match):
        name = match.group(1)
        if name in {"pi", "USER", "user", "<user>", "${USER}"}:
            return match.group()
        counts["personal_home"] += 1
        return "${LOCAL_USER_HOME}"
    return PERSONAL_HOME.sub(home, value)


def sanitized(value, replacements, counts):
    if isinstance(value, dict):
        result = {}
        for key, item in value.items():
            normalized = re.sub(r"[^a-z0-9]", "", key.lower())
            if normalized in SENSITIVE_KEYS and item not in (None, "", [], {}):
                counts["sensitive_fields"] += 1
                result[key] = "<redacted>"
            else:
                result[key] = sanitized(item, replacements, counts)
        return result
    if isinstance(value, list):
        return [sanitized(v, replacements, counts) for v in value]
    if isinstance(value, str):
        return sanitize_text(value, replacements, counts)
    return value


def minimized_code_evidence(data, counts):
    """Keep implementation/hash/line structures; omit full machine receipts."""
    result = dict(data)
    receipts = []
    for receipt in result.get("deployment_receipts", []):
        record = receipt.get("record") or {}
        retained = {k: record[k] for k in (
            "hashes", "deployed_source_sha256", "adaptive_helper_sha256",
            "unchanged_sources_verified", "timestamp_epoch_ms", "mode", "gear",
            "pi_tests", "pi_tests_passed", "python_tests", "physical_tracking_improvement_verified",
            "motion_command_sent", "neutral_pwm", "neutral_gpio",
            "old_missing_fixture_skipped", "real_motion_clip_frames_recognized",
            "adaptive_lighting_synthetic_cases", "lost_steering", "live_validation",
            "offline_benchmark", "physical_road_response", "loop_timing",
        ) if k in record}
        receipts.append({"path": receipt.get("path"), "sha256": receipt.get("sha256"),
                         "summary": retained,
                         "full_receipt_included": False})
        counts["deployment_receipts_minimized"] += 1
    result["deployment_receipts"] = receipts
    result["export_scope"] = {
        "raw_deployment_records_included": False,
        "saved_source_and_recorded_build_hashes_preserved": True,
        "note": "Paths describe evidence provenance. Raw evidence is not bundled; identifiers and personal path prefixes are anonymized.",
    }
    return result


def minimized_inventory(data):
    keep = ("schema", "created_utc", "downloads_allowlist", "downloads_matches",
            "downloads_unmatched_patterns", "original_repository", "scan_policy",
            "excluded_counts", "summary", "top_level_groups", "representatives")
    result = {k: data[k] for k in keep if k in data}
    result["export_scope"] = {
        "full_file_ledger_included": False,
        "original_ledger_entries": len(data.get("files", [])),
        "original_file_byte_hash_location": "release_export_manifest.json:selected_report_files",
        "note": "A compact classification/representative-source index; not the original per-file machine ledger. Original file-byte hash is in the release export manifest.",
    }
    return result


def discover(reports: Path):
    paths = {reports / name for name in TOP_DOCUMENTS if (reports / name).is_file()}
    paths.update(reports / "data" / name for name in DATA if (reports / "data" / name).is_file())
    paths.update(reports / "research" / name for name in RESEARCH if (reports / "research" / name).is_file())
    inventory = reports / "data/source_inventory.json"
    if inventory.is_file():
        paths.add(inventory)
    for folder, suffixes in (("scripts", {".py"}), ("skills", TEXT_SUFFIXES),
                             ("figures", {".png", ".pdf", ".svg", ".json"}),
                             ("data/contact_frames", {".jpg", ".json"}),
                             ("output/pdf", {".pdf"})):
        root = reports / folder
        if root.is_dir():
            paths.update(p for p in root.rglob("*") if p.is_file() and p.suffix.lower() in suffixes)
    for name in ("output/build_receipt.json", "output/verification_receipt.json"):
        if (reports / name).is_file():
            paths.add(reports / name)
    return sorted(p for p in paths if not p.is_symlink() and
                  not FORBIDDEN_PARTS.intersection(p.relative_to(reports).parts) and
                  p.suffix.lower() not in FORBIDDEN_SUFFIXES)


def source_snapshot_check(destination: Path):
    manifest_path = destination / "source/system/SOURCE_MANIFEST.json"
    if not manifest_path.is_file():
        raise ValueError("Existing immutable source/system/SOURCE_MANIFEST.json is required")
    manifest = json.loads(manifest_path.read_text())
    files = manifest.get("files", [])
    if len(files) != 25:
        raise ValueError("The competition source manifest must contain exactly 25 files")
    checked = []
    for entry in files:
        path = destination / entry["path"]
        if not path.resolve().is_relative_to((destination / "source/system").resolve()):
            raise ValueError("Source manifest path escapes source/system")
        if not path.is_file() or path.is_symlink() or digest(path.read_bytes()) != entry["sha256"]:
            raise ValueError("Immutable competition source mismatch: " + entry["path"])
        checked.append({"path": entry["path"], "sha256": entry["sha256"]})
    return checked


def portability_patch(relative: str, text: str, counts: Counter):
    if relative == "scripts/inventory_sources.py":
        # This optional inventory tool must never assume the author's old disk.
        text, n = re.subn(r'original = Path\("/media/[^\n]+"\)', 'original = root', text)
        counts["portable_inventory_original_root"] += n
        text, n = re.subn(r'default=Path\("/(?:home)/[^\n]+/Downloads"\)', 'default=Path.home() / "Downloads"', text)
        counts["portable_downloads_default"] += n
    return text


def residual_findings(text: str):
    findings = Counter()
    findings["private_ipv4"] = sum(private_ipv4(m) for m in IPV4.finditer(text))
    for pattern, key in ((PRIVATE_IPV6, "private_ipv6"), (MAC, "mac_addresses"),
                         (EMAIL, "email_addresses"), (KNOWN_TOKEN, "credential_tokens"),
                         (PRIVATE_KEY, "private_key_marker")):
        findings[key] = len(pattern.findall(text))
    findings["personal_home"] = sum(m.group(1) not in {"pi", "USER", "user", "<user>", "${USER}"}
                                    for m in PERSONAL_HOME.finditer(text))
    # Anonymized placeholders are explicitly not credential values.
    findings["credential_query_values"] = sum(m.group(2) != "<redacted>" for m in TOKEN_QUERY.finditer(text))
    return {k: v for k, v in findings.items() if v}


def pdf_text(path: Path):
    tool = shutil.which("pdftotext")
    if not tool:
        raise ValueError("pdftotext is required to inspect exported PDFs; no dependency is installed automatically")
    result = subprocess.run([tool, "-layout", str(path), "-"], text=True,
                            capture_output=True, timeout=30, check=True)
    return result.stdout


def check_jpeg_metadata(path: Path):
    # Original diagnostic JPEGs have no APP1 metadata. Do not silently rewrite
    # them: exact-byte hashes are used as evidence in the portable reproduction.
    data = path.read_bytes()
    offset = 2
    if data[:2] != b"\xff\xd8":
        raise ValueError("Invalid evidence JPEG")
    while offset + 4 <= len(data) and data[offset] == 255:
        marker = data[offset + 1]
        if marker in (0xDA, 0xD9):
            break
        if marker == 0xE1:
            raise ValueError("Evidence JPEG has APP1/EXIF metadata; review before export: " + path.name)
        size = int.from_bytes(data[offset+2:offset+4], "big")
        if size < 2:
            raise ValueError("Invalid JPEG metadata segment")
        offset += size + 2


def generated_documents():
    return {
        "README.md": """# MAGP 2026 技術報告と保存走行版

第4回MAGPに参加したNATC Tochigi / Monovision_2の、保存予選版と技術報告書の再解析用パッケージである。公式予選7位で決勝へ進出し、決勝初戦敗退は参加者報告である。決勝ログや最終総合順位を推定していない。

- 論文：`reports/paper.md` / `reports/output/pdf/autonomous_race_report_20261115.pdf`
- 図と集約データ：`reports/figures/` / `reports/data/`
- 根拠・未確認事項：`reports/verification.md` / `reports/research/`
- 再利用Skill：`reports/skills/autonomous-race-report/`
- 当時の走行版：`source/system/`。25本のhashを`SOURCE_MANIFEST.json`に固定した。

## 保存資料だけで論文を再生成する

リポジトリのルートで、必要な既存Python環境を選び、次を実行する。

```bash
python3 reports/scripts/rebuild.py --mode cached
```

このコマンドは保存済み集約値・3枚の根拠画像・保存ソースを検証し、図表とPDFを再生成する。カメラ、GPIO、Pi、Bluetooth、原JSONLを必要としない。`reports/environment.md`に利用環境と依存を記す。NumPy、matplotlib、Pillow、ReportLab、Poppler等を必要とする工程があるが、スクリプトは自動インストールしない。

原記録から再集計する場合は、**別途保管した本人の資料**を明示する。

```bash
python3 reports/scripts/rebuild.py --mode raw --source-root /path/to/preserved-evidence
```

raw入力の巨大ZIP/JSONL、無線AP一覧、個人の接続先、認証情報、デプロイ全文、モデルは同梱しない。集約値の原入力ハッシュと由来は残しているが、未同梱の原資料がこのリポジトリだけで復元できるという意味ではない。

## 走行ソースの扱い

`source/system/README.md`と`WIRING.md`に依存・配線・未確認事項を記した。記録用の旧起動スクリプトも保存しているが、パッケージ作成時に実車起動や新規インストールは試験していない。論文の再生成コマンドから走行プログラムは起動しない。実車に接続していないPCで走行スクリプトを実行して動作検証したとは扱わない。

## 権利とエクスポート範囲

`RIGHTS.md`を参照する。全体にMIT等を付けていない。既存コードの由来とEven SDK資材の再配布条件は未確認であり、本人用の非公開保存を想定する。リポジトリ公開設定の変更は、このパッケージ生成では行わない。

匿名化・選別・原本hashとの比較は`release_export_manifest.json`に残す。公開保存用の文書や集計JSONの個人絶対パスは`${SOURCE_ROOT}`等へ置換した。25本の走行コードは変更していない。数値とコードの採用版は保存資料の範囲でのみ確認でき、実舵角・対地速度・実軌跡・電波占有率は未測定である。
""",
        "PACKAGING.md": """# パッケージ作成者用の手順

これは競技用の起動手順ではなく、保存・再解析・公開前確認の手順である。

1. 元の報告作業域で `python3 reports/scripts/prepare_release.py` を実行する。既定はdry-runで、選別結果と検出件数だけを表示する。
2. 承認済みの別保存先に反映する際は `--destination /path/to/new-repository --apply` を明示する。既存の`source/system/SOURCE_MANIFEST.json`と25本の原本hashが前提であり、走行コードを書き換えない。
3. 保存先へ移り、`python3 reports/scripts/rebuild.py --mode cached`を実行する。匿名化で文書/JSONのbyte hashは変わるため、図metadataと検証receiptをこの場所で更新する。
4. `reports/data/verification_results.json`とPDFの描画を確認する。元rawがないことを欠測とし、成功率や実舵角等を補わない。
5. `release_export_manifest.json`の選別・除外・匿名化とソース25本の一致を確認し、権利範囲と公開設定を別途確認する。Git作成、commit、push、外部送信はこのスクリプトの処理に含まれない。

`reports/data/source_inventory.json`は分類・代表資料だけの軽量な索引であり、個人環境の全ファイル台帳ではない。`${SOURCE_ROOT}`、`${ORIGINAL_PROJECT_ROOT}`、`${VEHICLE_PROJECT_ROOT}`は由来を示す文字列であり、自動探索や実機接続の指定ではない。原資料を読む工程では明示した`--source-root`を使う。
""",
    }


def self_test():
    counts = Counter()
    private = str(ipaddress.ip_address((192 << 24) + (168 << 16) + 9))
    sample = {"ssid": "example-identity", "source_files": [{"sha256": "a"*64}],
              "note": private + " https://example.org/?token=" + "example-value"}
    output = sanitized(sample, [], counts)
    assert output["ssid"] == "<redacted>"
    assert output["source_files"][0]["sha256"] == "a"*64
    assert private not in output["note"] and "example-value" not in output["note"]
    assert not residual_findings(json.dumps(output))
    cidrs = " ".join(str(net) for net in (ipaddress.ip_network((10 << 24, 8)),
        ipaddress.ip_network(((172 << 24) + (16 << 16), 12)),
        ipaddress.ip_network(((192 << 24) + (168 << 16), 16))))
    assert sanitize_text(cidrs, [], Counter()) == cidrs
    assert not residual_findings(cidrs)
    own_source = Path(__file__).read_text()
    assert not residual_findings(own_source)
    print(json.dumps({"self_test": "passed", "checks": 7,
                      "canonical_cidr_policy_unchanged": True}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reports", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--destination", type=Path)
    parser.add_argument("--apply", action="store_true", help="Write the approved separate package; otherwise dry-run")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--max-file-mib", type=int, default=16)
    parser.add_argument("--max-total-mib", type=int, default=64)
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    reports = args.reports.resolve()
    destination = (args.destination or reports.parent / "github_release_20261115").resolve()
    if destination == reports or reports.is_relative_to(destination) or destination.is_relative_to(reports):
        raise ValueError("Destination must be a separate package, not the original report tree")
    immutable = source_snapshot_check(destination)
    missing = [name for name in REQUIRED if not (reports / name).is_file()]
    replacements = [
        (str(reports.parent), "${SOURCE_ROOT}"),
        (str(Path.home()), "${LOCAL_USER_HOME}"),
        ("/media/" + Path.home().name + "/5402273002271690/ChatGPT/MAGP", "${ORIGINAL_PROJECT_ROOT}"),
        ("/media/" + Path.home().name + "/", "${LOCAL_MEDIA_ROOT}/"),
        ("/home/pi/robust_lane_failsafe", "${VEHICLE_PROJECT_ROOT}"),
    ]
    replacements.sort(key=lambda pair: len(pair[0]), reverse=True)
    entries, payloads, findings, total_counts = [], {}, [], Counter()
    selected = discover(reports)
    for path in selected:
        relative = path.relative_to(reports).as_posix()
        original = path.read_bytes()
        if len(original) > args.max_file_mib * 1024 * 1024:
            raise ValueError("Allowlisted file exceeds size limit: " + relative)
        counts = Counter()
        data = original
        if path.suffix.lower() in TEXT_SUFFIXES:
            text = original.decode("utf-8")
            text = portability_patch(relative, text, counts)
            if relative == "scripts/prepare_release.py":
                # Keep this generic policy implementation byte-identical. Its
                # network constants and replacement-pattern literals are code,
                # not collected identities. Validate them in --self-test.
                data = original
            elif path.suffix == ".json":
                obj = json.loads(text)
                if relative == "research/code_evidence.json":
                    obj = minimized_code_evidence(obj, counts)
                if relative == "data/source_inventory.json":
                    obj = minimized_inventory(obj)
                    counts["full_inventory_minimized"] += 1
                obj = sanitized(obj, replacements, counts)
                data = (json.dumps(obj, ensure_ascii=False, indent=2) + "\n").encode()
            else:
                text = sanitize_text(text, replacements, counts)
                if relative == "source_inventory.md":
                    text = "<!-- Export note: data/source_inventory.json contains a compact summary and representative sources; the original full machine file ledger is intentionally omitted. -->\n\n" + text
                data = text.encode()
            final_text = data.decode("utf-8")
            residual = residual_findings(final_text)
            if path.suffix == ".py":
                ast.parse(final_text, filename=relative)
            if residual:
                findings.append({"path": "reports/" + relative, "counts": residual})
        elif path.suffix == ".pdf":
            residual = residual_findings(pdf_text(path))
            if residual:
                findings.append({"path": "reports/" + relative, "counts": residual,
                                 "action": "Regenerate PDF from anonymized documents; do not silently edit binary evidence"})
        elif path.suffix in {".jpg", ".jpeg"}:
            check_jpeg_metadata(path)
        payloads["reports/" + relative] = data
        total_counts.update(counts)
        entries.append({"path": "reports/" + relative,
                        "source_sha256": digest(original), "export_sha256": digest(data),
                        "source_bytes": len(original), "export_bytes": len(data),
                        "transforms": {k: v for k, v in counts.items() if v}})
    for path, text in generated_documents().items():
        payloads[path] = text.encode()
    total_bytes = sum(len(v) for v in payloads.values())
    if total_bytes > args.max_total_mib * 1024 * 1024:
        raise ValueError("Selected report exceeds package size limit")
    excluded = Counter()
    selected_set = set(selected)
    for path in reports.rglob("*"):
        if path.is_file() and path not in selected_set:
            first = path.relative_to(reports).parts[0]
            excluded[first] += 1
    review = {
        "schema": "magp-release-export-v1", "mode": "apply" if args.apply else "dry-run",
        "source_25_files_unchanged": immutable, "selected_report_files": entries,
        "generated_documents": sorted(generated_documents()), "total_export_bytes": total_bytes,
        "transform_counts": dict(total_counts), "residual_findings": findings,
        "missing_required_files": missing, "excluded_file_counts_by_top_folder": dict(excluded),
        "excluded_categories": ["full per-file machine inventory", "raw wireless identities and scans",
            "full deployment receipts", "raw JSONL/ZIP/log/media", "original HTML and third-party research source trees",
            "credentials, .env, HAR, SSH/account files", "models, environments and build scratch"],
        "needs_staged_cached_rebuild": True,
        "limitations": ["Only selected report artifacts are exported; raw evidence is retained separately.",
            "Source hashes and numeric values are preserved. Personal path prefixes and identifiers may be redacted.",
            "PDF text and evidence-JPEG metadata are checked; no general steganography detector is claimed.",
            "No network, Git mutation, install or vehicle operation is performed."],
    }
    if args.apply:
        if missing or findings:
            raise ValueError("Export blocked: missing required files or residual private data; inspect dry-run counts")
        for relative, data in payloads.items():
            target = destination / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
        # Metadata written outside source/system may itself contain old local
        # path prefixes; sanitize it while retaining all previous review facts.
        old_review = destination / "release_review.json"
        if old_review.is_file():
            old = json.loads(old_review.read_text())
            old_review.write_text(json.dumps(sanitized(old, replacements, Counter()), ensure_ascii=False, indent=2) + "\n")
        ignore = destination / ".gitignore"
        ignore_text = ignore.read_text() if ignore.is_file() else ""
        marker = "# Selected report deliverables (runtime output remains ignored)"
        if marker not in ignore_text:
            ignore.write_text(ignore_text + "\n" + marker + "\n!reports/output/\n!reports/output/pdf/\n!reports/output/pdf/*.pdf\n!reports/output/*receipt.json\n")
        source_snapshot_check(destination)
        source_readme = destination / "source/system/README.md"
        if source_readme.is_file():
            source_readme.write_text(source_readme.read_text().replace(
                "後述の欠落補助がある", "回収済みの補助と旧校正オプションは後述"))
        (destination / "release_export_manifest.json").write_text(json.dumps(review, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"mode": review["mode"], "source_files_unchanged": len(immutable),
                      "selected_report_files": len(entries), "total_export_bytes": total_bytes,
                      "transform_counts": dict(total_counts), "residual_findings": findings,
                      "missing_required_files": missing,
                      "write_performed": bool(args.apply)}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
