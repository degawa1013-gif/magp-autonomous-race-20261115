#!/usr/bin/env python3
"""Inventory preserved MAGP evidence without importing or changing vehicle code.

Only the selected project root and an explicit Downloads allowlist are scanned.
The reports directory containing this script, dependencies, symlinks and model
weights are excluded. This is a source ledger, not an algorithm verifier.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import fnmatch
import hashlib
import json
import os
from pathlib import Path
import stat


EXCLUDE_DIRS = {
    "node_modules", ".venv", "venv", "asr_venv", "voice_runtime", "__pycache__",
    ".git", ".cache", ".pytest_cache", "site-packages", "models", "model",
    "weights", "checkpoints", "build", "pdf_render", "rendered",
}
MODEL_SUFFIXES = {".onnx", ".pt", ".pth", ".ckpt", ".safetensors", ".vvm", ".vvbin"}
TEXT_HASH_SUFFIXES = {
    ".py", ".sh", ".js", ".mjs", ".css", ".html", ".md", ".txt", ".json",
    ".jsonl", ".csv", ".toml", ".yaml", ".yml", ".bib", ".tex", ".tsv",
}
DOWNLOAD_PATTERNS = (
    "robust_lane_failsafe_spec*.html", "quickstart*", "lane_analyzer*",
    "TT02_RaspberryPi_MAGP_Driving_Guide.pdf", "45053_2021.pdf",
)

REVIEWED_DOWNLOAD_SHA256 = {'45053_2021.pdf': '20dc89cee75c2ed715e0b5f1a17baab5f3601b4607cbdedb3f26b9ce66ccf209', 'TT02_RaspberryPi_MAGP_Driving_Guide.pdf': '026423d1346918692c3a39f36550fdf0ff839c39fcb1db3c7dcc376f0f0d0fcb', 'lane_analyzer (1).html': '67b57a2435c49080f5a8a191b94a311d090fd7e05316c6e0ee30c10757ad0bc5', 'lane_analyzer (2).html': 'acec803c58c2d0810675d4ce804ef81959aeb742f4163633e213d903b8c2bce8', 'lane_analyzer.html': '9316bf979f8750a17ffe028b2effa2143742f12445be5e5b2c7e9ac081dcc193', 'robust_lane_failsafe_spec (1).html': '73fbc51131e449517f5a5bd1ed915af1e5f1a36c6af19a0a9e36fe324a40908a', 'robust_lane_failsafe_spec (2).html': '38239a8a5cf3c21a08be55430c431b0bafc98f796530ac844783e06ff75dd9e2', 'robust_lane_failsafe_spec (3).html': '35dfda67189959b2e4b000d63c4efc4f4defbf246fa0e40cf0cd831e9c9a17ea', 'robust_lane_failsafe_spec (4).html': '35dfda67189959b2e4b000d63c4efc4f4defbf246fa0e40cf0cd831e9c9a17ea', 'robust_lane_failsafe_spec.html': '6db1fefd18dc381ed296def761c6e212381c181310e54433df810f6a9c3f12d0'}

# Roles are deliberately about what an artifact can establish, not design intent.
REPRESENTATIVES = [
    ("SRC01", "baseline_robust_lane_failsafe.py", "初期保存ソース", "初期実装との比較起点。大会最終稼働版とは同一視しない"),
    ("SRC02", "baseline_legacy_pi_output.py", "初期出力モジュール", "Pi直接出力の保存版。実配線や実波形の証明とは別"),
    ("SRC03", "controller_hud_candidate/robust_lane_failsafe.py", "統合制御ソース", "認識・制御・入力・記録を統合した保存ソース"),
    ("SRC04", "controller_hud_candidate/CURRENT_MANIFEST.json", "統合版ハッシュ一覧", "各モジュールの同一性を照合できる。Gitコミットではない"),
    ("SRC05", "controller_hud_candidate/IMPLEMENTATION.md", "実装・操作説明", "モード、G2、DualSenseと検証範囲の説明。記述時点を区別する"),
    ("SRC06", "controller_hud_candidate/PHYSICAL_VERIFICATION.md", "台上確認記録", "操舵・前後進等の確認。路面試験や成功率へ一般化しない"),
    ("SRC07", "controller_hud_candidate/steering_preview.py", "操舵先読み", "現行候補の実装を説明する一次ソース"),
    ("SRC08", "controller_hud_candidate/far_preview.py", "遠方白線抽出", "先読み候補の処理内容を確認できる"),
    ("SRC09", "controller_hud_candidate/curve_pace.py", "自動出力調整", "出力割合の演算。実速度や最速経路の証明ではない"),
    ("SRC10", "controller_hud_candidate/dualsense_control.py", "手動入力処理", "入力・モード・ギア処理の一次ソース"),
    ("SRC11", "controller_hud_candidate/fast_control_output.py", "出力処理", "操作遅延対策の候補実装。無線区間の遅延とは区別する"),
    ("SRC12", "controller_hud_candidate/drive_capture.py", "画像・状態記録", "記録条件・保存周期・容量制限の定義"),
    ("SRC13", "controller_hud_candidate/wifi_monitor.py", "車載Wi-Fi診断", "測定・キャッシュ・収集項目の定義"),
    ("SRC14", "controller_hud_candidate/bluetooth_monitor.py", "車載Bluetooth診断", "HCI診断等の取得方法。電波スペクトラム測定ではない"),
    ("SRC15", "qualifying_review_20261115_1137/archive_sources.json", "予選資料の凍結索引", "アーカイブ・走行ログ・ソースハッシュの対応"),
    ("SRC16", "qualifying_review_20261115_1137/run_control_log.csv", "予選制御ログ", "指令・認識・エンコーダ等の記録。実舵角や測位真値は含意しない"),
    ("SRC17", "qualifying_review_20261115_1137/run_log_manifest.json", "走行ログ形式", "CSV・マスク動画の同期キーと列を定義"),
    ("SRC18", "qualifying_review_20261115_1137/magp-capture-20261115-112838-1628279c-s003/frames.jsonl", "画像同期メタデータ", "画像と認識・指令を同一frameで照合する一次記録"),
    ("SRC19", "qualifying_review_20261115_1137/analysis_summary.json", "予選解析要約", "観測・仮説・未確定事項の二次資料。原ログとの照合が必要"),
    ("SRC20", "qualifying_review_20261115_1137/qualifying_diagnosis.png", "接触場面の図版", "画像と指令の説明図。指令値を実舵角として読まない"),
    ("SRC21", "qualifying_review_20261115_1137/macro_temporal_audit.json", "AUTO全区間の解析", "接触までの経過時間などを原記録から再検証する入口"),
    ("SRC22", "pico_radio_monitor_20261115/pico_main.py", "Pico観測ファーム", "走行制御用Picoの過去版と区別した、無線観測用コード"),
    ("SRC23", "pico_radio_monitor_20261115/logs/scans.jsonl", "Pico Wi-Fi観測", "PC位置でのAP検出記録。車両位置や帯域占有率ではない"),
    ("SRC24", "pico_radio_monitor_20261115/pi_radio_logs/pi_radio.jsonl", "車載診断の収集", "スナップショット・欠測・キャッシュを含む"),
    ("SRC25", "pico_radio_monitor_20261115/event_reports/event_report.json", "無線全体の分集計", "データ数・時系列集計の二次資料。原記録へ戻れる"),
    ("SRC26", "pico_radio_monitor_20261115/analysis_report_20261115/output/pdf/tournament_radio_measurement_report_20261115.pdf", "既存無線レポート", "先行報告の再利用元。今回の論文から無引用で転載しない"),
    ("SRC27", "pico_radio_monitor_20261115/measurement_window.json", "測定期間設定", "18時終了の予定。18時まで実際に採れたことの証拠にはしない"),
    ("SRC28", "steering_gpio19_candidate/CURRENT_WIRING.md", "保存配線資料", "当該版の接続表。長期開発で変更された配線と区別する"),
]


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def category(path: Path) -> str:
    name = path.name.lower()
    suffix = path.suffix.lower()
    parts = {p.lower() for p in path.parts}
    if suffix in {".zip", ".gz", ".tar", ".tgz"}:
        return "archive"
    if suffix in {".jpg", ".jpeg", ".png", ".webp", ".svg"}:
        return "captured_image_or_mask" if parts & {"images", "masks", "extra_masks"} else "image_or_figure"
    if suffix in {".mp4", ".mov", ".wav", ".mp3", ".m4a", ".flac"}:
        return "media"
    if suffix in {".md", ".html", ".pdf", ".txt"}:
        return "document"
    if suffix in {".py", ".js", ".mjs", ".ts", ".css", ".sh"}:
        return "test_source" if name.startswith(("test_", "benchmark_")) else "source_or_tool"
    if suffix in {".csv", ".tsv", ".jsonl"}:
        return "structured_log"
    if suffix in {".json", ".toml", ".yaml", ".yml"}:
        return "manifest_or_metadata"
    return "other"


def project_files(root: Path, report_dir: Path, excluded: Counter):
    for current, dirs, files in os.walk(root, followlinks=False):
        current_path = Path(current)
        kept = []
        for name in sorted(dirs):
            child = current_path / name
            if child == report_dir:
                excluded["report_output_directory"] += 1
            elif child.is_symlink():
                excluded["symlink_directory"] += 1
            elif name in EXCLUDE_DIRS or name.startswith("."):
                excluded["dependency_cache_or_hidden_directory"] += 1
            else:
                kept.append(name)
        dirs[:] = kept
        for name in sorted(files):
            path = current_path / name
            if path.is_symlink():
                excluded["symlink_file"] += 1
            elif name.startswith("."):
                excluded["hidden_file"] += 1
            elif path.suffix.lower() in MODEL_SUFFIXES:
                excluded["model_weight_file"] += 1
            elif stat.S_ISREG(path.stat().st_mode):
                yield path


def file_record(path: Path, origin: str, root: Path, hash_limit: int, representative: bool = False):
    st = path.stat()
    should_hash = representative or origin == "downloads_allowlist" or path.suffix.lower() in TEXT_HASH_SUFFIXES
    hashed = should_hash and st.st_size <= hash_limit
    return {
        "path": str(path), "relative_path": str(path.relative_to(root)), "origin": origin,
        "exists": True, "bytes": st.st_size, "category": category(path),
        "mtime_utc": datetime.fromtimestamp(st.st_mtime, timezone.utc).isoformat(),
        "sha256": digest(path) if hashed else None,
        "read_scope": "stat_and_full_file_hash" if hashed else "stat_only",
        "hash_omission_reason": None if hashed else ("above_hash_size_limit" if should_hash else "bulk_binary_content_not_read"),
    }


def build_inventory(root: Path, downloads: Path, report_dir: Path, hash_limit: int):
    excluded = Counter()
    representatives = {r[1]: r for r in REPRESENTATIVES}
    records = []
    if root.is_dir():
        for path in project_files(root, report_dir, excluded):
            rel = str(path.relative_to(root))
            try:
                records.append(file_record(path, "preserved_project", root, hash_limit, rel in representatives))
            except OSError as exc:
                records.append({"path": str(path), "relative_path": rel, "origin": "preserved_project", "exists": path.exists(), "error": str(exc)})
    matched_downloads = []
    # Enumerate names to apply the allowlist; never open unrelated Downloads files.
    if downloads.is_dir():
        for path in sorted(downloads.iterdir()):
            if not any(fnmatch.fnmatch(path.name.casefold(), pat.casefold()) for pat in DOWNLOAD_PATTERNS):
                continue
            if not path.is_file() or path.is_symlink():
                continue
            matched_downloads.append(path.name)
            records.append(file_record(path, "downloads_allowlist", downloads, hash_limit, True))
    by_path = {r["path"]: r for r in records}
    selected = []
    for ident, rel, role, limit in REPRESENTATIVES:
        record = by_path.get(str(root / rel))
        selected.append({"id": ident, "relative_path": rel, "role": role, "evidence_limit": limit,
                         "exists": bool(record), "bytes": record.get("bytes") if record else None,
                         "sha256": record.get("sha256") if record else None,
                         "content_review_scope": "Inventory confirms existence and hash; algorithmic review is documented separately"})
    groups = defaultdict(lambda: {"files": 0, "bytes": 0, "hashed_files": 0})
    top = defaultdict(lambda: {"files": 0, "bytes": 0})
    for rec in records:
        group = groups[rec.get("category", "unreadable")]
        group["files"] += 1; group["bytes"] += rec.get("bytes", 0); group["hashed_files"] += int(bool(rec.get("sha256")))
        if rec["origin"] == "preserved_project":
            parts = Path(rec["relative_path"]).parts
            name = parts[0] if len(parts) > 1 else "(root)"
            top[name]["files"] += 1; top[name]["bytes"] += rec.get("bytes", 0)
    original = root
    return {
        "schema": "magp_source_inventory_v1", "created_utc": datetime.now(timezone.utc).isoformat(),
        "project_root": str(root), "reports_output_excluded": str(report_dir),
        "downloads_root": str(downloads), "downloads_allowlist": list(DOWNLOAD_PATTERNS),
        "downloads_matches": matched_downloads,
        "downloads_unmatched_patterns": [p for p in DOWNLOAD_PATTERNS if not any(fnmatch.fnmatch(n.casefold(), p.casefold()) for n in matched_downloads)],
        "original_repository": {"path": str(original), "exists": original.is_dir(),
            "mount_path": str(original.parents[1]), "mount_present": os.path.ismount(original.parents[1]),
            "git_metadata_at_original_root": (original / ".git").exists(),
            "git_metadata_at_preserved_root": (root / ".git").exists(),
            "history_read": False, "reason": "No Git history was read; preserved snapshots and SHA manifests are not Git commits"},
        "scan_policy": {"excluded_directory_names": sorted(EXCLUDE_DIRS), "hidden_directories_excluded": True,
            "symlinks_followed": False, "model_suffixes_excluded": sorted(MODEL_SUFFIXES),
            "hash_max_bytes": hash_limit, "hash_policy": "text/source/log/metadata and representative artifacts; ordinary bulk images/media are stat only",
            "source_files_modified": False, "network_access": False, "original_program_imported": False},
        "excluded_counts": dict(sorted(excluded.items())), "summary": dict(sorted(groups.items())),
        "top_level_groups": dict(sorted(top.items())), "representatives": selected,
        "files": records,
    }


def markdown(inventory: dict, output: Path) -> str:
    root = Path(inventory["project_root"])
    lines = ["# 論文用資料の一覧と利用範囲", "",
        "調査対象は保存作業領域のソース・設計資料・走行記録と、Downloads内の明示された技術資料である。競技用プログラムは変更していない。全ファイルの機械一覧は `data/source_inventory.json` に保存し、本書は代表資料の役割と制限を示す。", "",
        "## 1. リポジトリと保存資料の状態", "",
        f"- 保存資料：`{root}`", "- 元の作業場所：`${ORIGINAL_PROJECT_ROOT}`。"]
    original = inventory["original_repository"]
    if not original["exists"]:
        lines.append("- 調査時点で元ドライブは未マウントで、この場所へアクセスできなかった。保存資料ルートにも `.git` は見つからず、Git履歴は未取得である。変更点は保存版・差分・配布記録・SHA-256を根拠とし、架空のコミット番号や開発順序を与えない。")
    else:
        lines.append("- 元ディレクトリの存在は確認した。Git履歴の調査はこの一覧作成スクリプトの範囲外である。")
    lines += ["- ファイルの更新時刻はコピー・展開で変わりうるため、実験時刻や採用順序の根拠にしない。", "",
        "## 2. 分類別の保存状況", "", "| 分類 | ファイル数 | 合計bytes | SHA-256取得数 |", "|---|---:|---:|---:|"]
    for key, val in inventory["summary"].items():
        lines.append(f"| {key} | {val['files']:,} | {val['bytes']:,} | {val['hashed_files']:,} |")
    lines += ["", "画像・マスク・動画・アーカイブは全件を開いたことを意味しない。大量画像は主に存在・サイズのみを確認し、ソース・構造化ログ・メタデータと代表資料にSHA-256を付与した。読み取り範囲は機械一覧の `read_scope` で区別する。", "",
        "## 3. 代表資料と根拠の役割", "", "パスは保存資料ルートからの相対パスである。SHA-256の全文は機械一覧に保存する。", "",
        "| ID | 資料 | 役割・利用上の限界 | 存在 |", "|---|---|---|---|"]
    for rec in inventory["representatives"]:
        lines.append(f"| {rec['id']} | `{rec['relative_path']}` | {rec['role']}：{rec['evidence_limit']} | {'あり' if rec['exists'] else '未確認'} |")
    lines += ["", "## 4. Downloads内で確認した技術資料", "",
        "個人の登録資料や無関係なダウンロードは開かず、ファイル名の許可リストに一致する資料だけを対象にした。HTMLはブラウザで実行せず、表示用テキストの冒頭を読んだ。", "",
        "| 資料 | サイズbytes | 確認範囲と利用方針 |", "|---|---:|---|"]
    for rec in inventory["files"]:
        if rec["origin"] != "downloads_allowlist":
            continue
        name = Path(rec["path"]).name
        reviewed = rec["sha256"] == REVIEWED_DOWNLOAD_SHA256.get(name)
        if not reviewed:
            note = "名前・サイズ・SHA-256のみ。初回に読んだ版との同一性が確認できず、本文は要再確認"
        elif name.startswith("robust_lane_failsafe_spec"):
            note = "冒頭の仕様・構成概要を確認。初期仕様の資料であり、現在の実装・配線と照合して使う"
        elif name.startswith("lane_analyzer"):
            note = "冒頭UIと制御ログ項目を確認。可視化ツールの存在は評価の実施や結果を証明しない"
        elif name == "TT02_RaspberryPi_MAGP_Driving_Guide.pdf":
            note = "1〜2ページの本文を抽出確認。10方式の設計候補集であり、全方式の採用を意味しない"
        elif name == "45053_2021.pdf":
            note = "8ページの画像PDF。1ページ目を描画確認し、Finespec2.4G RCセット取説と識別。抽出テキストは空で、全ページを精読したとは扱わない"
        else:
            note = "名前・サイズ・SHA-256のみ。本文の技術レビューは別途行う"
        lines.append(f"| `{name}` | {rec['bytes']:,} | {note} |")
    if inventory["downloads_unmatched_patterns"]:
        lines.append("\n該当がなかった許可パターン：" + "、".join(f"`{p}`" for p in inventory["downloads_unmatched_patterns"]) + "。不存在の資料を参照済みとして扱わない。")
    lines += ["", "## 5. 初回調査で内容を確認した範囲（2026年11月15日）", "",
        "- `controller_hud_candidate/IMPLEMENTATION.md`：モード・G2/DualSense・検証・配線の説明を通読。資料自身が台上試験と路面走行を区別している。",
        "- `qualifying_review_20261115_1137/run_run_meta.json`、`run_log_manifest.json`、`analysis_summary.json`：メタデータと要約を確認。同期キーは `frame_index`。261.07秒の画像セッションと、接触の最終事象同定が未確定である旨を記載している。",
        "- `controller_hud_candidate/CURRENT_MANIFEST.json`：キーとハッシュの構成を確認。`archive_sources.json`・`measurement_window.json` は上位キーを確認した。",
        "- `qualifying_review_20261115_1137/run_control_log.csv`：ヘッダーを確認。操舵・出力・エンコーダ・オドメトリ等の列はあるが、列名だけで独立した測位真値と認めない。",
        "- この節の内容確認は初回調査の記録であり、再走査によって新しい内容を精読したことにはならない。上記以外のソースのアルゴリズム、ログ全行、画像全件について、本書の一覧化だけで精読・実測検証を済ませたとは扱わない。技術的な確認箇所は `system_analysis.md` と `verification.md` を参照する。", "",
        "## 6. 根拠の強さと不足データ", "",
        "1. **実装の根拠**：保存された該当ソース、設定、SHAで結ぶ配布記録。コードは実行された版・設定と対応して初めて実験の説明になる。",
        "2. **観測の根拠**：同一frameの生画像・状態・CSV、測定条件付きの診断値。PWM指令と実舵角、車輪換算速度と独立した実車速度を区別する。",
        "3. **解釈の根拠**：過去の解析JSON・図版・報告書は再利用できる二次資料である。主張は元記録へ照合し、反復転載を独立した証拠として数えない。",
        "4. **結果の根拠**：公式結果と当事者の説明を区別する。予選ログを決勝ログと呼び替えない。ユーザーが報告した7位・決勝初戦敗退と、公式表の順位区分は別途照合する。",
        "",
        "不足または照合が必要な資料：元リポジトリのGit履歴（本一覧では未照合）、決勝走行に対応すると確定できるログ・画像・採用設定、コースの実測寸法・基準軌跡、実舵角・独立した測位真値、同一条件の反復試行と対照実験、無線の帯域使用時間・再送・電波スペクトラム。資料が揃わない項目について成功率、実軌跡誤差、原因の断定を作らない。", "",
        "## 7. 再実行", "",
        "```bash", f"python3 '{output.parent / 'scripts/inventory_sources.py'}' \\",
        f"  --project-root '{root}' \\", f"  --downloads-root '{inventory['downloads_root']}'", "```", "",
        "出力は `reports/data/source_inventory.json` と本書である。既存の走行ファイルへの書込み、ネットワークアクセス、走行プログラムのimportはない。本書は再生成されるため、次回の固有資料はスクリプトの代表資料一覧または独立した検証記録に追加する。", "",
        "除外：この `reports/` 出力、依存ライブラリ、仮想環境、音声ランタイム、モデル重み、キャッシュ、ビルド中間物、隠しファイル、シンボリックリンク。アーカイブ内部は展開しない。", ""]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--downloads-root", type=Path, default=Path.home() / "Downloads")
    parser.add_argument("--hash-max-mib", type=int, default=32)
    args = parser.parse_args()
    root = args.project_root.resolve()
    reports = Path(__file__).resolve().parents[1]
    if not root.is_dir():
        parser.error("project root is not an accessible directory")
    if args.hash_max_mib < 1:
        parser.error("hash size limit must be positive")
    inventory = build_inventory(root, args.downloads_root.resolve(), reports, args.hash_max_mib * 1024 * 1024)
    target = reports / "data/source_inventory.json"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(inventory, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    report_path = reports / "source_inventory.md"
    report_path.write_text(markdown(inventory, report_path), encoding="utf-8")
    print(json.dumps({"files": len(inventory["files"]), "hashed_files": sum(bool(x.get("sha256")) for x in inventory["files"]),
                      "representatives_present": sum(x["exists"] for x in inventory["representatives"]),
                      "representatives_total": len(inventory["representatives"]),
                      "downloads_files": len(inventory["downloads_matches"]), "json": str(target), "markdown": str(report_path)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
