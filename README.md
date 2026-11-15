# MAGP 2026 技術報告と保存走行版

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

## 車載Piでの起動

既存依存と接続を確認済みの車載Piで使用するランチャーは、リポジトリのルートから次で指定できる。

```bash
bash source/system/run_with_g2.sh
```

これは論文生成とは別の実機用コマンドである。起動時は停止・Nであり、依存・接続条件は `source/system/README.md` と `WIRING.md` に記した。本保存作業で新規実機への起動試験は行っていない。

## 権利とエクスポート範囲

`RIGHTS.md`を参照する。全体にMIT等を付けていない。既存コードの由来とEven SDK資材の再配布条件は未確認であり、本人用の非公開保存を想定する。リポジトリ公開設定の変更は、このパッケージ生成では行わない。

匿名化・選別・原本hashとの比較は`release_export_manifest.json`に残す。公開保存用の文書や集計JSONの個人絶対パスは`${SOURCE_ROOT}`等へ置換した。25本の走行コードは変更していない。数値とコードの採用版は保存資料の範囲でのみ確認でき、実舵角・対地速度・実軌跡・電波占有率は未測定である。
