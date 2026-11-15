# 論文の再生成環境

本環境は保存資料の分析と技術報告書の生成に用いる。車両、GPIO、カメラ、Bluetooth、Wi-Fiへ接続する処理は再生成パイプラインに含まれない。既存の走行プログラムを実行・変更せず、パッケージの追加インストールも行わない。

## 同梱資料から再生成する

配布物のルートを作業ディレクトリとする。必要な配置は `reports/` と、その隣の `source/system/` である。`source/system/` は予選保存版25ファイルの読取用スナップショットであり、車両への導入先ではない。

```bash
python3 reports/scripts/rebuild.py --mode cached --dry-run
python3 reports/scripts/rebuild.py --mode cached
```

`cached` は同梱のCSV・JSON、接触場面のJPEG3枚、保存ソースから、図7点（PNG・PDF・SVG）、文献表、本文PDF、検証記録を生成する。元の回収フォルダ、巨大な原ログ、過去の絶対パス、ネットワークは不要である。本文の生成済み参考文献欄と `references.bib` は保存書誌から更新するが、本文の議論や数値を自動執筆し直す機能ではない。参考文献URLの再アクセスも行わない。

主な出力は次のとおりである。

|出力|内容|
|---|---|
|`figures/`|図7点×3形式と入力・出力ハッシュ、条件、単位、限界の台帳|
|`output/pdf/autonomous_race_report_20261115.pdf`|本文PDF。日付はユーザー指定の2026-11-15を保持|
|`data/reference_index.json`、`references.bib`|確認済み保存書誌12件からの文献表|
|`data/verification_results.json`|保存CSVからの独立再計算、版・図・引用・本文・PDFの整合性検査|
|`output/build_receipt.json`|実行コマンド、工程ごとの終了値、生成物のSHA-256、再計算範囲|

PDFの文章調整中には `--skip-figures` で既存図を使える。この場合も既存図のハッシュを検証する。データ・図の確認中には `--skip-pdf` を使えるが、PDFの合格を意味しない。最終版では両方を省略した完全なコマンドを実行する。`--reports /path/to/reports` で報告書の配置先を明示でき、空白を含むパスは引用符で囲む。

## 原資料から再集計する

```bash
python3 reports/scripts/rebuild.py --mode raw --source-root /path/to/preserved-evidence --dry-run
python3 reports/scripts/rebuild.py --mode raw --source-root /path/to/preserved-evidence
```

`raw` には明示した原資料ルートが必須である。`report_config.json` に列挙した11回収フォルダのJSONLまたはZIP、予選の制御CSV、保存した4条件のソフトウェア比較JSON、および無線レポートの凍結集計2ファイルが必要である。欠けていれば集計前に一覧を出して終了し、同梱集計で代用して生ログ再計算済みと表示することはない。

走行部分は原JSONL/ZIPと制御CSVから再集計する。一方、無線部分は既存レポートの凍結集計から必要な項目を再抽出する。無線の全生レコード、RF波形、I/Qスペクトルを新たに計算し直す工程ではない。この違いはビルド記録にも残る。原資料の一覧941件を今回保存した調査台帳と比較する作業は、このパイプラインとは別である。

## 既存の実行環境と依存

調査時に以下の環境が存在した。再生成に必要なライブラリを持つ既存Pythonを `rebuild.py` が工程ごとに選択する。見つからなければ変更内容を示さずにインストールすることはなく、必要なモジュール名を出して終了する。固定した環境を使う場合は `report_config.json` の `python.core`、`python.figures`、`python.pdf` に実行ファイルを指定する。

|工程|必要なもの|調査時に利用可能であった環境|
|---|---|---|
|集計・文献更新・検証|Python標準ライブラリ。Python 3.9以上|システムPython 3.10.12|
|図生成|NumPy、matplotlib、Pillow、日本語フォント|NumPy 2.2.6、matplotlib 3.10.9、Pillow 9.0.1。Noto Sans CJK JP|
|PDF生成|ReportLab、既存の日本語・欧文フォント|システムReportLab 3.6.8。アプリ同梱Python 3.12.14／ReportLab 4.4.9も存在|
|PDFの構造・文字検証|`pdfinfo`、`pdftotext`|`/usr/bin/pdfinfo`、`/usr/bin/pdftotext`|
|PDFの目視用描画|`pdftoppm`など既存の描画手段|`/usr/bin/pdftoppm`。自動検査とは別工程|

PDFスクリプトは既存の `DroidSansFallbackFull.ttf`、`DejaVuSerif.ttf`、`DejaVuSans-Bold.ttf` を使用する。調査時の配置先はそれぞれ `/usr/share/fonts/truetype/droid/`、`/usr/share/fonts/truetype/dejavu/` であった。別OSでは既存のフォントに合わせた明示的な調整が必要となり得る。フォントや描画ライブラリの版が異なれば、数値が同じでも図・PDFのバイト列は変わり得るため、生成後にSHA台帳を更新し、全ページを再確認する。

このPCのアプリ同梱PopplerはGLIBCの要件を満たさず実行できなかった。検証スクリプトは利用可能なシステム `/usr/bin` のPDFツールを優先する。文書PDFの構造・文字検査を通過しても、改ページ、図の可読性、日本語組版の目視確認を代替しない。目視確認の記録は `research/pdf_visual_review.json` に、対象PDFのSHA、確認した全ページ、残る問題と対応を保存する。

## Skillと次大会への再利用

科学執筆には、監査・承認後に導入した `sciwrite-manuscript`、`sciwrite-prose`、`sciwrite-process` を用いた。採否、固定コミット、ライセンス、導入記録は `research/skill_review.md` と `research/install_receipt.json` に残した。英語向けの文章監査は日本語の校閲や技術的妥当性の証明ではない。Claude Figure Skillは追加依存と環境依存を要し、既存のmatplotlibで必要な図を生成できるため採用しなかった。

専用Skillは `skills/autonomous-race-report/` に配布用として置く。グローバルなSkill領域にはインストールしていない。根拠の整理とプロジェクト固有の取り違え防止を担当し、一般的な論文執筆は上記3Skillへ委ねる。

`report_config.json` と現在の図・接触注釈は2026-11-15用の凍結設定である。日付や順位だけを書き換えて別大会に使ってはならない。次大会では原JSONL/ZIPを保存し、明示した回収フォルダに対して次から始める。

```bash
python3 reports/scripts/analyze_runs.py --base /path/to/next-event \
  --reviews next_event_review --inventory-only --out reports/data/next_event
```

現在の `analyze_runs.py` の通常実行には今回固有の予選区間・画像注釈が含まれる。次大会の対象版、ログ列、同期、欠測、評価条件を確かめてからイベント別設定・集計アダプターを用意する。派生CSVだけが残っている場合は列と型を確認して既存集計へ接続し、原画像・原JSONL・版照合を再検証できていない範囲を明記する。
