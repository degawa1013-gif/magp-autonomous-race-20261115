<!-- Export note: data/source_inventory.json contains a compact summary and representative sources; the original full machine file ledger is intentionally omitted. -->

# 論文用資料の一覧と利用範囲

調査対象は保存作業領域のソース・設計資料・走行記録と、Downloads内の明示された技術資料である。競技用プログラムは変更していない。全ファイルの機械一覧は `data/source_inventory.json` に保存し、本書は代表資料の役割と制限を示す。

## 1. リポジトリと保存資料の状態

- 保存資料：`${SOURCE_ROOT}`
- 元の作業場所：`${ORIGINAL_PROJECT_ROOT}`。
- 元ディレクトリの存在は確認した。Git履歴の調査はこの一覧作成スクリプトの範囲外である。
- ファイルの更新時刻はコピー・展開で変わりうるため、実験時刻や採用順序の根拠にしない。

## 2. 分類別の保存状況

| 分類 | ファイル数 | 合計bytes | SHA-256取得数 |
|---|---:|---:|---:|
| archive | 34 | 183,392,957 | 0 |
| captured_image_or_mask | 10,332 | 40,254,114 | 0 |
| document | 69 | 5,930,352 | 69 |
| image_or_figure | 3,474 | 87,976,058 | 1 |
| manifest_or_metadata | 501 | 16,827,198 | 501 |
| media | 48 | 103,823,728 | 0 |
| other | 43 | 34,140,526 | 0 |
| source_or_tool | 195 | 5,325,901 | 195 |
| structured_log | 111 | 184,010,357 | 111 |
| test_source | 64 | 651,404 | 64 |

画像・マスク・動画・アーカイブは全件を開いたことを意味しない。大量画像は主に存在・サイズのみを確認し、ソース・構造化ログ・メタデータと代表資料にSHA-256を付与した。読み取り範囲は機械一覧の `read_scope` で区別する。

## 3. 代表資料と根拠の役割

パスは保存資料ルートからの相対パスである。SHA-256の全文は機械一覧に保存する。

| ID | 資料 | 役割・利用上の限界 | 存在 |
|---|---|---|---|
| SRC01 | `baseline_robust_lane_failsafe.py` | 初期保存ソース：初期実装との比較起点。大会最終稼働版とは同一視しない | あり |
| SRC02 | `baseline_legacy_pi_output.py` | 初期出力モジュール：Pi直接出力の保存版。実配線や実波形の証明とは別 | あり |
| SRC03 | `controller_hud_candidate/robust_lane_failsafe.py` | 統合制御ソース：認識・制御・入力・記録を統合した保存ソース | あり |
| SRC04 | `controller_hud_candidate/CURRENT_MANIFEST.json` | 統合版ハッシュ一覧：各モジュールの同一性を照合できる。Gitコミットではない | あり |
| SRC05 | `controller_hud_candidate/IMPLEMENTATION.md` | 実装・操作説明：モード、G2、DualSenseと検証範囲の説明。記述時点を区別する | あり |
| SRC06 | `controller_hud_candidate/PHYSICAL_VERIFICATION.md` | 台上確認記録：操舵・前後進等の確認。路面試験や成功率へ一般化しない | あり |
| SRC07 | `controller_hud_candidate/steering_preview.py` | 操舵先読み：現行候補の実装を説明する一次ソース | あり |
| SRC08 | `controller_hud_candidate/far_preview.py` | 遠方白線抽出：先読み候補の処理内容を確認できる | あり |
| SRC09 | `controller_hud_candidate/curve_pace.py` | 自動出力調整：出力割合の演算。実速度や最速経路の証明ではない | あり |
| SRC10 | `controller_hud_candidate/dualsense_control.py` | 手動入力処理：入力・モード・ギア処理の一次ソース | あり |
| SRC11 | `controller_hud_candidate/fast_control_output.py` | 出力処理：操作遅延対策の候補実装。無線区間の遅延とは区別する | あり |
| SRC12 | `controller_hud_candidate/drive_capture.py` | 画像・状態記録：記録条件・保存周期・容量制限の定義 | あり |
| SRC13 | `controller_hud_candidate/wifi_monitor.py` | 車載Wi-Fi診断：測定・キャッシュ・収集項目の定義 | あり |
| SRC14 | `controller_hud_candidate/bluetooth_monitor.py` | 車載Bluetooth診断：HCI診断等の取得方法。電波スペクトラム測定ではない | あり |
| SRC15 | `qualifying_review_20261115_1137/archive_sources.json` | 予選資料の凍結索引：アーカイブ・走行ログ・ソースハッシュの対応 | あり |
| SRC16 | `qualifying_review_20261115_1137/run_control_log.csv` | 予選制御ログ：指令・認識・エンコーダ等の記録。実舵角や測位真値は含意しない | あり |
| SRC17 | `qualifying_review_20261115_1137/run_log_manifest.json` | 走行ログ形式：CSV・マスク動画の同期キーと列を定義 | あり |
| SRC18 | `qualifying_review_20261115_1137/magp-capture-20261115-112838-1628279c-s003/frames.jsonl` | 画像同期メタデータ：画像と認識・指令を同一frameで照合する一次記録 | あり |
| SRC19 | `qualifying_review_20261115_1137/analysis_summary.json` | 予選解析要約：観測・仮説・未確定事項の二次資料。原ログとの照合が必要 | あり |
| SRC20 | `qualifying_review_20261115_1137/qualifying_diagnosis.png` | 接触場面の図版：画像と指令の説明図。指令値を実舵角として読まない | あり |
| SRC21 | `qualifying_review_20261115_1137/macro_temporal_audit.json` | AUTO全区間の解析：接触までの経過時間などを原記録から再検証する入口 | あり |
| SRC22 | `pico_radio_monitor_20261115/pico_main.py` | Pico観測ファーム：走行制御用Picoの過去版と区別した、無線観測用コード | あり |
| SRC23 | `pico_radio_monitor_20261115/logs/scans.jsonl` | Pico Wi-Fi観測：PC位置でのAP検出記録。車両位置や帯域占有率ではない | あり |
| SRC24 | `pico_radio_monitor_20261115/pi_radio_logs/pi_radio.jsonl` | 車載診断の収集：スナップショット・欠測・キャッシュを含む | あり |
| SRC25 | `pico_radio_monitor_20261115/event_reports/event_report.json` | 無線全体の分集計：データ数・時系列集計の二次資料。原記録へ戻れる | あり |
| SRC26 | `pico_radio_monitor_20261115/analysis_report_20261115/output/pdf/tournament_radio_measurement_report_20261115.pdf` | 既存無線レポート：先行報告の再利用元。今回の論文から無引用で転載しない | あり |
| SRC27 | `pico_radio_monitor_20261115/measurement_window.json` | 測定期間設定：18時終了の予定。18時まで実際に採れたことの証拠にはしない | あり |
| SRC28 | `steering_gpio19_candidate/CURRENT_WIRING.md` | 保存配線資料：当該版の接続表。長期開発で変更された配線と区別する | あり |

## 4. Downloads内で確認した技術資料

個人の登録資料や無関係なダウンロードは開かず、ファイル名の許可リストに一致する資料だけを対象にした。HTMLはブラウザで実行せず、表示用テキストの冒頭を読んだ。

| 資料 | サイズbytes | 確認範囲と利用方針 |
|---|---:|---|
| `45053_2021.pdf` | 2,943,241 | 8ページの画像PDF。1ページ目を描画確認し、Finespec2.4G RCセット取説と識別。抽出テキストは空で、全ページを精読したとは扱わない |
| `TT02_RaspberryPi_MAGP_Driving_Guide.pdf` | 250,709 | 1〜2ページの本文を抽出確認。10方式の設計候補集であり、全方式の採用を意味しない |
| `lane_analyzer (1).html` | 23,205 | 冒頭UIと制御ログ項目を確認。可視化ツールの存在は評価の実施や結果を証明しない |
| `lane_analyzer (2).html` | 22,287 | 冒頭UIと制御ログ項目を確認。可視化ツールの存在は評価の実施や結果を証明しない |
| `lane_analyzer.html` | 20,522 | 冒頭UIと制御ログ項目を確認。可視化ツールの存在は評価の実施や結果を証明しない |
| `robust_lane_failsafe_spec (1).html` | 66,862 | 冒頭の仕様・構成概要を確認。初期仕様の資料であり、現在の実装・配線と照合して使う |
| `robust_lane_failsafe_spec (2).html` | 66,863 | 冒頭の仕様・構成概要を確認。初期仕様の資料であり、現在の実装・配線と照合して使う |
| `robust_lane_failsafe_spec (3).html` | 80,549 | 冒頭の仕様・構成概要を確認。初期仕様の資料であり、現在の実装・配線と照合して使う |
| `robust_lane_failsafe_spec (4).html` | 80,549 | 冒頭の仕様・構成概要を確認。初期仕様の資料であり、現在の実装・配線と照合して使う |
| `robust_lane_failsafe_spec.html` | 43,603 | 冒頭の仕様・構成概要を確認。初期仕様の資料であり、現在の実装・配線と照合して使う |

該当がなかった許可パターン：`quickstart*`。不存在の資料を参照済みとして扱わない。

## 5. 初回調査で内容を確認した範囲（2026年11月15日）

- `controller_hud_candidate/IMPLEMENTATION.md`：モード・G2/DualSense・検証・配線の説明を通読。資料自身が台上試験と路面走行を区別している。
- `qualifying_review_20261115_1137/run_run_meta.json`、`run_log_manifest.json`、`analysis_summary.json`：メタデータと要約を確認。同期キーは `frame_index`。261.07秒の画像セッションと、接触の最終事象同定が未確定である旨を記載している。
- `controller_hud_candidate/CURRENT_MANIFEST.json`：キーとハッシュの構成を確認。`archive_sources.json`・`measurement_window.json` は上位キーを確認した。
- `qualifying_review_20261115_1137/run_control_log.csv`：ヘッダーを確認。操舵・出力・エンコーダ・オドメトリ等の列はあるが、列名だけで独立した測位真値と認めない。
- この節の内容確認は初回調査の記録であり、再走査によって新しい内容を精読したことにはならない。上記以外のソースのアルゴリズム、ログ全行、画像全件について、本書の一覧化だけで精読・実測検証を済ませたとは扱わない。技術的な確認箇所は `system_analysis.md` と `verification.md` を参照する。

## 6. 根拠の強さと不足データ

1. **実装の根拠**：保存された該当ソース、設定、SHAで結ぶ配布記録。コードは実行された版・設定と対応して初めて実験の説明になる。
2. **観測の根拠**：同一frameの生画像・状態・CSV、測定条件付きの診断値。PWM指令と実舵角、車輪換算速度と独立した実車速度を区別する。
3. **解釈の根拠**：過去の解析JSON・図版・報告書は再利用できる二次資料である。主張は元記録へ照合し、反復転載を独立した証拠として数えない。
4. **結果の根拠**：公式結果と当事者の説明を区別する。予選ログを決勝ログと呼び替えない。ユーザーが報告した7位・決勝初戦敗退と、公式表の順位区分は別途照合する。

不足または照合が必要な資料：元リポジトリのGit履歴（本一覧では未照合）、決勝走行に対応すると確定できるログ・画像・採用設定、コースの実測寸法・基準軌跡、実舵角・独立した測位真値、同一条件の反復試行と対照実験、無線の帯域使用時間・再送・電波スペクトラム。資料が揃わない項目について成功率、実軌跡誤差、原因の断定を作らない。

## 7. 再実行

```bash
python3 '${SOURCE_ROOT}/reports/scripts/inventory_sources.py' \
  --project-root '${SOURCE_ROOT}' \
  --downloads-root '${LOCAL_USER_HOME}/Downloads'
```

出力は `reports/data/source_inventory.json` と本書である。既存の走行ファイルへの書込み、ネットワークアクセス、走行プログラムのimportはない。本書は再生成されるため、次回の固有資料はスクリプトの代表資料一覧または独立した検証記録に追加する。

除外：この `reports/` 出力、依存ライブラリ、仮想環境、音声ランタイム、モデル重み、キャッシュ、ビルド中間物、隠しファイル、シンボリックリンク。アーカイブ内部は展開しない。
