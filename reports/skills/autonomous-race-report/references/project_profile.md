# MAGP保存資料のプロファイル（2026年11月15日）

今回の資料へ戻るための索引である。以下の名前・版・数値を次大会の既定値にはしない。ソースの詳細や測定結果は重複転記せず、既存の調査・検証文書へ戻る。

## 資料の置き場所

報告書ルートを `REPORTS`、保存競技資料のルートを `BASE` とする。この配布時点では `BASE/reports` が `REPORTS` である。移設時にはユーザーが指定するrootと設定ファイルから解決し、Skillの絶対インストール先に固定しない。

| 調査目的 | 参照する既存資料 |
|---|---|
| 全体の存在・ハッシュ・読取範囲 | `REPORTS/source_inventory.md`、`REPORTS/data/source_inventory.json` |
| 実装、採用版、継承、別系列 | `REPORTS/system_analysis.md`、`REPORTS/research/code_evidence.json` |
| 元リポジトリのGit状態 | `REPORTS/research/original_repository_recovery.json` |
| セッション集計と場面の事実 | `REPORTS/competition_analysis.md`、`REPORTS/data/run_metrics.json` |
| 文献と出典の確認 | `REPORTS/literature_review.md`、`REPORTS/references.bib`、`REPORTS/verification.md`（存在するもの） |
| 外部Skillの版・監査・導入範囲 | `REPORTS/research/skill_review.md`、`REPORTS/research/install_receipt.json` |
| 実行構成・配布ハッシュ | `BASE/controller_hud_candidate/CURRENT_MANIFEST.json`、同 `IMPLEMENTATION.md`、当該走行の配布記録 |
| 予選の原記録 | `BASE/qualifying_review_20261115_1137/`。CSV、JSONL、画像、アーカイブを同期キーで照合 |
| 無線の観測と過去の報告 | `BASE/pico_radio_monitor_20261115/`。原ログ、分集計、測定期間、既存PDFを区別 |

## 今回、報告範囲を誤らないための確認点

- 元ドライブは初回に未認識だったが、同日中に復帰した。回収した `.git` にはHEADコミットがなく、3件のCodex checkpoint参照はtreeであった。「Gitがない」と固定せず、復旧記録を読み、通常のコミット履歴も作らない。
- `controller_hud_candidate/` は名称だけで未採用と判断できない。保存予選の `metadata.build`、配布記録、ファイルハッシュの照合が採用の根拠である。元リポジトリの同名ファイルは別の古い統合系列であり、保存予選版の説明を置換しない。
- 回収した `rpi_system/overtaking/` には局所計画とメートル単位のPure Pursuitがある。一方、保存予選の白線追従版は、その系列をimportすることが確認されていない。プロジェクト全体にPure Pursuitがないとも、予選がPure Pursuitで走ったとも書かない。模擬試験・shadow確認を実車追い抜きの成功に数えない。
- 初期仕様書やTT-02の方式選択ガイドは設計・検討資料である。記載された全方式、パラメータ、ハードウェア型番を採用実績としない。`baseline_` も最初の提供原本そのものと同定されているか確認する。
- 既存の接触解析は**予選**の連続AUTO記録である。決勝初戦と同一の記録であるとは確認されていない。ログ末尾と最後の衝突を自動的に同一視しない。
- ユーザーは「7位、決勝進出後の初戦敗退」と報告した。添付図の7は**予選順位**欄である。公式結果の順位区分・チームを照合し、「決勝総合7位」へ読み替えない。公式ページ：<https://minicar-autogp.org/race/04th_results/>。
- 画像から白線が消えた後にもAUTOを維持する設定が今回の運用要求に含まれた。知覚STOPを安全停止と説明せず、実際のアクセル指令・運転者STOP・緊急停止を別々に示す。

## 既存スクリプトの再利用境界

`REPORTS/scripts/analyze_runs.py` の通常実行は、特定の予選セッション・画像番号に注釈を付ける。次大会は明示した `--reviews` と `--inventory-only` から始める。そこでは競技段階や接触時刻は推定されない。新しい画像を実際に確認して、別の注釈・根拠を作る。

`REPORTS/scripts/make_figures.py` は作成時点で今回の `run_metrics.json`、`qualifying_timeline.csv`、`qualifying_30s_bins.csv`、`manual_software_latency_benchmark.csv` に対応する。汎用モードの集計だけではこれらが揃わないため、新大会に対して無条件に実行しない。後続版に設定対応がある場合は、その実在する設定・CLI・スキーマを優先する。

再生成・検証の統合スクリプトが `REPORTS/scripts/rebuild.py`、`verify_report.py` として存在すれば、現在の設定と `--help` を確認して用いる。未作成または移設で欠けたファイルを実行済みとして記録しない。追加パッケージやSkillの有効化は自動的に含めず、既存環境で実行できる処理を先に使う。
