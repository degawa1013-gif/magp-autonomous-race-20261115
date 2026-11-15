# 第4回MAGP 技術報告書

NATC Tochigi / Monovision_2の競技開発を、保存ソース・走行記録・公式結果に対応させた日本語技術報告である。詳細な走行分析は保存予選区間を対象とし、決勝初戦のログを解析したとは扱わない。

## 読む順序

1. [本文](paper.md) または `output/pdf/autonomous_race_report_20261115.pdf`
2. [競技結果・接触場面の分析](competition_analysis.md)
3. [システム構成と実装](system_analysis.md)
4. [根拠・検証・未確認事項](verification.md)
5. [再生成環境と次大会への再利用](environment.md)

資料一覧は `source_inventory.md`、先行研究は `literature_review.md`、章構成は `paper_outline.md`、引用は `references.bib` に保存した。7図は `figures/`、解析用データは `data/`、再生成コードは `scripts/` にある。

## 再生成

保存パッケージのルート（`reports/` と `source/` のある場所）から実行する。

```bash
python3 reports/scripts/rebuild.py --mode cached
```

同梱CSV・JSON・JPEG・保存ソースだけから図表とPDFを作り、12群の整合性検査を行う。追加インストール、車両操作、ネットワークアクセスは行わない。原ログからの再集計には別途保管した原資料と `--mode raw --source-root /path/to/preserved-evidence` が必要である。原資料の欠落を同梱集計で代用したとは表示しない。

## 次大会の報告

`skills/autonomous-race-report/SKILL.md` が資料照合から執筆・レビューまでのプロジェクト専用手順である。一般的な科学文章の構成・推敲・工程は、監査後に導入した3つの `sciwrite-*` Skillへ委ねる。今回の順位、接触注釈、固定集計値を次大会へ流用せず、元JSONL/ZIP、実装ハッシュ、競技段階を新たに固定する。

著者・所属、決勝のログとセットスコア、実機の一部型番・導入版、実舵角等の校正値は未確認である。提出前の確認対象を `verification.md` にまとめた。
