# 論文執筆 Skills と作図環境の調査

調査・導入日：2026年11月15日（日本標準時）。導入内容を提示後、ユーザーから科学執筆3件の導入について明示承認を得た。監査版の3件を Codex の `~/.codex/skills/` に導入済みである。pip・npm、論文に対する監査スクリプトは実行していない。公開ソースの原文は `research/skill_audit/` に保持する。実行・照合記録は `research/install_receipt.json` に保存した。

## 1. 判断

| 候補 | 判断 | 今回の適用範囲 |
|---|---|---|
| `sciwrite-manuscript` | 採用・承認後に導入済み | 図表を先に確定し、結果・方法・考察を区別する。ユーザー指定の日本語8章構成を優先する |
| `sciwrite-prose` | 採用・承認後に導入済み | 用語・数値を変えずに文章を整理する。日本語本文は読解による校閲を主とする |
| `sciwrite-process` | 採用・承認後に導入済み | 資料収集、執筆、推敲を分離し、出典・数値の整合性を確認する |
| Wavelix `sci-figure` | 今回は導入を見送る | 標準 matplotlib で必要な時系列図・散布図・構成図を再生成できる。追加の scipy、draw.io、MATLAB は不要 |
| 既存 `skill-creator` | 使用 | プロジェクト固有の資料・ログの読み方を担う専用 Skill を作成する。論文一般の指南は複製しない |
| 既存 `pdf:pdf` | 使用 | 日本語PDF生成後のレンダリング、文字・改ページ・図表の確認 |
| 既存 `documents:documents` | DOCX が必要な場合に使用 | 今回の必須成果物は Markdown とPDFであり、DOCXを増やすためには使用しない |

導入前の調査時点では、4候補とも `${LOCAL_USER_HOME}/.codex/skills` と `${LOCAL_USER_HOME}/.agents/skills` のいずれにも存在しなかった。承認後、科学執筆3件のみ `${LOCAL_USER_HOME}/.codex/skills` へ追加した。セッションで提示された既存 Skill 一覧も確認した。

導入前は監査済み原文を参考資料として読み、導入後は3件の実体を利用できる。CodexのSkill一覧への反映は次ターン以降となる。いずれの場合も、どの原文を参考にしたかを本書と検証記録に残す。

## 2. 版固定と出典

### Scientific Writing Skills

- リポジトリ：[dvdsosa/scientific-writing-skills](https://github.com/dvdsosa/scientific-writing-skills)
- 監査コミット：`2eb42c9afb4c9edd4ad5c3a4ae9351e139bae0a0`（2026-11-08T13:33:08Z）
- [README](https://github.com/dvdsosa/scientific-writing-skills/blob/2eb42c9afb4c9edd4ad5c3a4ae9351e139bae0a0/README.md)
- [sciwrite-manuscript/SKILL.md](https://github.com/dvdsosa/scientific-writing-skills/blob/2eb42c9afb4c9edd4ad5c3a4ae9351e139bae0a0/skills/sciwrite-manuscript/SKILL.md)
- [sciwrite-prose/SKILL.md](https://github.com/dvdsosa/scientific-writing-skills/blob/2eb42c9afb4c9edd4ad5c3a4ae9351e139bae0a0/skills/sciwrite-prose/SKILL.md)
- [sciwrite-process/SKILL.md](https://github.com/dvdsosa/scientific-writing-skills/blob/2eb42c9afb4c9edd4ad5c3a4ae9351e139bae0a0/skills/sciwrite-process/SKILL.md)

公式READMEの Any agent 節は Codex を対象に含め、`npx skills add dvdsosa/scientific-writing-skills` を案内している。ただし、このコマンドは未固定の npm CLI とリポジトリの最新状態を取得しうる。そのまま実行せず、下記の Codex 付属インストーラーと監査済みコミットの組合せを推奨する。

対象3件の `SKILL.md` は `name`・`description` を持つ Markdown 形式であり、特定の Claude ツールを必須としない。回答言語もユーザーの言語に合わせるため、日本語の構成・推敲には利用できる。Claude plugin 用メタデータは Codex では必要ない。

### Claude Figure Skill

- リポジトリ：[Wavelix/claude-figure-skill](https://github.com/Wavelix/claude-figure-skill)
- 監査コミット：`3c24a5379b8604f5e505f3e73ba624239b884224`（2026-09-30T11:47:10Z）
- [README](https://github.com/Wavelix/claude-figure-skill/blob/3c24a5379b8604f5e505f3e73ba624239b884224/README.md)
- [sci-figure/SKILL.md](https://github.com/Wavelix/claude-figure-skill/blob/3c24a5379b8604f5e505f3e73ba624239b884224/skills/sci-figure/SKILL.md)
- [pyproject.toml](https://github.com/Wavelix/claude-figure-skill/blob/3c24a5379b8604f5e505f3e73ba624239b884224/skills/sci-figure/python/pyproject.toml)

Skill の Markdown 自体は Codex でも読めるが、READMEは著者個人の macOS・conda・MATLAB・draw.io 環境向けであることを明記している。Skill 構文の互換性と、付属ツール一式が現在のPCで動くことは別である。

## 3. ソースコードと実行時の確認

### 3.1 Scientific Writing Skills

確認したものは、README、対象3件の `SKILL.md`、論文章節・図表・最終確認の参照文書、工学系推敲例、および次の4スクリプトである。

| ファイル | 実装で確認した入出力・依存 | 利用上の限界 |
|---|---|---|
| `extract_text.py` | `argparse`、`re`、`zipfile`、XMLなどPython標準ライブラリ。指定した Markdown/TeX/TXT/DOCX を読み、節・本文を標準出力へ出す | 章名の正規化は英語・スペイン語中心。日本語の「要旨」「緒言」を abstract/introduction として自動認識しない。TeX の図表・数式環境は除去される |
| `prose_audit.py` | 指定原稿または標準入力を読み、英語表現・文長・略語などの候補を表示。外部通信や原稿への書込みは見当たらない | 英語の語彙・文分割・受動態正規表現が中心。日本語本文で指摘ゼロでも品質保証にはならない |
| `keyword_consistency.py` | 上記抽出器と標準ライブラリ。用語の出現箇所、英語同義語、略語候補を表示 | 自動同義語検出は英語。`--terms` で指定した日本語文字列の出現確認は可能だが、存在しない節への一律追加を意味しない |
| `numeric_consistency.py` | 抽出された数字、`n=`、割合、桁数、平均±ばらつきを標準出力へ表示 | 日本語章名を要旨と認識しないと abstract/body 比較は働かない。異なる実験群の n を機械的に疑う。負号・単位・物理量の意味の全面的検証は行わない |

4本の Python ソースを静的に読み、構文解析も行った。調査対象4本にネットワーク接続、シェルコマンド実行、`eval`/`exec`、ファイル削除、原稿を書き換える処理は確認されなかった。これは静的確認の範囲であり、依存先や未調査の将来版を含む安全性の保証ではない。外部コードの実行試験は今回行っていない。

リポジトリ直下の `scripts` は `skills/sciwrite-prose/scripts` へのシンボリックリンクである。監査コピーではリンク先の文字列も通常ファイルとして保持し、実体の4本を対象 Skill 下に別途保存した。Codex へは実体のある対象3ディレクトリを導入すればよい。

**Codex向けに補う点：** upstream のシェル探索例は `.claude/skills` と `.agents/skills` を探索するが、`~/.codex/skills` は含まない。正式導入後にスクリプトを使う場合、次のように実在する絶対パスを指定する。Claude の探索ループや plugin コマンドをそのまま流用しない。

```bash
# 承認を得て導入した後の実行例。今回このコマンドは実行していない。
python3 ${LOCAL_USER_HOME}/.codex/skills/sciwrite-prose/scripts/numeric_consistency.py reports/paper.md --json
python3 ${LOCAL_USER_HOME}/.codex/skills/sciwrite-prose/scripts/keyword_consistency.py reports/paper.md --terms '白線抽出,操舵指令,車輪換算速度' --json
```

日本語原稿の品質確認では、元の日本語章構成を維持し、本文・図表・集計JSONとの突合を主とする。upstream の原文やコードを無断で日本語対応版に置き換える必要はない。将来英語稿を作る場合には、英語の機械的監査の適用範囲が広がる。

### 3.2 Claude Figure Skill

Pythonパッケージ `sciplot` 0.3.0 の必須依存は、Python≥3.10、numpy≥1.24、scipy≥1.10、matplotlib≥3.8、Pillow≥9である。build backend は setuptools≥68。任意依存は pypdf≥4、pytest≥7、LaTeX、MATLAB、draw.io desktop CLI である。

- `python/sciplot` の読取りで、主処理はグラフ・画像・図式の生成とファイル保存であった。
- `drawio.py` はローカルの draw.io CLI を `subprocess.run([...])` で呼ぶ。`shell=True` ではない。draw.io実行ファイルがなければ編集用 `.drawio` のみ保存してPDF/PNG変換を省略する。
- `examples/run_all.sh` は `conda run -n control` と `/Applications/MATLAB_R2026a.app/bin/matlab` を固定している。Linuxの現在の環境でそのまま動かさない。
- READMEの `ln -s ... ~/.claude/skills/...` は Claude Code 用である。Codexへの導入方法として採用しない。
- README の `pip install -e ...` は現在のPython環境を変更するため、今回実行していない。
- 作図例のデータは合成データであり、今回の競技評価に流用しない。

参考にできる方針は、最終印刷寸法での作図、単位付き軸、色と線種の併用、ベクトルPDFと確認用PNGの同時保存、レンダリングを見た修正である。これらは既存のPDF制作 Skill と標準 matplotlib でも実現できる。今回は比較手法や反復試行のないデータに、仮のベースラインや信頼区間を追加しない。日本語原稿では Times 指定をそのまま採用せず、日本語を表示できる既存フォントを用いる。

### 3.3 現在の環境（メタデータの読取り）

| 項目 | システム Python | Codex 同梱 Python |
|---|---|---|
| Python | 3.10.12 | 3.12.14 |
| numpy | 2.2.6 | 2.3.5 |
| matplotlib | 3.10.9 | 未導入 |
| scipy | 1.8.0 | 未導入 |
| Pillow | 9.0.1 | 13.0.0 |
| pypdf | 未導入 | 6.21.4 |
| reportlab | 3.6.8 | 4.4.9 |
| pytest | 6.2.5 | 未導入 |

この表は `importlib.metadata` による版情報であり、すべての組合せの実行確認を意味しない。`sci-figure` 必須の scipy≥1.10 はシステムPythonで満たさず、同梱Pythonには scipy/matplotlib がない。一方、numpyとmatplotlibのみを使う今回の標準作図には scipyを新規導入する理由がない。`drawio`、`matlab`、`pdflatex`、`lualatex`、`xelatex`、`pandoc`、`latexmk`、`typst` はこの調査時点のPATHに見当たらなかった。PDFは既存のReportLab等で作成できるため、TeX環境の追加は必須としない。

## 4. 承認後に実施した導入

推奨は、Codex付属の `skill-installer` による3件のみの版固定導入である。付属インストーラーのソースも読み、公開アーカイブの取得、展開先チェック、Skill 検証、未存在の宛先へのコピーが主な処理であることを確認した。`--method download` を指定して git/SSH へのフォールバックを避ける。既存宛先がある場合は中断し、上書きしない。

```bash
# ユーザーの明示承認後に実行済みの導入コマンド。
python3 ${LOCAL_USER_HOME}/.codex/skills/.system/skill-installer/scripts/install-skill-from-github.py \
  --repo dvdsosa/scientific-writing-skills \
  --ref 2eb42c9afb4c9edd4ad5c3a4ae9351e139bae0a0 \
  --method download \
  --path skills/sciwrite-manuscript skills/sciwrite-prose skills/sciwrite-process \
  --dest ${LOCAL_USER_HOME}/.codex/skills
```

変更範囲は `~/.codex/skills/sciwrite-manuscript`、`sciwrite-prose`、`sciwrite-process` の3ディレクトリ追加である。Python/npmパッケージの追加・更新、シェル設定、走行用プログラム、Git履歴は変更しない。導入後にリポジトリ直下の `LICENSE` と `NOTICE` を各Skillへ複製し、監査コピーとの一致を確認した。各Skillに `ORIGIN.json` を加え、監査コミット・導入日時・上流のパス・ハッシュ一覧への参照を記録した。上流Skillの実体19ファイルは監査コピーとSHA-256がすべて一致した。4本のスクリプトの存在を確認したが、論文への実行は行っていない。既存Skillの上書きは発生していない。

この導入案を事前提示し、ユーザーから3件を導入してよいという承認を得て実施した。ユーザー指定の `npx skills add ...` は対応方法として確認済みだが、今回は npm CLI の追加調査・依存取得を増やさないため推奨コマンドにはしない。`claude plugin install` は実行しない。

## 5. ライセンス・保存物

両リポジトリは監査版で MIT License である。科学執筆側の著作権表記は David Sosa-Trejo (2026)、作図側は Wavelix (2026) である。元の `LICENSE` を監査コピーに保持した。

科学執筆側の `NOTICE` は、MIT がコード・独自文章を対象とし、Stanford の講義そのものの権利を付与しないこと、独立した非提携プロジェクトであることを説明している。論文に講義の例文を転用せず、執筆手順の参考としてのみ利用する。

監査用保存物：

- `research/skill_audit/source_manifest.json`：92ファイル、533,502 bytes。取得元URL、commit、各ファイルSHA-256、元のGitモードを保持。
- `research/skill_audit/static_audit.json`：科学執筆4スクリプトの構文・import・関連I/O呼出しの静的確認結果。
- `research/skill_audit/scientific-writing-skills/source/`：README、原文SKILL、参照文書、コード、LICENSE、NOTICE。
- `research/skill_audit/claude-figure-skill/source/`：README、原文SKILL、参照文書、コード、依存定義、LICENSE。画像ギャラリーは再配布の必要がないため取得していない。
- 各リポジトリの `commit.json` と `tree.json`：版と元の構成を確認するための公開GitHubメタデータ。

## 6. 専用 Skill への引継ぎ

`autonomous-race-report` には、プロジェクト固有のソース選定、版・設定の特定、AUTO区間と画像の対応付け、車輪換算速度と実車速度の区別、無線スナップショットのキャッシュ・欠測の扱い、公式順位とユーザー報告の区別を記録する。論文一般の章立て・文章編集・レビュー工程は監査済みの科学執筆3件を参照し、丸ごと複製しない。

正式導入前または未導入の環境では、同梱した原文の参照パスを案内し、Skills がインストール済みであると誤記しない。上流の原文を変更する場合は、元のLICENSE・NOTICEを保持したうえで変更点を記録する。今回の分析に実際に使用しない ROS/SLAM/Stanley 等の機能を、この専用Skillに必須手順として追加しない。
