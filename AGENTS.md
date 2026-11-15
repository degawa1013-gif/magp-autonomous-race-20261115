# この保存リポジトリでの作業

技術報告書の作成・更新を行う際は、先に `reports/CREATIVE_DIRECTION.md`、`reports/verification.md` と `reports/skills/autonomous-race-report/SKILL.md` を読む。プロジェクト専用Skillは資料照合と再利用の手順であり、一般的な科学執筆は利用可能な `sciwrite-manuscript`、`sciwrite-prose`、`sciwrite-process` を使う。Skillのグローバル導入は前提としない。

- `source/system/` は予選記録と照合した保存版である。報告書の更新によって、この25ファイルを変更しない。別の開発を依頼された場合は、保存版と変更版の由来を分けて記録する。
- 再生成・検証は `reports/environment.md` のコマンドを用いる。論文生成時に車両を起動せず、依存の追加インストールを自動で行わない。
- 指令値、車輪換算値、実測値を区別する。知覚STOPと運転モードSTOP、予選順位と決勝成績を混同しない。
- 次大会の数値を今回の固定設定から作らない。原JSONL/ZIP、競技段階、実装ハッシュ、公式結果を新たに確認し、不明事項は `[要確認]` とする。
- 原ログ、無線識別子、認証情報、個人パスを追加保存する前に、保存範囲と配布範囲を確認する。
