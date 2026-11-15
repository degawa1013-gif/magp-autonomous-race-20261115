# パッケージ作成者用の手順

これは競技用の起動手順ではなく、保存・再解析・公開前確認の手順である。

1. 元の報告作業域で `python3 reports/scripts/prepare_release.py` を実行する。既定はdry-runで、選別結果と検出件数だけを表示する。
2. 承認済みの別保存先に反映する際は `--destination /path/to/new-repository --apply` を明示する。既存の`source/system/SOURCE_MANIFEST.json`と25本の原本hashが前提であり、走行コードを書き換えない。
3. 保存先へ移り、`python3 reports/scripts/rebuild.py --mode cached`を実行する。匿名化で文書/JSONのbyte hashは変わるため、図metadataと検証receiptをこの場所で更新する。
4. `reports/data/verification_results.json`とPDFの描画を確認する。元rawがないことを欠測とし、成功率や実舵角等を補わない。
5. `release_export_manifest.json`の選別・除外・匿名化とソース25本の一致を確認し、権利範囲と公開設定を別途確認する。Git作成、commit、push、外部送信はこのスクリプトの処理に含まれない。

`reports/data/source_inventory.json`は分類・代表資料だけの軽量な索引であり、個人環境の全ファイル台帳ではない。`${SOURCE_ROOT}`、`${ORIGINAL_PROJECT_ROOT}`、`${VEHICLE_PROJECT_ROOT}`は由来を示す文字列であり、自動探索や実機接続の指定ではない。原資料を読む工程では明示した`--source-root`を使う。
