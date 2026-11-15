# 回収した旧受信入力の校正補助

元の `rpi_system/` から無変更で回収した `rc_calibrate.py`、`rc_receiver.py`、`rc_trigger_gesture.py`、`rc_pico_receiver.py` である。最終予選のDualSense入力・Pi直接出力とは別系列であり、最終稼働版の制御経路へ再導入していない。由来は `../RECOVERED_HELPERS_MANIFEST.json` を参照する。

校正補助は受信入力を観測する旧ツールであり、入力配線、機器、校正仕様を理解した環境向けである。Pico経路はpyserial、Pi経路はpigpio等を必要とする。当日の配備済み校正ツールと完全一致したこと、既存設定へ適用できること、実機で起動することは本保存作業では検証していない。環境変更、校正、実車操作は行っていない。

Python構文とローカルimport依存のみを静的に確認した。元プログラムを改造したり、架空の校正結果を生成したりするためのものではない。
