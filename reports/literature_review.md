# 採用技術に対応する先行資料・公式技術文書

確認日：2026年11月15日（日本標準時）。本調査は `system_analysis.md` で確認できた実装に限定する。L1–L9は予選保存稼働版の理論の一般説明とAPI・測定値の意味に対応する。元フォルダ再認識後に追加したL10は、別系列の追い抜き開発に存在するPure Pursuitの説明だけに用いる。プロジェクト独自のしきい値、操作仕様、性能、開発者の選定理由は内部ソース・ログで裏付け、外部文献から後付けしない。書誌と実装との対応は `research/literature_sources.json` に保存した。

## 1. 画像処理

**[L1] OpenCV, “Thresholding Operations using inRange.”** Lorena García（原著者）、OpenCV文書。取得版4.13.0。HSV各成分の範囲と画素の範囲検査による領域抽出を説明する。今回のHSV彩度条件と `cv2.inRange` の一般的背景に使う。床を基準に動的しきい値を作る本システム固有の規則は、この文書が提案した手法とはしない。公開年は確認できず、参照日を記す。[公式文書](https://docs.opencv.org/4.13.0/da/d97/tutorial_threshold_inRange.html)

**[L2] OpenCV, “Morphological Transformations.”** 二値画像と構造要素による形態処理を整理する。クロージングは膨張と収縮の組合せであり、小さな穴を埋める用途が説明される。`adaptive_white.py` 120行の3×3クロージングに対応する。太線内部を横方向の明度条件で埋める独自処理全体と同一視しない。公開年は未確認。[公式文書](https://docs.opencv.org/4.13.0/d9/d61/tutorial_py_morphological_ops.html)

**[L3] OpenCV, “Geometric Image Transformations.”** 四対の対応点から射影変換を求める関数と、変換先から変換元への画像再標本化を説明する。主制御の鳥瞰変換、および遠方観測の逆射影・remapに対応する。変換行列の使用だけで地面のメートル座標が校正されたことにはならない。公開年は未確認。[公式API文書](https://docs.opencv.org/4.13.0/da/d54/group__imgproc__transform.html)

## 2. 比例・微分制御とフィルタ

**[L4] Karl Johan Åström and Richard M. Murray, _Feedback Systems: An Introduction for Scientists and Engineers_, 2nd ed., Princeton University Press, 2021.** 著者管理サイトの第11章公開PDF（v3.1.5、2020年7月24日）を確認した。比例・微分作用と、微分が観測雑音を増幅し得ること、微分をフィルタと組み合わせる説明が、今回の画像誤差PDとD項平滑化の一般的背景に対応する。実装には積分項が確認できないため、採用方式はPID全体ではなくPDと記す。実装固有の基準周期補正・D項制限・先読み補助をこの書籍と同一の設計と断定せず、コードの式を示す。書籍の出版年は出版社で2021年を確認し、公開PDFの2020年と区別する。DOIは未確認のため記載しない。[著者公開章](https://www.cds.caltech.edu/~murray/books/AM08/pdf/fbs-pid_24Jul2020.pdf)、[出版社の書誌](https://press.princeton.edu/books/hardcover/9780691193984/feedback-systems)

本資料は、ふらつきが必ずPDゲインの問題であることや、今回の補助項が安定性を保証することを証明しない。車両応答モデル・遅延・路面条件の同定がないため、安定余裕の定量評価は未実施である。

## 3. 実行基盤と入力・出力

**[L5] The Linux kernel development community, “Input event codes.”** Linux入力プロトコルの一次文書である。値の変化時にイベントを出す状態保持型の仕組み、SYN_REPORTによる同時変化の区切り、SYN_DROPPED後の状態再取得を確認した。保持入力を「無イベントだから途絶」と判定しない処理と、レポート単位仲裁に対応する。読取スレッドの生存だけで無線パケット到達を証明できるわけではない。更新年は未確認。[Linux公式文書](https://docs.kernel.org/input/event-codes.html)

**[L6] Joan / pigpio contributors, “pigpio Python module: hardware_PWM,” author repository source documentation.** 開発者リポジトリの `pigpio.py` の関数ドキュメントを確認した。周波数と百万分率デューティを指定するAPI、およびGPIO12/18がPWM0、GPIO13/19がPWM1を共有する説明がある。`LegacyPiOutput` がGPIO12と19を別チャンネルとして使う根拠に対応する。50 Hzは本プロジェクトが設定した値であり、ライブラリ全体が50 Hz固定という意味ではない。公式Web APIページはHTTP 403だったため、開発者の公開ソースを一次資料として用いる。取得したmasterを当日実機のインストール版と同一とはしない。[開発者のソース文書](https://raw.githubusercontent.com/joan2937/pigpio/master/pigpio.py)

**[L7] Raspberry Pi Ltd, _The Picamera2 Library_, Release 7, 2026.** 2026年10月27日ビルド、文書が対象とするPicamera2は0.3.37である。queueパラメータの説明（PDF内18ページ）と、画像・metadataを同じcapture requestから取得しreleaseする説明（PDF内38ページ）を確認した。主制御の `queue=False` と `try/finally` によるrequest解放に対応する。この説明から制御遅延がゼロまたは一定以下になると主張しない。実機ライブラリ版は別途確認が必要である。[公式マニュアル](https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf)

## 4. 無線観測の意味と限界

**[L8] Linux Wireless documentation contributors, “About iw.”** 接続状態のsignal [dBm]、送受信レート、station統計の取得方法を確認した。`wifi_monitor.py` によるリンク状態取得の意味を説明する出典とする。受信強度、物理レート、AP検出数、応答時間は別の観測であり、当日の干渉量や操作の片道遅延へ自動換算しない。更新年は未確認。[Linux Wireless公式文書](https://wireless.docs.kernel.org/en/latest/en/users/documentation/iw.html)

**[L9] Bluetooth SIG, _Bluetooth Core Specification_, v6.1, Vol. 4, Part E, §§7.5.3–7.5.5.** Link Qualityは0–255で測り方はベンダー依存、BR/EDRのRSSIは受信機が選ぶ範囲に対する相対dB、AFH mapは79チャンネルの使用状態を表すビット列であると確認した。`bluetooth_monitor.py` の記録値の解釈に用いる。品質値をロス率へ換算せず、AFH mapを実測スペクトルと呼ばない。またmap読出しは直近の設定状態を返すため、実電波上で全候補の使用を観測したことにはならない。公開年は当該ページで確認できず、版と参照日を記す。[Bluetooth SIG公式仕様](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-61/out/en/host-controller-interface/host-controller-interface-functional-specification.html)

## 5. 別系列の探索的開発への追補

**[L10] R. Craig Coulter, _Implementation of the Pure Pursuit Path Tracking Algorithm_, Carnegie Mellon University, Technical Report CMU-RI-TR-92-01, January 1992.** 元フォルダ再認識後、`rpi_system/overtaking/controller.py` 130–137行にメートル単位の先読み点と `κ=2x/(x²+z²)` の実装を確認したため、既存 `OVERTAKING.md` が参照するCMUの一次資料を検証した。公式公開ページで著者、題名、年月、報告番号を確認した。これは別系列の追い抜き開発の位置付けに用い、予選の画像PD制御がPure Pursuitである、または当日追い抜きに成功したという根拠にしない。DOIは未確認のため記載しない。[CMU公式公開ページ](https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm/)

## 6. 予選稼働版に結び付けない技術・未確認の主張

Pure Pursuitや局所障害物回避の開発コードは別系列に存在するが、今回の予選保存稼働版への採用根拠はない。Stanley、SLAM、ROS/ROS 2も予選の制御説明の中心として引用しない。二次曲線を用いた画像経路近似や遠方の観測点利用だけでは、これらの採用を意味しない。予選版の先読みは画素座標と指令出力率に基づく追加補正であり、別系列で実装する標準Pure Pursuitの車両座標曲率式とは異なる。

内部の開発資料・過去レポートは、実装版と測定条件を照合した上で出典を明示して要約する。対照走行の条件、ラップ計時、実舵角、実軌跡の正解が欠ける場合、文献の一般論を根拠として性能向上率を補わない。今回の主要な技術内容は `system_analysis.md` と記録のソースハッシュが示す実装・運用上の統合であり、既存要素の新規発明を主張するものではない。

