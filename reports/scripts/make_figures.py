#!/usr/bin/env python3
"""Generate reproducible paper figures from reports/data, without vehicle access.

Existing environment: Python 3, numpy, matplotlib. No package installation.
All figure panels are 170 mm wide and export SVG, PDF and 300 dpi PNG.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import shutil

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "magp-paper-matplotlib"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D
from matplotlib.patches import Patch, Rectangle, FancyArrowPatch
from matplotlib.path import Path as PlotPath
import matplotlib.ticker as mticker
import numpy as np

MM = 1 / 25.4
COLOR = dict(steer="#00637B", throttle="#38495B", requested="#8B949D",
             speed="#956000", contact="#A23434", lost="#BE5844",
             TRACK="#4E8D91", LOST="#BE5844", DEGRADED="#C8953D", STOP="#818B96")


def hash_file(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_csv(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def column(rows, key):
    return np.array([float(r[key]) if r.get(key) not in (None, "") else np.nan for r in rows])


def configure_font():
    path = font_manager.findfont("Noto Sans CJK JP", fallback_to_default=False)
    font_manager.fontManager.addfont(path)
    plt.rcParams.update({
        "font.family": "Noto Sans CJK JP", "font.size": 8.5,
        "axes.labelsize": 8.5, "xtick.labelsize": 8, "ytick.labelsize": 8,
        "legend.fontsize": 8.0, "axes.linewidth": .65, "lines.linewidth": 1.0,
        "axes.spines.top": False, "axes.spines.right": False,
        "grid.color": "#DCE0E3", "grid.linewidth": .55,
        "pdf.fonttype": 42, "ps.fonttype": 42, "svg.fonttype": "none",
        "svg.hashsalt": "magp-race-paper-v1",
        "savefig.facecolor": "white", "figure.facecolor": "white",
        "axes.unicode_minus": False,
    })
    return path


def perception_intervals(rows):
    """Left-hold state, ending at last retained observation (zero last weight)."""
    intervals = []
    start = float(rows[0]["elapsed_s"])
    state = rows[0]["perception_mode"]
    for row in rows[1:]:
        t = float(row["elapsed_s"])
        if row["perception_mode"] != state:
            intervals.append((start, t, state))
            start, state = t, row["perception_mode"]
    intervals.append((start, float(rows[-1]["elapsed_s"]), state))
    return intervals


def state_band(ax, rows):
    for left, right, state in perception_intervals(rows):
        if right > left:
            ax.broken_barh([(left, right-left)], (0, 1), facecolors=COLOR[state], edgecolors="none")
    ax.set_ylim(0, 1)
    ax.set_yticks([])
    ax.set_ylabel("認識状態", labelpad=14)
    ax.spines["left"].set_visible(False)


def save(fig, name, output, entry, manifest):
    files = {}
    for ext in ("svg", "pdf", "png"):
        p = output / (name + "." + ext)
        metadata = {"Creator": "MAGP reproducible report figure"}
        if ext == "pdf":
            metadata.update(CreationDate=None, ModDate=None)
        elif ext == "svg":
            metadata["Date"] = None
        fig.savefig(p, dpi=300, metadata=metadata)
        files[ext] = {"path": p.name, "sha256": hash_file(p), "bytes": p.stat().st_size}
    entry["files"] = files
    entry["width_mm"] = round(fig.get_figwidth() / MM, 3)
    entry["height_mm"] = round(fig.get_figheight() / MM, 3)
    manifest["figures"][name] = entry
    plt.close(fig)


def trace_figure(rows, output, manifest, detail, zoom=False):
    elapsed = column(rows, "elapsed_s")
    contact = detail["visual_contact"]["barrier_fills_view_by_elapsed_s"]
    loss = detail["recognition_loss_elapsed_s"]
    recovery = detail["track_recovery_elapsed_s"]
    limits = (contact-5, contact+2) if zoom else (0, float(elapsed[-1]))
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(170*MM, 128*MM),
                             gridspec_kw={"height_ratios": [2.15, 1.65, 1.7, .55], "hspace": .15})
    fig.subplots_adjust(left=.13, right=.98, top=.92, bottom=.17)
    ax = axes[0]
    ax.plot(elapsed, column(rows, "steering_command_us"), color=COLOR["steer"], label="操舵PWM指令")
    ax.axhline(1530, color="#7D858E", lw=.8, ls="--", label="中立1530 µs")
    ax.set_ylabel("操舵指令\n[µs]")
    ax.set_ylim(1050, 2010)
    ax.set_yticks([1100, 1300, 1530, 1750, 1959])
    ax.legend(loc="upper left", ncol=2, frameon=False, bbox_to_anchor=(0, 1.29), borderaxespad=0)
    axes[1].plot(elapsed, column(rows, "requested_forward_percent"), color=COLOR["requested"],
                 lw=.8, ls=":", label="設定値")
    axes[1].plot(elapsed, column(rows, "forward_command_percent"), color=COLOR["throttle"], label="制限適用後の指令")
    axes[1].set_ylabel("前進指令\n[%]")
    axes[1].set_ylim(0, 60)
    axes[1].set_yticks([0, 20, 40, 60])
    axes[1].legend(loc="upper left", ncol=2, frameon=False)
    axes[2].plot(elapsed, column(rows, "encoder_speed_kph"), color=COLOR["speed"], lw=.75)
    axes[2].set_ylabel("車輪換算速度\n[km/h]")
    axes[2].set_ylim(0, 15 if zoom else 24)
    axes[2].set_yticks([0, 5, 10, 15] if zoom else [0, 5, 10, 15, 20])
    state_band(axes[3], rows)
    fig.legend(handles=[Patch(facecolor=COLOR[s], label=s) for s in ("TRACK", "LOST", "DEGRADED", "STOP")],
               loc="center", bbox_to_anchor=(.565, .065), ncol=4, frameon=False,
               columnspacing=1.2, handlelength=1.1, handleheight=.9)
    for a in axes[:3]:
        a.grid(axis="both")
        a.tick_params(axis="x", labelbottom=False, length=2)
        a.axvspan(loss, recovery, color="#E5E8EB", alpha=.5, lw=0, zorder=-2)
    for a in axes:
        a.set_xlim(*limits)
        a.axvline(contact, color=COLOR["contact"], ls=":", lw=1.0)
        if zoom:
            a.axvline(loss, color=COLOR["lost"], ls="--", lw=.8)
    axes[3].set_xlabel("最初の保存AUTO観測からの経過時間 [s]", labelpad=4)
    if zoom:
        axes[3].xaxis.set_major_locator(mticker.MultipleLocator(1))
        ax.annotate("白線喪失\n221.216 s", xy=(loss, 1661), xytext=(loss-1.15, 1860),
                    ha="right", va="center", fontsize=8.0, color=COLOR["lost"],
                    arrowprops=dict(arrowstyle="-", color=COLOR["lost"], lw=.7))
        ax.annotate("接触場面\n221.879 s", xy=(contact, 1530), xytext=(contact+.6, 1750),
                    ha="left", va="center", fontsize=8.0, color=COLOR["contact"],
                    arrowprops=dict(arrowstyle="-", color=COLOR["contact"], lw=.7))
        name = "qualifying_contact_detail"
        caption = "予選の接触場面前後の拡大。最初の保存AUTO観測を0秒とし、接触場面の5秒前から2秒後までを示す。破線は近傍白線点が消えたLOST開始、点線はバリアが画面を占めることを確認した保存観測である。"
    else:
        axes[3].set_xticks([0, 60, 120, 180, 240, round(float(elapsed[-1]), 2)])
        ax.text(.99, 1.13, "接触場面 221.879 s", color=COLOR["contact"],
                ha="right", va="center", transform=ax.transAxes, fontsize=8.0)
        name = "qualifying_full_timeline"
        caption = "予選の261.070秒の連続AUTO保存区間。操舵PWM指令、前進設定値と制限適用後の指令、車輪換算速度、および認識状態を示す。点線は開始221.879秒後の接触場面、淡灰色は主たるLOSTからTRACK復帰までの区間である。"
    fig.text(.13, .025, "実舵角は未計測。認識STOPでもAUTO継続。接触瞬間は未計測。", fontsize=8.0, color="#4E5965")
    save(fig, name, output, {
        "caption_ja": caption,
        "source_files": ["data/qualifying_timeline.csv", "data/run_metrics.json"],
        "conditions_ja": "1973保存観測。操舵は受理PWM指令、前進率はソフトウェア指令、速度は車輪回転からの換算値。状態帯は左端保持。",
        "limitations_ja": "実舵角・対地速度・物理横偏差を測定していない。線は保存観測間を接続したもので、未保存の制御フレームを実測したものではない。接触線は記録画像の注釈で、衝突センサの実測時刻ではない。",
        "x_limits_s": list(limits), "contact_annotation_s": contact,
        "line_loss_annotation_s": loss,
    }, manifest)


def latency_figure(rows, output, manifest):
    fig, axes = plt.subplots(1, 2, sharey=True, figsize=(170*MM, 83*MM))
    fig.subplots_adjust(left=.105, right=.98, top=.86, bottom=.24, wspace=.16)
    bar_colors = ["#617483", "#00637B"]
    width = .34
    for ax, condition, label in zip(axes, ("no_perception_load", "perception_load"),
                                     ("(a) 画像処理負荷なし", "(b) 画像処理負荷あり")):
        for i, version in enumerate(("before", "after")):
            row = next(r for r in rows if r["condition"] == condition and r["version"] == version)
            values = [float(row["pipe_to_fake_output_median_ms"]), float(row["pipe_to_fake_output_p95_ms"])]
            x = np.array([0, 1]) + (i-.5)*width
            bars = ax.bar(x, values, width=width, color=bar_colors[i],
                          label=("改修前" if i==0 else "改修後"), edgecolor="white", linewidth=.5,
                          hatch=("" if i==0 else "///"))
            for b, v in zip(bars, values):
                ax.text(b.get_x()+b.get_width()/2, v+.18, "%.3f" % v, fontsize=8.0,
                        ha="center", va="bottom")
        ax.set_xticks([0,1], ["中央値", "p95"])
        ax.set_title(label, fontsize=8.5, pad=16, loc="left")
        ax.set_ylim(0, 12)
        ax.set_yticks([0, 3, 6, 9, 12])
        ax.grid(axis="y", zorder=-1)
        ax.set_axisbelow(True)
    axes[0].set_ylabel("パイプ送信から模擬出力まで [ms]")
    fig.legend(handles=[Patch(facecolor=bar_colors[0], label="改修前"), Patch(facecolor=bar_colors[1], hatch="///", edgecolor="white", label="改修後")],
               loc="center", bbox_to_anchor=(.57, .15), ncol=2, frameon=False, fontsize=8.0)
    fig.text(.105, .08, "各条件200イベントのPi上ソフトウェア試験。Bluetooth・実GPIO・機械応答は含まない。", fontsize=8.0, color="#4E5965")
    save(fig, "manual_software_latency", output, {
        "caption_ja": "手動入力処理のソフトウェアベンチマーク。evdev形式のイベントをパイプ送信した時刻から、模擬出力先が更新された時刻までの中央値とp95を、改修前後・画像処理負荷の有無で比較した。",
        "source_files": ["data/manual_software_latency_benchmark.csv", "manual_event_latency_review_20261115/benchmark_before_pi.json", "manual_event_latency_review_20261115/benchmark_after_pi.json", "manual_event_latency_review_20261115/benchmark_before_load_pi.json", "manual_event_latency_review_20261115/benchmark_after_load_pi.json"],
        "conditions_ja": "各条件200イベント。Pi上の保存済みソフトウェア試験であり、本図の作成時に実機試験は再実行していない。p95は保存された実験集計値を利用。",
        "limitations_ja": "Bluetooth空中伝送、実GPIO/PWM、機械応答は測定対象外。負荷ありの測定では画像処理反復回数が異なる。独立反復の信頼区間はないため誤差棒を描いていない。",
    }, manifest)


def macro_figure(rows, output, manifest, detail):
    left = column(rows, "start_s")
    right = column(rows, "end_s")
    mid = (left+right)/2
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(170*MM, 105*MM),
                             gridspec_kw={"height_ratios": [1, 1.25], "hspace": .2})
    fig.subplots_adjust(left=.13, right=.98, top=.92, bottom=.2)
    axes[0].bar(mid, column(rows, "track_time_percent"), width=(right-left)*.86,
                facecolor="#609195", edgecolor="white", linewidth=.5)
    axes[0].set_ylim(0, 106)
    axes[0].set_yticks([0, 50, 100])
    axes[0].set_ylabel("TRACK推定時間率\n[%]")
    axes[1].plot(mid, column(rows, "camera_to_output_median_ms"), color=COLOR["steer"],
                 marker="o", markersize=3, label="中央値")
    axes[1].plot(mid, column(rows, "camera_to_output_p95_ms"), color=COLOR["throttle"],
                 ls="--", marker="s", markersize=3, label="p95")
    axes[1].set_ylabel("カメラから出力まで\n[ms]")
    axes[1].set_ylim(40, 95)
    axes[1].set_yticks([40, 60, 80])
    axes[1].legend(loc="upper left", ncol=2, frameon=False)
    for ax in axes:
        ax.set_xlim(0, right[-1])
        ax.grid(axis="y")
        ax.set_axisbelow(True)
        ax.axvline(detail["visual_contact"]["barrier_fills_view_by_elapsed_s"], color=COLOR["contact"], ls=":", lw=.9)
    axes[0].text(.99, 1.11, "点線：接触場面 221.879 s", color=COLOR["contact"],
                 transform=axes[0].transAxes, ha="right", fontsize=8.0)
    axes[1].set_xticks([0,30,60,90,120,150,180,210,240,round(right[-1],2)])
    axes[1].set_xlabel("最初の保存AUTO観測からの経過時間 [s]")
    fig.text(.13, .04, "30秒区間で集計。最終区間は21.070秒。TRACK推定時間率は認識正解率ではない。", fontsize=8.0, color="#4E5965")
    save(fig, "qualifying_macro_windows", output, {
        "caption_ja": "予選のマクロな時間変化。上段は30秒区間のTRACK推定時間率、下段は同じ区間に保存されたカメラからソフトウェア出力までの時間の中央値とp95である。各点は区間中央に配置した。",
        "source_files": ["data/qualifying_30s_bins.csv", "data/run_metrics.json"],
        "conditions_ja": "区間は[0,30),[30,60),…,[240,261.070449]秒。状態の左端保持で時間率を推定し、各区間の先頭から末尾の保存観測までを分母とする。点線は接触場面。",
        "limitations_ja": "TRACK判定の真偽を人手で全件ラベル付けしていないため、認識正解率ではない。区間内の設定速度・曲線条件は一定でなく、性能の因果比較には使用しない。",
    }, manifest)


def system_figure(output, manifest):
    fig, ax = plt.subplots(figsize=(170*MM, 141*MM))
    fig.subplots_adjust(left=.01, right=.99, top=.99, bottom=.01)
    ax.set_xlim(0,1); ax.set_ylim(0,1); ax.axis("off")
    def block(x,y,w,h,text,software=False):
        ax.add_patch(Rectangle((x,y),w,h,facecolor="#EDF3F5" if software else "white",
                               edgecolor="#657582",linewidth=.8))
        ax.text(x+w/2,y+h/2,text,ha="center",va="center",fontsize=8.0,linespacing=1.45)
    def arrow(points, dashed=False, both=False):
        path=PlotPath(points,[PlotPath.MOVETO]+[PlotPath.LINETO]*(len(points)-1))
        ax.add_patch(FancyArrowPatch(path=path,arrowstyle="<->" if both else "->",
                                    mutation_scale=8,linewidth=.8,color="#38495B",
                                    linestyle="--" if dashed else "-",zorder=1))
    block(.02,.86,.27,.10,"単眼カメラ\n320×240画像")
    block(.37,.86,.27,.10,"DualSense\nBluetooth接続")
    block(.72,.86,.26,.10,"車輪回転センサー\nGPIO17")
    block(.02,.61,.27,.14,"動的白線抽出・鳥瞰点\n画像誤差PD・遠方先読み\n指令出力率の上限管理",True)
    block(.37,.61,.27,.14,"Linux evdev\nモード・ギア・停止仲裁",True)
    block(.72,.61,.26,.14,"回転パルスの速度換算\nゲイン選択の一部へ",True)
    block(.32,.34,.36,.15,"唯一の出力ワーカー\nFastControlOutput\nイベント起床・10 ms待機で補完",True)
    block(.02,.34,.24,.15,"非同期診断記録\n画像・マスク・指令・時刻\n分割・容量制限",True)
    block(.75,.34,.23,.15,"Web／Even G2\nスマートフォン経由\n状態表示・モード要求")
    block(.34,.19,.32,.09,"pigpio hardware PWM\n実出力50 Hz",True)
    block(.04,.055,.34,.08,"操舵サーボ\nGPIO19（物理35番）")
    block(.63,.055,.35,.08,"ESC / 走行モーター\nGPIO12（物理32番）")
    arrow([(.155,.86),(.155,.75)])
    arrow([(.505,.86),(.505,.75)])
    arrow([(.85,.86),(.85,.75)])
    arrow([(.72,.68),(.69,.68),(.69,.79),(.255,.79),(.255,.75)],dashed=True)
    ax.text(.69,.812,"換算速度",fontsize=8.0,ha="right",color="#53636F")
    arrow([(.155,.61),(.155,.55),(.36,.55),(.36,.49)])
    ax.text(.18,.563,"自動計画",fontsize=8.0,ha="left")
    arrow([(.505,.61),(.505,.49)])
    arrow([(.09,.61),(.09,.49)],dashed=True)
    arrow([(.32,.415),(.26,.415)],dashed=True)
    arrow([(.68,.415),(.75,.415)],dashed=True)
    ax.text(.715,.44,"状態",fontsize=8.0,ha="center")
    arrow([(.865,.49),(.865,.55),(.62,.55),(.62,.61)],dashed=True)
    ax.text(.74,.565,"モード要求",fontsize=8.0,ha="center")
    arrow([(.50,.34),(.50,.28)])
    arrow([(.42,.19),(.42,.16),(.21,.16),(.21,.135)])
    arrow([(.59,.19),(.59,.16),(.805,.16),(.805,.135)])
    ax.text(.02,.015,"青灰色：車載Pi上の処理。G2表示と診断保存は、自動判断の往復経路に含まれない。",fontsize=8.0,color="#4E5965")
    save(fig,"system_architecture_qualifying",output,{
        "caption_ja":"保存予選版のシステム構成。カメラ由来の自動計画とDualSenseの入力を、画像処理から独立した唯一の出力ワーカーで仲裁し、操舵とESCへ50 HzのPWMを出力する。点線は状態・診断・モード要求および換算速度の経路を示す。",
        "source_files":["system_analysis.md","research/code_evidence.json","controller_hud_candidate/fast_control_output.py","controller_hud_candidate/legacy_pi_output.py"],
        "conditions_ja":"予選記録のsource SHA-256 159b6996facdd74e151967fc76e5f408d61ad805f57245340c70a31a1157b398と照合した構成。10 msは出力ワーカーの周期待機補完であり、PWM周期や遅延保証ではない。",
        "limitations_ja":"実装の依存関係を抽象化した構成図であり、信号線・電源の配線図ではない。決勝の稼働版は未照合。別系列のPure PursuitやROSを予選経路へ含めていない。",
    },manifest)


def contact_images_figure(reports, output, manifest, detail):
    from PIL import Image
    selected={r["frame"]:r for r in detail["selected_frames"]}
    copied=reports/"data/contact_frames";copied.mkdir(exist_ok=True)
    image_manifest=json.loads((copied/"source_manifest.json").read_text())
    image_sources={entry["frame"]:entry for entry in image_manifest["images"]}
    fig=plt.figure(figsize=(170*MM,81*MM))
    sources=[]
    descriptions=["右カーブで近傍8点を採用","白マスク0画素・近傍0点","バリアが前方画面を占める"]
    for i,(frame,description) in enumerate(zip((338,341,346),descriptions)):
        entry=image_sources[frame]
        source=reports.parent/entry["original_relative_path"]
        target=reports/entry["copy_relative_path"]
        # Raw archives are optional for a cached build; included JPEGs retain
        # the source digest so an absent archive never means skipping integrity.
        if source.is_file():
            if hash_file(source)!=entry["sha256"]:
                raise ValueError("Source contact frame digest mismatch: %s"%source)
            if source.resolve()!=target.resolve():
                shutil.copy2(source,target)
        if not target.is_file() or hash_file(target)!=entry["sha256"]:
            raise ValueError("Included contact frame missing or modified: %s"%target)
        r=selected[frame]
        left=.025+i*.328
        ax=fig.add_axes([left,.315,.307,.485])
        ax.imshow(Image.open(target));ax.axis("off")
        fig.text(left,.88,"(%s) %.3f s / %s"%(chr(97+i),r["elapsed_s"],r["perception_mode"]),fontsize=8.0,va="center")
        fig.text(left,.245,"操舵 %d µs / 前進 %.1f%%"%(r["steering_command_us"],r["forward_command_percent"]),fontsize=8.0)
        fig.text(left,.165,description,fontsize=8.0,color="#38495B")
        sources.append({"frame":frame,"copy_relative_path":entry["copy_relative_path"],"original_relative_path":entry["original_relative_path"],"sha256":hash_file(target),"elapsed_s":r["elapsed_s"],"steering_command_us":r["steering_command_us"],"forward_command_percent":r["forward_command_percent"]})
    fig.text(.025,.045,"経過秒は最初の保存AUTO観測を起点とする。実舵角・接触の瞬間は未計測。",fontsize=8.0,color="#4E5965")
    save(fig,"qualifying_contact_frames",output,{
        "caption_ja":"予選のカーブ接近から白線抽出喪失、バリア接近までの元カメラ画像。(a) s003フレーム338、(b)341、(c)346。画像下に同時刻の操舵PWM指令とソフトウェア前進指令を示す。",
        "source_files":["data/qualifying_timeline.csv","data/run_metrics.json","data/contact_frames/"],
        "conditions_ja":"元JPEG3枚を変更せずコピーし、SHA-256一致を確認した。白マスク0画素は保存マスクの計数であり、元画像に白線が物理的に存在しないという意味ではない。",
        "limitations_ja":"画像と同時刻の指令の対応を示すもので、実舵角や因果を実測したものではない。バリアが前方を占める画像を、力センサで計測した衝突の瞬間として扱わない。",
        "copied_images":sources,
    },manifest)


def radio_figure(reports,output,manifest):
    summary=json.loads((reports/"data/radio_event_summary.json").read_text())
    minutes=summary["minutes"]
    x=np.array([m["minute_of_day_jst"] for m in minutes],dtype=float)
    ap=np.array([m["pico_ap_count"]["mean"] if m["pico_ap_count"] else np.nan for m in minutes])
    fig,axes=plt.subplots(4,1,sharex=True,figsize=(170*MM,132*MM),gridspec_kw={"hspace":.16})
    fig.subplots_adjust(left=.14,right=.98,top=.91,bottom=.15)
    axes[0].plot(x,ap,color=COLOR["steer"],lw=.9)
    axes[0].set_ylabel("PC側Pico\n検出AP数")
    axes[0].set_ylim(0,22);axes[0].set_yticks([0,10,20])
    axes[0].text(650,10,"Pico測定前",ha="center",fontsize=8.0,color="#6C7781")
    configs=[("wifi_signal_dbm","車載Pi RSSI\n[dBm]",(-80,-10),[-75,-50,-25],COLOR["steer"]),
             ("gateway_rtt_ms","車載Pi 往復RTT\n[ms]",(0,450),[0,200,400],COLOR["throttle"]),
             ("bt_lq_raw_first_link","車載Pi BT LQ\n[0–255]",(0,265),[0,128,255],COLOR["speed"])]
    for ax,(key,label,limits,ticks,color) in zip(axes[1:],configs):
        y=np.array([m["pi_metrics"][key]["mean"] if m["pi_metrics"][key]["mean"] is not None else np.nan for m in minutes])
        ax.plot(x,y,color=color,lw=.9)
        ax.set_ylabel(label);ax.set_ylim(*limits);ax.set_yticks(ticks)
        for gap in summary["coverage"]["pi"]["missing_runs_between_first_and_last"]:
            def number(t):
                h,m=t.split(":");return int(h)*60+int(m)
            ax.axvspan(number(gap["start_jst"]),number(gap["end_jst_inclusive"])+1,color="#ECEFF1",lw=0,zorder=-2)
    for ax in axes:
        ax.set_xlim(590,950);ax.grid(axis="both");ax.tick_params(axis="x",length=2)
    axes[-1].set_xticks([600,660,720,780,840,900])
    axes[-1].set_xticklabels(["10:00","11:00","12:00","13:00","14:00","15:00"])
    axes[-1].set_xlabel("2026年11月15日の時刻 [JST]")
    fig.text(.14,.958,"Pico：12:01–15:47 / Pi：09:51–15:45。観測点が異なる。",fontsize=8.0,color="#38495B")
    fig.text(.14,.045,"各点は分平均。薄灰はPi欠測。RTT・BTはキャッシュ診断を含む。",fontsize=8.0,color="#4E5965")
    save(fig,"radio_event_timeline",output,{
        "caption_ja":"大会当日の無線診断の時間変化。上段はノートPC側Pico 2 Wで検出したAP数、下3段は車載Piの接続先Wi-FiのRSSI、ゲートウェイへの往復RTT、Bluetoothリンク品質指標LQである。すべて分平均であり、欠測区間を補間していない。",
        "source_files":["data/radio_event_summary.json"],
        "conditions_ja":"Pico1360スキャン、12:01–15:47の227分。Pi7297診断スナップショット、09:51–15:45の253観測分、同期間内102分欠測。薄灰色はPiに記録のない区間。RSSI等が個別に欠測する分も線を結ばない。",
        "limitations_ja":"Picoと車載Piは異なる観測点。RTTとBluetoothはキャッシュ値を繰り返し含み、独立試行や混線率ではない。LQ0–255はコントローラ定義の値で、成功率ではない。予選接触時11:32にPicoは測定開始前である。",
    },manifest)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reports", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    reports = args.reports.resolve()
    output = (args.out or reports / "figures").resolve()
    output.mkdir(parents=True, exist_ok=True)
    font = configure_font()
    metrics = json.loads((reports / "data/run_metrics.json").read_text())
    timeline = read_csv(reports / "data/qualifying_timeline.csv")
    assert len(timeline) == metrics["sessions"][metrics["qualifying_detail"]["session"]]["samples"]
    assert all(r["vehicle_mode"] == "AUTO" for r in timeline)
    manifest = {"schema": "race-paper-figure-metadata-v1", "font": font,
                "font_point_size": 8.5, "minimum_annotation_point_size": 8.0, "native_vector_formats": ["svg", "pdf"], "png_dpi": 300,
                "sources": {n: hash_file(reports / "data" / n) for n in ("run_metrics.json", "qualifying_timeline.csv", "qualifying_30s_bins.csv", "manual_software_latency_benchmark.csv")}, "figures": {}}
    manifest["sources"]["radio_event_summary.json"] = hash_file(reports / "data/radio_event_summary.json")
    manifest["system_analysis_sha256"] = hash_file(reports / "system_analysis.md")
    detail = metrics["qualifying_detail"]
    trace_figure(timeline, output, manifest, detail)
    trace_figure(timeline, output, manifest, detail, zoom=True)
    latency_figure(read_csv(reports / "data/manual_software_latency_benchmark.csv"), output, manifest)
    macro_figure(read_csv(reports / "data/qualifying_30s_bins.csv"), output, manifest, detail)
    system_figure(output,manifest)
    contact_images_figure(reports,output,manifest,detail)
    radio_figure(reports,output,manifest)
    (output / "figure_metadata.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+"\n")
    print(json.dumps({"figures": list(manifest["figures"]), "output": str(output), "font": font}, ensure_ascii=False))


if __name__ == "__main__":
    main()
