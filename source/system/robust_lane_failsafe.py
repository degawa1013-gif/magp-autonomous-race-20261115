import os
import time
import glob
import json
import sys
import select
import math
import tty
import termios
import signal
import threading
import html
import zipfile
import tempfile
import copy
import hashlib
from urllib.parse import unquote, parse_qs, urlsplit
from dataclasses import dataclass, field
from collections import deque
from dualsense_control import ControlState, DualSenseReader
from telemetry_hud import compact_status
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from unittest.mock import MagicMock

import cv2
import numpy as np

from legacy_pi_output import LegacyPiOutput
from fast_control_output import FastControlOutput
from adaptive_white import adaptive_white_mask
from drive_capture import DriveCapture
from async_visuals import LatestWorker
from curve_pace import CurvePace, apply_forward_cap
from far_preview import extract_far_preview
from steering_preview import SteeringPreview
from return_damping import ReturnDamping

os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"
sys.modules["pykms"] = MagicMock()


# ============================================================
# Fail-safe line tracing for TT-02 / single front camera
#
# Priority:
#   1. Never trust one frame too much.
#   2. Hold verified steering briefly during visual loss, then recenter gently.
#   3. Reduce speed before trying to recover.
#   4. Prefer previous-frame continuity over a new isolated white blob.
#
# Main idea:
#   Reliable line observations determine steering. Keep the last verified
#   command for three untrusted frames, then move toward neutral at a bounded
#   rate. Weak points and edge blobs cannot choose a new turn during recovery.
#   A reliable line immediately restores the normal tracking controller.
# ============================================================


# --- Existing TT-02 PWM settings copied from rpi_drive.py ---
PWM_Hz = 70
STR_NEUTRAL = 10.00
SPD_NEUTRAL = 10.48
SPD_MIN = 7.413  # Calibrated forward endpoint: 1059 us.
SPD_MAX = 11.00
GPPIN_ACC = 12
GPPIN_STR = 19
GPPIN_ENCODER = 17
WHEEL_DIAMETER_M = 0.06
ENCODER_PULSES_PER_REV = 36.0
ENCODER_SPEED_SCALE = 1.0


# --- Conservative driving defaults ---
FRAME_W = 320
FRAME_H = 240
CENTER_X = FRAME_W // 2

SPEED_TRACK = SPD_NEUTRAL - (SPD_NEUTRAL - SPD_MIN) * 0.13
# Wheels-raised measurement on 2026-11-14: 1494us produced no rotation;
# 1470us produced 311 encoder pulses in two seconds. Road speed is uncalibrated.
SPEED_DEGRADED = 10.34    # slower while uncertain or hard turning
SPEED_LOST = 10.42        # very slow; still moving to reacquire
SPEED_STOP = SPD_NEUTRAL
SPEED_STEP = (SPD_NEUTRAL - SPD_MIN) * 0.05  # Five forward-output points.
SPEED_BOOST_MAX = SPEED_TRACK - SPD_MIN  # Retain the original starting speed.
REVERSE_SPEED = 11.88     # reverse; higher duty than neutral is reverse side
MANUAL_FORWARD_SPEED = SPEED_TRACK
MANUAL_REVERSE_SPEED = REVERSE_SPEED
MANUAL_SPEED_STEP = 0.05
MANUAL_FORWARD_SPEED_MIN = SPD_MIN
MANUAL_FORWARD_SPEED_MAX = SPD_NEUTRAL
MANUAL_REVERSE_SPEED_MIN = SPD_NEUTRAL
MANUAL_REVERSE_SPEED_MAX = 12.20
HIGH_SPEED_BOOST_THRESHOLD = 0.28
HIGH_SPEED_ENCODER_KPH_THRESHOLD = 2.0
HIGH_SPEED_ENCODER_PPS_THRESHOLD = 18.0
HIGH_SPEED_CURVE_STEER_THRESHOLD = 1.25
HIGH_SPEED_CURVE_SPEED = SPEED_DEGRADED
STRAIGHT_SPEED_MODE_ENABLED = True
STRAIGHT_SPEED_SCORE_THRESHOLD = 0.72
STRAIGHT_SPEED_CONFIDENCE_MIN = 0.90
STRAIGHT_SPEED_HOLD_SEC = 2.0
STRAIGHT_SPEED_EXTRA_BOOST = 0.25
STRAIGHT_SPEED_CURVE_SCORE_MAX = 0.45
STRAIGHT_SPEED_CURVE_SPEED = SPEED_DEGRADED
ENCODER_DEBOUNCE_US = 500
ENCODER_RATE_ALPHA = 0.72
ODOM_WHEELBASE_M = 0.26
ODOM_MAX_STEER_ANGLE_DEG = 28.0
ODOM_STEER_GAIN = 0.50
ODOM_STEER_DEADBAND = 0.00
ODOM_STEER_SIGN = -1.0
ODOM_VIEW_MARGIN_PX = 28
ODOM_VIEW_MIN_SCALE = 4.0
ODOM_VIEW_MAX_SCALE = 45.0
ODOM_TRAIL_MAX_POINTS = 3000
ODOM_DRAW_MAX_POINTS = 900
ODOM_MIN_DT = 0.001
ODOM_MAX_DT = 0.20

STEER_LIMIT = 3.00        # duty offset from STR_NEUTRAL
STEER_RATE = 0.80         # max duty change per frame
STEER_SMOOTH_ALPHA_STRAIGHT = 0.60
STEER_SMOOTH_ALPHA_CURVE = 1.0
STEER_SMOOTH_ALPHA_RECOVERY = 0.42
# The original filter coefficients describe a 30 Hz control loop. Real camera
# cadence varies, so purposeful corrections use elapsed time, while quiet input
# retains at least the existing noise suppression at low frame rates.
STEER_REFERENCE_DT = 1.0 / 30.0
STEER_MIN_DT = 0.005
STEER_MAX_DT = 0.30
HF_RESPONSE_START_PX = 4.0
HF_RESPONSE_FULL_PX = 10.0
HF_RESPONSE_TIME_SEC = 0.08
HF_RESPONSE_RELEASE_SEC = 0.15
UNRELIABLE_STEER_HOLD_FRAMES = 3
UNRELIABLE_STEER_RATE = 0.20  # Duty points per frame toward neutral after the hold.
MANUAL_STEER_LEFT = STR_NEUTRAL - STEER_LIMIT
MANUAL_STEER_RIGHT = STR_NEUTRAL + STEER_LIMIT
RECOVERY_STEER = 2.05     # held turn while lost
HARD_CURVE_STEER = 2.60

MAX_IMAGES = 80
ERROR_SAVE_INTERVAL_FRAMES = 12
OUTPUT_DIR = "output"
MASK_LOG_HZ = 4.0
PREVIEW_HZ = 10.0
MASK_LOG_VIDEO = "mask_log.mp4"
MASK_LOG_FOURCC = "avc1"
CONTROL_LOG_CSV = "control_log.csv"
THIN_LINE_ONLY = True
MAX_LINE_STROKE_WIDTH = 72.0
MIN_LINE_ELONGATION = 1.7
HLS_L_MIN = 145
WHITE_DELTA_MIN = 10
ROI_Y_TOP = 35
ROI_Y_BOTTOM = 200
HIST_Y_START = 120
HIST_Y_END = ROI_Y_BOTTOM
WINDOW_Y_TOP = ROI_Y_TOP
WINDOW_Y_BOTTOM = ROI_Y_BOTTOM
BODY_MASK_CENTER = (85, 200, 235, FRAME_H)
BODY_MASK_LEFT = (0, 205, 70, FRAME_H)
BODY_MASK_RIGHT = (250, 205, FRAME_W, FRAME_H)
HIST_MIN = 3000.0
HIST_SEARCH_MARGIN = 45
FULL_REACQUIRE_AFTER_FRAMES = 8
WINDOW_SEARCH_HALF_WIDTH = 40
WINDOW_MIN_PIXELS = 32
WINDOW_MAX_PIXELS = 2000
MAX_WINDOW_STEP = 24
MAX_PREDICT_ERROR_NEAR = 24
MAX_PREDICT_ERROR_FAR = 12
MAX_PREV_FRAME_ERROR = 30
MAX_CONSECUTIVE_REJECTS = 1
MAX_STEP_DELTA_NEAR = 22
MAX_STEP_DELTA_FAR = 12
MIN_REVERSAL_STEP = 6
SINGLE_POINT_OUTLIER_ERROR = 18
LINE_WIDTH_MIN = 4.0
LINE_WIDTH_MAX = 72.0
LINE_WIDTH_TOL_RATIO = 0.65
LINE_WIDTH_TOL_ABS = 7.0
CENTER_ERROR_GAIN = 0.025
CENTER_ERROR_D_GAIN = 0.1
SPEED_PD_GAIN_ENABLED = True
SPEED_PD_SWITCH_KPH = 4.0
# Previous good driving used the existing 0.010 gain at 23--28% commands.
# A missing wheel pulse must not restore 2.5 times the position correction.
# PWM percentage is only a gain-scheduling fallback, never measured road speed.
SPEED_PD_COMMAND_START_PERCENT = 13.0
SPEED_PD_COMMAND_FULL_PERCENT = 23.0
SPEED_PD_BLEND_TIME_SEC = 0.15
CENTER_ERROR_GAIN_HIGH_SPEED = 0.01
CENTER_ERROR_D_GAIN_HIGH_SPEED = 0.10
CENTER_ERROR_D_LIMIT = 18.0
CENTER_ERROR_D_FILTER_ALPHA = 0.65
CENTER_ERROR_BOOST_THRESHOLD = 14.0
CENTER_ERROR_BOOST_GAIN = 0.0
PATH_HEADING_GAIN = 0.085
PATH_FIT_BLEND_STRAIGHT = 0.88
PATH_FIT_BLEND_CURVE = 0.72
CONTROL_POINT_INDEX = 2
CONTROL_POINT_NEAR_INDEX = 1
CONTROL_POINT_FAR_INDEX = 5
LOOKAHEAD_Y_NEAR = 158.0
LOOKAHEAD_Y_BASE = 146.0
LOOKAHEAD_Y_FAR = 116.0
LOOKAHEAD_SPEED_MIN_BOOST = 0.12
LOOKAHEAD_SPEED_FULL_BOOST = 0.45
FORCE_FARTHEST_LOOKAHEAD = True
BOOST_MODE_LOOKAHEAD_BOOST = LOOKAHEAD_SPEED_FULL_BOOST
BOOST_MODE_FORCE_FARTHEST_LOOKAHEAD = True
LOOKAHEAD_CURVE_STEP_START = 4.0
LOOKAHEAD_CURVE_STEP_FULL = 14.0
CURVE_GAIN_SCALE_MAX = 1.22
HF_SUPPRESSION_ENABLED = True
HF_FILTER_ALPHA_STRAIGHT = 0.84
HF_FILTER_ALPHA_CURVE = 0.42
HF_SUPPRESSION_MAX = 0.82
STRAIGHT_DEADBAND_PX = 4.0
STRAIGHT_SCORE_ENABLE = 0.62
STRAIGHT_FIT_RMS_GOOD = 3.0
STRAIGHT_FIT_RMS_BAD = 12.0
STRAIGHT_STEP_STD_GOOD = 2.5
STRAIGHT_STEP_STD_BAD = 11.0
STRAIGHT_MEAN_STEP_GOOD = 3.0
STRAIGHT_MEAN_STEP_BAD = 18.0


# Bird-eye transform. Keep compatible with rpi_drive.py first.
SRC_PTS = np.float32([[70, 100], [250, 100], [320, 235], [0, 235]])
DST_PTS = np.float32([[80, 0], [240, 0], [240, 240], [80, 240]])
M_TRANS = cv2.getPerspectiveTransform(SRC_PTS, DST_PTS)
M_INV = cv2.getPerspectiveTransform(DST_PTS, SRC_PTS)


_legacy_pi_output = None
_legacy_pi_connection = None
fast_control_output = None

latest_jpeg = None
latest_status = {}
status_lock = threading.Lock()
current_run_dir = ""
current_run_name = ""
latest_mask_log_file = ""
latest_mask_log_frame_index = None
latest_mask_log_time = None
drive_capture = None
visual_worker = None
recording_build = {}
control_lock = threading.Lock()
pending_control_commands = deque()
vehicle_control = ControlState(auto_min_forward=SPEED_TRACK)
vehicle_control.stop("startup_stop")
dualsense_reader = None
MOTOR_TEST_LOCK = os.environ.get("MAGP_MOTOR_TEST_LOCK", "0") == "1"
STATIC_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "even_hud_dist")
HUD_WAIT_SEC = 0.15  # Pace G2 reads on the server; hidden WebView timers can be throttled.
STREAM_AUTO_HZ = 20.0  # Preserve the existing AUTO stream ceiling.
STREAM_OTHER_HZ = 5.0  # Native controller operation does not depend on video.
STREAM_WRITE_TIMEOUT_SEC = 2.0


def live_status_payload():
    """Overlay the independently applied controls on the latest camera frame.

    The frame timestamp and perception remain tied to the camera. The output
    sample carries its own clock; a stopped camera cannot delay manual status.
    """
    with status_lock:
        payload = dict(latest_status)
    worker = fast_control_output
    if worker is not None:
        sample = worker.status()
        if "steering" in sample and "throttle" in sample and sample.get("vehicle_control"):
            payload.update(vehicle_control=sample["vehicle_control"],
                           actuator_output=sample["actuator_output"],
                           output_steer=sample["steering"],
                           output_speed=sample["throttle"],
                           fast_control_output=sample)
            payload["forward_output_percent"] = round(clamp(
                (SPD_NEUTRAL - sample["throttle"]) / (SPD_NEUTRAL - SPD_MIN) * 100.,
                0., 100.), 1)
    return payload



INDEX_HTML = b"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robust Lane Failsafe</title>
  <style>
    body {
      margin: 0;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #111;
      color: #eee;
    }
    .page {
      display: grid;
      grid-template-columns: minmax(360px, 1fr) 330px;
      gap: 12px;
      padding: 12px;
      box-sizing: border-box;
    }
    .video {
      width: 100%;
      max-height: calc(100vh - 24px);
      object-fit: contain;
      background: #000;
      border: 1px solid #333;
    }
    .panel {
      display: flex;
      flex-direction: column;
      gap: 10px;
    }
    .box {
      border: 1px solid #333;
      background: #1b1b1b;
      padding: 10px;
    }
    .title {
      font-size: 16px;
      font-weight: 700;
      margin-bottom: 8px;
    }
    .mode {
      font-size: 28px;
      font-weight: 800;
      padding: 8px 10px;
      color: #111;
      background: #aaa;
    }
    .mode.TRACK { background: #35d07f; }
    .mode.DEGRADED { background: #ffd44d; }
    .mode.LOST { background: #ff7a3d; }
    .mode.STOP { background: #ff4d5a; }
    .mode.INIT { background: #9bc7ff; }
    .row {
      display: grid;
      grid-template-columns: 120px 1fr;
      gap: 8px;
      font-size: 14px;
      line-height: 1.55;
    }
    .label { color: #aaa; }
    .bar {
      height: 12px;
      background: #333;
      position: relative;
      overflow: hidden;
    }
    .fill {
      height: 100%;
      width: 0%;
      background: #35d07f;
    }
    .steer {
      height: 30px;
      position: relative;
      background: #2a2a2a;
      border: 1px solid #444;
    }
    .center {
      position: absolute;
      left: 50%;
      top: 0;
      width: 1px;
      height: 100%;
      background: #777;
    }
    .knob {
      position: absolute;
      top: 5px;
      width: 20px;
      height: 20px;
      transform: translateX(-50%);
      background: #ff4d5a;
    }
    .speedmeter {
      height: 18px;
      background: #2a2a2a;
      border: 1px solid #444;
      position: relative;
      overflow: hidden;
    }
    .speedfill {
      height: 100%;
      width: 0%;
      background: #35d07f;
    }
    .speedscale {
      display: flex;
      justify-content: space-between;
      color: #aaa;
      font-size: 12px;
      margin-top: 3px;
    }
    .forward-range {
      border: 1px solid #4a6855;
      background: #17241d;
      padding: 12px;
      margin: 10px 0;
    }
    #forwardOutput { font-size: 28px; font-variant-numeric: tabular-nums; }
    #forwardOutputBar {
      display: block;
      width: 100%;
      height: 18px;
      accent-color: #35d07f;
      margin: 10px 0;
    }
    .speed-note { color: #becac2; font-size: 12px; line-height: 1.6; }
    .controls {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
    }
    .controls button {
      min-height: 46px;
      border: 1px solid #444;
      background: #2a2a2a;
      color: #eee;
      font: inherit;
      font-weight: 700;
      touch-action: manipulation;
    }
    .controls button.primary { background: #276b46; }
    .controls button.danger { background: #74303a; }
    .controls button:active { filter: brightness(1.3); }
    code {
      color: #9bc7ff;
      word-break: break-all;
    }
    @media (max-width: 900px) {
      .page { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <div class="page">
    <img class="video" src="/stream.mjpg">
    <div class="panel">
      <div class="box">
        <div class="title">State</div>
        <div id="mode" class="mode INIT">INIT</div>
      </div>
      <div class="box">
        <div class="title">Control</div>
        <div class="forward-range">
          <div class="row"><div class="label">&#21069;&#36914;&#20986;&#21147;</div><strong id="forwardOutput">--</strong></div>
          <progress id="forwardOutputBar" max="100" value="0" aria-label="&#21069;&#36914;&#20986;&#21147;&#25351;&#20196;"></progress>
          <div class="speed-note">&#19978;&#38480;100%&#12290;1&#22238;&#12391;&#32004;5&#12509;&#12452;&#12531;&#12488;&#35519;&#25972;&#12290;&#25351;&#20196;&#20516;&#12391;&#12354;&#12426;&#12289;&#23455;&#36895;&#24230;&#12392;&#12399;&#30064;&#12394;&#12426;&#12414;&#12377;&#12290;</div>
          <div class="speed-note">Speed +/- adjusts the setting only. Use Drive ON to start. <a href="/controls.html">Open controller / G2 dashboard</a></div>
        </div>
        <div class="controls">
          <button class="primary" onclick="sendControl('drive_on')">Drive ON</button>
          <button class="danger" onclick="sendControl('neutral')">Neutral</button>
          <button onclick="sendControl('faster')">&#22679;&#36895; &#65291;</button>
          <button onclick="sendControl('slower')">&#28187;&#36895; &#8722;</button>

          <button class="primary" onclick="sendControl('boost_on')">Boost ON</button>
          <button class="danger" onclick="sendControl('boost_off')">Boost OFF</button>
        </div>
        <div class="title"><a href="/controls.html">DualSense / G2 controls</a></div>
        <div class="row"><div class="label">Drive</div><div id="enabled">false</div></div>
        <div class="row"><div class="label">Reverse</div><div id="reverse">false</div></div>
        <div class="row"><div class="label">Manual</div><div id="manual">false</div></div>
        <div class="row"><div class="label">Manual speed</div><div id="manualSpeed">-</div></div>
        <div class="row"><div class="label">Plan steer</div><div id="steer">0.00</div></div>
        <div class="row"><div class="label">Output steer</div><div id="outSteer">0.00</div></div>
        <div class="steer"><div class="center"></div><div id="steerKnob" class="knob"></div></div>
        <div class="row"><div class="label">Output speed</div><div id="speed">0.00</div></div>
        <div class="row"><div class="label">Speed boost</div><div id="speedBoost">0.00</div></div>
        <div class="row"><div class="label">Boost mode</div><div id="boostMode">false</div></div>
        <div class="row"><div class="label">Encoder</div><div id="encoder">0.0 Hz</div></div>
        <div class="row"><div class="label">Speedometer</div><div id="speedKph">0.00 km/h</div></div>
        <div class="speedmeter"><div id="speedFill" class="speedfill"></div></div>
        <div class="speedscale"><span>0</span><span>17.5</span><span>35 km/h</span></div>
        <div class="row"><div class="label">Speed policy</div><div id="speedPolicy">-</div></div>
        <div class="row"><div class="label">PWM</div><div id="pwm">-</div></div>
        <div class="row"><div class="label">Raw error</div><div id="rawError">0.0</div></div>
        <div class="row"><div class="label">Used error</div><div id="usedError">0.0</div></div>
        <div class="row"><div class="label">PD d-term</div><div id="pdTerm">0.0</div></div>
        <div class="row"><div class="label">Lookahead</div><div id="lookahead">-</div></div>
        <div class="row"><div class="label">Straight</div><div id="straightScore">0.00</div></div>
        <div class="row"><div class="label">HF suppress</div><div id="hfSuppress">0.00</div></div>
      </div>
      <div class="box">
        <div class="title">Detection</div>
        <div class="row"><div class="label">Confidence</div><div><span id="confidence">0.00</span></div></div>
        <div class="bar"><div id="confBar" class="fill"></div></div>
        <div class="row"><div class="label">Kind</div><div id="kind">none</div></div>
        <div class="row"><div class="label">Reason</div><div id="reason">-</div></div>
        <div class="row"><div class="label">Side</div><div id="side">0</div></div>
        <div class="row"><div class="label">Target x</div><div id="target">160</div></div>
        <div class="row"><div class="label">Lost frames</div><div id="lost">0</div></div>
        <div class="row"><div class="label">Mask ratio</div><div id="maskRatio">0.00</div></div>
        <div class="row"><div class="label">Gated ratio</div><div id="gatedRatio">0.00</div></div>
        <div class="row"><div class="label">Line width</div><div id="lineWidth">0.0</div></div>
      </div>
      <div class="box">
        <div class="title">Candidates</div>
        <pre id="candidates">[]</pre>
      </div>
      <div class="box">
        <div class="title">Endpoints</div>
        <div class="row"><div class="label">JSON</div><div><code>/status.json</code></div></div>
        <div class="row"><div class="label">Stream</div><div><code>/stream.mjpg</code></div></div>
        <div class="row"><div class="label">Output</div><div><a href="/output/">/output/</a></div></div>
      </div>
    </div>
  </div>
  <script>
    function fixed(v, n) {
      if (typeof v !== "number") return String(v);
      return v.toFixed(n);
    }
    async function sendControl(action) {
      try {
        await fetch("/control?action=" + encodeURIComponent(action), {
          method: "POST",
          cache: "no-store"
        });
        refresh();
      } catch (e) {
        document.getElementById("reason").textContent = "control failed";
      }
    }
    async function refresh() {
      try {
        const r = await fetch("/status.json", {cache: "no-store"});
        const s = await r.json();
        const mode = document.getElementById("mode");
        mode.textContent = s.mode || "INIT";
        mode.className = "mode " + (s.mode || "INIT");
        document.getElementById("enabled").textContent = String(s.enabled);
        document.getElementById("reverse").textContent = String(s.reverse_mode);
        document.getElementById("manual").textContent =
          String(s.manual_mode) + " / " + String(s.manual_drive);
        document.getElementById("manualSpeed").textContent =
          "fwd=" + fixed(s.manual_forward_speed, 3) +
          " back=" + fixed(s.manual_reverse_speed, 3);
        document.getElementById("steer").textContent = fixed(s.plan_steer, 3);
        document.getElementById("outSteer").textContent = fixed(s.output_steer, 3);
        const forward = Number.isFinite(s.forward_output_percent)
          ? Math.max(0, Math.min(100, s.forward_output_percent)) : null;
        document.getElementById("forwardOutput").textContent =
          forward === null ? "--" : forward.toFixed(1) + "%";
        document.getElementById("forwardOutputBar").value = forward === null ? 0 : forward;
        document.getElementById("speed").textContent = fixed(s.output_speed, 3);
        document.getElementById("speedBoost").textContent = fixed(s.speed_boost, 3);
        document.getElementById("boostMode").textContent =
          String(s.boost_mode_enabled) +
          " active=" + String(s.straight_speed_active) +
          " curve_cut=" + String(s.straight_speed_curve_cut) +
          " t=" + fixed(s.straight_speed_duration, 1) + "s";
        document.getElementById("encoder").textContent =
          fixed(s.encoder_pulse_rate_hz, 1) + " Hz / " + String(s.encoder_pulse_count) +
          " / " + fixed(s.encoder_speed_mps, 2) + " m/s " +
          fixed(s.encoder_speed_kph, 2) + " km/h";
        const speedKph = Math.max(0, Math.min(35, s.encoder_speed_kph || 0));
        document.getElementById("speedKph").textContent =
          fixed(s.encoder_speed_kph, 2) + " km/h";
        document.getElementById("speedFill").style.width =
          (speedKph / 35 * 100) + "%";
        document.getElementById("speedPolicy").textContent =
          "high=" + String(s.high_speed_active) +
          " curve_cut=" + String(s.high_speed_curve_speed_cut) +
          " boost=" + String(s.boost_mode_enabled) +
          " th=" + fixed(s.high_speed_encoder_kph_threshold, 1) + "km/h";
        document.getElementById("pwm").textContent =
          "str=" + s.steer_pwm + " acc=" + s.speed_pwm;
        document.getElementById("rawError").textContent =
          fixed(s.raw_center_error, 1) + " / filt " + fixed(s.filtered_center_error, 1);
        document.getElementById("usedError").textContent =
          fixed(s.used_center_error, 1) + " / hf " + fixed(s.high_freq_error, 1);
        document.getElementById("pdTerm").textContent =
          "d=" + fixed(s.d_center_error, 1) + " filt=" + fixed(s.filtered_d_center_error, 1) +
          " term=" + fixed(s.pd_derivative_term, 3) +
          " P=" + fixed(s.center_error_gain_active, 3) +
          " D=" + fixed(s.center_error_d_gain_active, 3) +
          " spdPD=" + String(s.speed_pd_active);
        document.getElementById("lookahead").textContent =
          "pos=" + fixed(s.control_point_position, 2) + " x=" + fixed(s.control_point_x, 1) +
          " y=" + fixed(s.control_point_y, 1) + " hdg=" + fixed(s.path_heading_step, 1);
        document.getElementById("straightScore").textContent =
          fixed(s.straight_score, 2) + " rms=" + fixed(s.fit_rms, 1);
        document.getElementById("hfSuppress").textContent =
          fixed(s.hf_suppression, 2) + " on=" + String(s.hf_suppression_enabled);
        document.getElementById("confidence").textContent = fixed(s.confidence, 2);
        document.getElementById("kind").textContent = s.kind || "none";
        document.getElementById("reason").textContent = s.reason || "-";
        document.getElementById("side").textContent = String(s.side);
        document.getElementById("target").textContent = String(s.target_x);
        document.getElementById("lost").textContent = String(s.lost_frames);
        document.getElementById("maskRatio").textContent = fixed(s.mask_ratio, 4);
        document.getElementById("gatedRatio").textContent = fixed(s.gated_ratio, 4);
        document.getElementById("lineWidth").textContent =
          fixed(s.line_width, 1) + " / exp " + fixed(s.expected_line_width, 1);
        document.getElementById("confBar").style.width =
          Math.max(0, Math.min(100, (s.confidence || 0) * 100)) + "%";
        const steerRange = 1.05;
        const steerOffset = Math.max(-steerRange, Math.min(steerRange, (s.output_steer || 10.0) - 10.0));
        document.getElementById("steerKnob").style.left =
          (50 + 50 * steerOffset / steerRange) + "%";
        document.getElementById("candidates").textContent =
          JSON.stringify(s.candidates || [], null, 2);
      } catch (e) {
        document.getElementById("reason").textContent = "status fetch failed";
      }
    }
    setInterval(refresh, 120);
    refresh();
  </script>
</body>
</html>
"""


class MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def json_response(self, payload, status=200):
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", len(body))
        self.end_headers()
        self.wfile.write(body)

    def serve_monitor_file(self, relative):
        path = os.path.realpath(os.path.join(STATIC_ROOT, relative))
        if not path.startswith(os.path.realpath(STATIC_ROOT) + os.sep) or not os.path.isfile(path):
            self.send_error(404)
            return
        content_type = "text/html; charset=utf-8" if path.endswith(".html") else "text/javascript; charset=utf-8" if path.endswith(".js") else "text/css; charset=utf-8"
        with open(path, "rb") as source:
            body = source.read()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", len(body))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        route = self.path.split("?", 1)[0]
        if route == "/diagnostics/mark":
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 2048:
                    raise ValueError("invalid body size")
                payload = json.loads(self.rfile.read(length))
                if not isinstance(payload, dict):
                    raise ValueError("expected object")
                label = payload.get("label", "response_issue")
                if not isinstance(label, str) or not 1 <= len(label) <= 100:
                    raise ValueError("invalid label")
                accepted = drive_capture is not None and drive_capture.mark(label)
                self.json_response({"ok": bool(accepted), "reason": None if accepted else "not_recording"})
            except (ValueError, TypeError, json.JSONDecodeError) as exc:
                self.json_response({"ok": False, "error": str(exc)}, 400)
            return
        if route in ("/hud/mode", "/hud/speed", "/hud/emergency-reset"):
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 2048:
                    raise ValueError("invalid body size")
                payload = json.loads(self.rfile.read(length))
                if not isinstance(payload, dict):
                    raise ValueError("expected object")
                request_id = payload.get("request_id")
                if not isinstance(request_id, str) or not 1 <= len(request_id) <= 100:
                    raise ValueError("request_id required")
                if route == "/hud/mode":
                    if payload.get("mode") not in ("AUTO", "MANUAL", "STEER_AUTO", "STOP"):
                        raise ValueError("invalid mode")
                    result = vehicle_control.request_mode(payload["mode"], request_id=request_id)
                    if not result.get("ok"):
                        rejected = dict(vehicle_control.snapshot(), reason=result.get("reason"))
                        result["reason_text"] = compact_status({}, rejected)["reason_text"]
                    self.json_response(result)
                elif route == "/hud/emergency-reset":
                    self.json_response(vehicle_control.clear_emergency(request_id=request_id))
                else:
                    delta = payload.get("delta")
                    if type(delta) is not int or delta not in (-1, 1):
                        raise ValueError("delta must be +1 or -1")
                    with control_lock:
                        pending_control_commands.append(("speed_setting", delta, request_id))
                    self.json_response({"ok": True, "accepted": True, "request_id": request_id})
            except (ValueError, TypeError, json.JSONDecodeError) as exc:
                self.json_response({"ok": False, "error": str(exc)}, 400)
            return
        if self.path.startswith("/control"):
            self.handle_control()
        else:
            self.send_error(404)

    def do_GET(self):
        route = self.path.split("?", 1)[0]
        if route == "/diagnostics/frame":
            query = parse_qs(urlsplit(self.path).query)
            try:
                frame_number = int(query.get("frame", [""])[0])
                kind = query.get("kind", ["image"])[0]
                if not 1 <= frame_number <= 1800 or kind not in ("image", "mask", "bird", "gated"):
                    raise ValueError("invalid frame or kind")
            except (ValueError, TypeError):
                self.json_response({"ok": False, "reason": "invalid_frame"}, 400)
                return
            recording = drive_capture.status() if drive_capture is not None else {}
            name = recording.get("archive_name")
            if not name or os.path.basename(name) != name or not name.startswith("magp-capture-"):
                self.json_response({"ok": False, "reason": "no_completed_recording"}, 404)
                return
            member = (f"images/{frame_number:06d}.jpg" if kind == "image" else
                      f"masks/{frame_number:06d}.png" if kind == "mask" else
                      f"extra_masks/{kind}/{frame_number:06d}.png")
            try:
                with zipfile.ZipFile(os.path.join(OUTPUT_DIR, "drive_capture", name)) as archive:
                    info = archive.getinfo(member)
                    if info.file_size > 2 * 1024 * 1024:
                        raise ValueError("image is too large")
                    body = archive.read(info)
            except (OSError, KeyError, ValueError, zipfile.BadZipFile):
                self.json_response({"ok": False, "reason": "frame_unavailable"}, 404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg" if kind == "image" else "image/png")
            self.send_header("Content-Length", len(body))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass
        elif route == "/diagnostics/latest.json":
            recording = drive_capture.status() if drive_capture is not None else {}
            name = recording.get("archive_name")
            if not name or os.path.basename(name) != name or not name.startswith("magp-capture-"):
                self.json_response({"ok": False, "reason": "no_completed_recording"}, 404)
                return
            path = os.path.join(OUTPUT_DIR, "drive_capture", name)
            try:
                with zipfile.ZipFile(path) as archive:
                    info = archive.getinfo("analysis.json")
                    if info.file_size > 512 * 1024:
                        raise ValueError("analysis is too large")
                    report = json.loads(archive.read(info))
                self.json_response({"ok": True,
                    "archive_url": recording["latest_archive_url"], "analysis": report})
            except (OSError, KeyError, ValueError, zipfile.BadZipFile) as exc:
                self.json_response({"ok": False, "reason": "report_unavailable", "error": str(exc)}, 404)
        elif route == "/diagnostics/status":
            payload = drive_capture.status() if drive_capture is not None else {
                "active": False, "frames": 0, "dropped": 0,
                "error": None, "latest_archive_url": None, "pending": False,
            }
            payload["pending"] = bool(payload.get("finalizing", False))
            with status_lock:
                payload["timings"] = dict(latest_status.get("loop_timing", {}))
            self.json_response(payload)
        elif route in ("/", "/controls.html") and os.path.isfile(os.path.join(STATIC_ROOT, "controls.html")):
            self.serve_monitor_file("controls.html")
        elif route in ("/hud/status", "/hud/status.json", "/hud/status/wait"):
            if route == "/hud/status/wait":
                # Never hold the status/control locks while waiting. Sample
                # afterward so a slow phone receives the newest output frame.
                time.sleep(HUD_WAIT_SEC)
            payload = live_status_payload()
            try:
                self.json_response(compact_status(payload, payload.get("vehicle_control", vehicle_control.snapshot())))
            except (BrokenPipeError, ConnectionResetError):
                pass  # A suspended/closed phone may cancel a read already in flight.
        elif route == "/g2.html" or route.startswith("/assets/"):
            self.serve_monitor_file(unquote(route).lstrip("/"))
        elif self.path in ("/", "/vision"):
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            self.wfile.write(INDEX_HTML)
        elif self.path.startswith("/control"):
            self.handle_control()
        elif self.path == "/output" or self.path.startswith("/output/"):
            self.serve_output_path()
        elif self.path == "/status.json":
            payload = live_status_payload()
            body = json.dumps(payload).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-length", len(body))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/stream.mjpg":
            previous_frame = None
            last_sent_at = None
            try:
                # This timeout is confined to the optional image connection.
                # A slow reader cannot keep its writer blocked indefinitely.
                self.connection.settimeout(STREAM_WRITE_TIMEOUT_SEC)
                self.send_response(200)
                self.send_header(
                    "Content-type", "multipart/x-mixed-replace; boundary=frame"
                )
                self.end_headers()
                while True:
                    mode = vehicle_control.snapshot().get("mode")
                    interval = 1.0 / (STREAM_AUTO_HZ if mode == "AUTO" else STREAM_OTHER_HZ)
                    now = time.monotonic()
                    if last_sent_at is None or now >= last_sent_at + interval:
                        # A bytes reference is immutable. Use this same sample
                        # for length and body even if the producer publishes
                        # another image during a socket write. Never queue old
                        # images or resend the same published frame.
                        frame = latest_jpeg
                        if frame is not None and frame is not previous_frame:
                            self.wfile.write(b"--frame\r\n")
                            self.send_header("Content-type", "image/jpeg")
                            self.send_header("Content-length", len(frame))
                            self.end_headers()
                            self.wfile.write(frame)
                            self.wfile.write(b"\r\n")
                            previous_frame = frame
                            last_sent_at = time.monotonic()
                    # No control/status lock is held during pacing or writes.
                    # Recheck mode and take only the newest image on each turn.
                    time.sleep(1.0 / STREAM_AUTO_HZ)
            except (OSError, ValueError):
                pass  # A timeout, closed phone, or broken image socket ends this client only.
            finally:
                self.close_connection = True

    def handle_control(self):
        action = ""
        if "?" in self.path:
            query = self.path.split("?", 1)[1]
            for part in query.split("&"):
                key, _, value = part.partition("=")
                if key == "action":
                    action = unquote(value)
                    break
        if action not in (
            "drive_on",
            "neutral",
            "faster",
            "slower",
            "reverse",
            "boost_on",
            "boost_off",
            "manual_on",
            "manual_off",
            "manual_left",
            "manual_right",
            "manual_center",
            "manual_forward",
            "manual_reverse",
            "manual_stop",
            "manual_forward_faster",
            "manual_forward_slower",
            "manual_reverse_faster",
            "manual_reverse_slower",
            "map_reset",
        ):
            self.send_error(400)
            return
        with control_lock:
            pending_control_commands.append(action)
        body = json.dumps({"ok": True, "action": action}).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-length", len(body))
        self.end_headers()
        self.wfile.write(body)

    def serve_output_path(self):
        output_root = os.path.realpath(os.path.join(os.getcwd(), OUTPUT_DIR))
        rel_url = self.path[len("/output"):].split("?", 1)[0]
        rel_path = unquote(rel_url).lstrip("/")
        requested = os.path.realpath(os.path.join(output_root, rel_path))

        if requested != output_root and not requested.startswith(output_root + os.sep):
            self.send_error(403)
            return
        # Completed capture archives are already final and must be streamed
        # verbatim; the older run archive builder only knows its legacy files.
        if rel_path.endswith(".zip") and os.path.isfile(requested):
            self.serve_output_file(requested)
            return
        if rel_path.endswith(".zip"):
            run_name = os.path.basename(rel_path[:-4])
            run_dir = os.path.realpath(os.path.join(output_root, run_name))
            if run_dir == output_root or not run_dir.startswith(output_root + os.sep):
                self.send_error(403)
                return
            if not os.path.isdir(run_dir):
                self.send_error(404)
                return
            self.serve_output_zip(run_dir, run_name)
            return
        if not os.path.exists(requested):
            self.send_error(404)
            return
        if os.path.isdir(requested):
            self.serve_output_dir(output_root, requested)
        else:
            self.serve_output_file(requested)

    def serve_output_dir(self, output_root, requested):
        rel_dir = os.path.relpath(requested, output_root)
        if rel_dir == ".":
            rel_dir = ""
        title = f"/output/{rel_dir}".rstrip("/") + "/"
        rows = []
        if requested != output_root:
            parent = os.path.dirname(rel_dir).replace(os.sep, "/")
            rows.append(f'<li><a href="/output/{html.escape(parent)}/">..</a></li>')
        for name in sorted(os.listdir(requested)):
            path = os.path.join(requested, name)
            href_rel = os.path.join(rel_dir, name).replace(os.sep, "/")
            href = "/output/" + href_rel + ("/" if os.path.isdir(path) else "")
            label = name + ("/" if os.path.isdir(path) else "")
            if requested == output_root and os.path.isdir(path):
                zip_href = "/output/" + html.escape(name) + ".zip"
                rows.append(
                    f'<li><a href="{html.escape(href)}">{html.escape(label)}</a> '
                    f'<a href="{zip_href}">[zip]</a></li>'
                )
            else:
                rows.append(f'<li><a href="{html.escape(href)}">{html.escape(label)}</a></li>')

        body = (
            "<!doctype html><html><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            f"<title>{html.escape(title)}</title>"
            "<style>body{font-family:system-ui,sans-serif;background:#111;color:#eee;padding:20px}"
            "a{color:#9bc7ff}li{line-height:1.8}</style></head><body>"
            f"<h1>{html.escape(title)}</h1><ul>{''.join(rows)}</ul></body></html>"
        ).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-type", "text/html; charset=utf-8")
        self.send_header("Content-length", len(body))
        self.end_headers()
        self.wfile.write(body)

    def serve_output_file(self, path):
        name = os.path.basename(path)
        ext = os.path.splitext(name)[1].lower()
        content_type = {
            ".csv": "text/csv; charset=utf-8",
            ".json": "application/json; charset=utf-8",
            ".jpg": "image/jpeg",
            ".jpeg": "image/jpeg",
            ".png": "image/png",
            ".txt": "text/plain; charset=utf-8",
        }.get(ext, "application/octet-stream")
        with open(path, "rb") as source:
            self.send_response(200)
            self.send_header("Content-type", "application/zip" if ext == ".zip" else content_type)
            self.send_header("Content-length", os.fstat(source.fileno()).st_size)
            if ext in (".csv", ".zip"):
                self.send_header("Content-Disposition", f'attachment; filename="{name}"')
            self.end_headers()
            try:
                while True:
                    chunk = source.read(256 * 1024)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
            except (BrokenPipeError, ConnectionResetError):
                pass

    def serve_output_zip(self, run_dir, run_name):
        with tempfile.NamedTemporaryFile(suffix=".zip", delete=False) as tmp:
            zip_path = tmp.name
        try:
            with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_STORED) as zf:
                for file_name in (CONTROL_LOG_CSV, MASK_LOG_VIDEO):
                    path = os.path.join(run_dir, file_name)
                    if os.path.exists(path):
                        zf.write(path, file_name)
            with open(zip_path, "rb") as f:
                data = f.read()
            name = f"{run_name}.zip"
            self.send_response(200)
            self.send_header("Content-type", "application/zip")
            self.send_header("Content-length", len(data))
            self.send_header("Content-Disposition", f'attachment; filename="{name}"')
            self.end_headers()
            self.wfile.write(data)
        finally:
            try:
                os.remove(zip_path)
            except OSError:
                pass


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    # Streaming clients must not prevent the CLI process from shutting down.
    daemon_threads = True


@dataclass
class Detection:
    points: list = field(default_factory=list)
    confidence: float = 0.0
    kind: str = "none"          # line, edge_blob, weak, none
    side: int = 0               # -1 left, +1 right, 0 unknown
    target_x: int = CENTER_X
    reason: str = ""
    mask: np.ndarray | None = None
    bird: np.ndarray | None = None
    gated: np.ndarray | None = None
    candidates: list = field(default_factory=list)
    line_width: float = 0.0
    white_stats: dict = field(default_factory=dict)
    far_preview: dict = field(default_factory=dict)


@dataclass
class TrackerState:
    mode: str = "INIT"          # INIT, TRACK, DEGRADED, LOST, STOP
    last_points: deque = field(default_factory=lambda: deque(maxlen=8))
    last_target_x: int = CENTER_X
    last_steer: float = STR_NEUTRAL
    last_reliable_steer: float = STR_NEUTRAL
    unreliable_steer_frames: int = 0
    last_side: int = 0
    lost_frames: int = 0
    good_frames: int = 0
    frame_index: int = 0
    control_dt: float = STEER_REFERENCE_DT
    steering_response_activity: float = 0.0
    steering_filter_alpha: float = HF_FILTER_ALPHA_STRAIGHT
    steering_derivative_unbounded: float = 0.0
    expected_line_width: float = 0.0
    bright_cutoff_reference: float = 0.0
    bright_reference_frame: int = -100000
    bright_floor_reference: float = 0.0
    raw_center_error: float = 0.0
    filtered_center_error: float = 0.0
    used_center_error: float = 0.0
    prev_used_center_error: float = 0.0
    d_center_error: float = 0.0
    filtered_d_center_error: float = 0.0
    pd_derivative_term: float = 0.0
    speed_pd_active: bool = False
    speed_pd_blend: float = 0.0
    gain_schedule_initialized: bool = False
    gain_schedule_target_blend: float = 0.0
    gain_schedule_command_percent: float = 13.0
    gain_schedule_source: str = "low_command"
    preview_controller: SteeringPreview = field(default_factory=SteeringPreview)
    steering_preview: dict = field(default_factory=dict)
    return_controller: ReturnDamping = field(default_factory=ReturnDamping)
    return_damping: dict = field(default_factory=dict)
    steering_autonomy_active: bool = True
    baseline_target_steer: float = STR_NEUTRAL
    pace_reference_steer: float = STR_NEUTRAL
    pace_reference_reliable_steer: float = STR_NEUTRAL
    pace_reference_ready: bool = False
    curve_pd_blend: float = 0.0
    active_center_error_gain: float = CENTER_ERROR_GAIN
    active_center_error_d_gain: float = CENTER_ERROR_D_GAIN
    high_freq_error: float = 0.0
    target_steer: float = STR_NEUTRAL
    steer_smoothing_alpha: float = 0.0
    straight_score: float = 0.0
    hf_suppression: float = 0.0
    fit_rms: float = 0.0
    step_std: float = 0.0
    mean_step: float = 0.0
    control_point_index: int = CONTROL_POINT_INDEX
    control_point_position: float = float(CONTROL_POINT_INDEX)
    control_point_x: float = float(CENTER_X)
    control_point_y: float = 0.0
    lookahead_blend: float = 0.0
    path_heading_step: float = 0.0
    path_fit_blend: float = 0.0
    straight_speed_start_time: float = 0.0
    straight_speed_duration: float = 0.0
    straight_speed_active: bool = False
    straight_speed_curve_cut: bool = False
    prev_status_values: dict = field(default_factory=dict)


@dataclass
class EncoderState:
    pulse_count: int = 0
    last_tick: int | None = None
    pulse_rate_hz: float = 0.0
    speed_mps: float = 0.0
    speed_kph: float = 0.0
    last_update_time: float = 0.0
    enabled: bool = False

    def update_speed(self):
        circumference = WHEEL_DIAMETER_M * float(np.pi)
        rev_per_sec = self.pulse_rate_hz / max(1.0, ENCODER_PULSES_PER_REV)
        self.speed_mps = rev_per_sec * circumference * ENCODER_SPEED_SCALE
        self.speed_kph = self.speed_mps * 3.6

    def pulse_callback(self, gpio, level, tick):
        if level != 1:
            return
        if self.last_tick is not None:
            dt_us = (tick - self.last_tick) & 0xFFFFFFFF
            if dt_us < ENCODER_DEBOUNCE_US:
                return
            instant_hz = 1_000_000.0 / max(1.0, float(dt_us))
            self.pulse_rate_hz = (
                self.pulse_rate_hz * ENCODER_RATE_ALPHA
                + instant_hz * (1.0 - ENCODER_RATE_ALPHA)
            )
            self.update_speed()
        self.last_tick = tick
        self.pulse_count += 1
        self.last_update_time = time.time()
        self.enabled = True

    def update_idle(self):
        if self.last_update_time and time.time() - self.last_update_time > 0.35:
            self.pulse_rate_hz *= 0.85
            self.update_speed()
            if self.pulse_rate_hz < 0.05:
                self.pulse_rate_hz = 0.0
                self.update_speed()


@dataclass
class DeadReckoningState:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0
    last_time: float = 0.0
    trail: deque = field(default_factory=lambda: deque(maxlen=ODOM_TRAIL_MAX_POINTS))

    def reset(self):
        self.x_m = 0.0
        self.y_m = 0.0
        self.yaw_rad = 0.0
        self.last_time = time.time()
        self.trail.clear()
        self.trail.append((0.0, 0.0))

    def update(self, output_steer, signed_speed_mps, now=None):
        if now is None:
            now = time.time()
        if self.last_time <= 0.0:
            self.last_time = now
            self.trail.append((self.x_m, self.y_m))
            return
        dt = clamp(now - self.last_time, ODOM_MIN_DT, ODOM_MAX_DT)
        self.last_time = now

        steer_norm = clamp(
            (output_steer - STR_NEUTRAL) / max(0.001, STEER_LIMIT),
            -1.0,
            1.0,
        )
        if abs(steer_norm) < ODOM_STEER_DEADBAND:
            steer_norm = 0.0
        steer_rad = (
            np.deg2rad(ODOM_MAX_STEER_ANGLE_DEG)
            * steer_norm
            * ODOM_STEER_GAIN
            * ODOM_STEER_SIGN
        )
        yaw_rate = signed_speed_mps / max(0.01, ODOM_WHEELBASE_M) * np.tan(steer_rad)

        self.yaw_rad = wrap_angle_rad(self.yaw_rad + float(yaw_rate * dt))
        self.x_m += float(signed_speed_mps * np.cos(self.yaw_rad) * dt)
        self.y_m += float(signed_speed_mps * np.sin(self.yaw_rad) * dt)
        position = (self.x_m, self.y_m)
        if not self.trail or self.trail[-1] != position:
            self.trail.append(position)


def duty100(rate: float) -> int:
    rate = max(7.5, min(13.0, rate))
    return int(rate * 10000)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap_angle_rad(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def rate_limit(target, prev, step):
    return clamp(target, prev - step, prev + step)


def manage_storage(run_dir="."):
    files = glob.glob(os.path.join(run_dir, "robust_error_*.jpg"))
    if len(files) > MAX_IMAGES:
        files.sort(key=os.path.getmtime)
        for f in files[: len(files) - MAX_IMAGES]:
            try:
                os.remove(f)
            except OSError:
                pass


def create_run_dir():
    timestamp = "run_" + time.strftime("%Y%m%d_%H%M%S")
    base_dir = os.path.join(os.getcwd(), OUTPUT_DIR)
    os.makedirs(base_dir, exist_ok=True)
    run_dir = os.path.join(base_dir, timestamp)
    suffix = 1
    while os.path.exists(run_dir):
        run_dir = os.path.join(base_dir, f"{timestamp}_{suffix:02d}")
        suffix += 1
    os.makedirs(run_dir)
    return run_dir, os.path.basename(run_dir)


def write_run_meta(run_dir, run_name):
    meta = {
        "run_name": run_name,
        "created_time": time.time(),
        "created_local": run_name,
        "cwd": os.getcwd(),
        "web": "http://<RaspberryPi IP>:8000/",
        "status_json": "http://<RaspberryPi IP>:8000/status.json",
        "notes": "Per-run output directory. Final archive contains control_log.csv and mask_log.mp4.",
        "mask_log_hz": MASK_LOG_HZ,
        "mask_log": "mask_log.mp4 contains BGR-encoded grayscale mask frames. Each frame is left=bird_mask,right=gated_mask.",
        "mask_log_fourcc": MASK_LOG_FOURCC,
        "sync_key": "frame_index",
    }
    path = os.path.join(run_dir, "run_meta.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)


def write_log_manifest(run_dir, run_name):
    manifest = {
        "format": "robust_lane_failsafe_run",
        "version": 1,
        "run_name": run_name,
        "sync_key": "frame_index",
        "csv": CONTROL_LOG_CSV,
        "video": MASK_LOG_VIDEO,
        "archive": f"{run_name}.zip",
        "mask_log": {
            "file": MASK_LOG_VIDEO,
            "fourcc": MASK_LOG_FOURCC,
            "hz": MASK_LOG_HZ,
            "layout": "left=bird_mask,right=gated_mask",
            "channels": "bgr_encoded_grayscale",
            "csv_reference_columns": [
                "mask_log_file",
                "mask_log_frame_index",
                "mask_log_time",
            ],
        },
        "csv_sync_columns": {
            "frame_index": "control frame index",
            "time": "control timestamp",
            "mask_log_file": "relative path to the mask video",
            "mask_log_frame_index": "latest frame index written to mask video",
            "mask_log_time": "timestamp of latest mask video write",
        },
    }
    path = os.path.join(run_dir, "log_manifest.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2)


def write_log_viewer(run_dir, run_name):
    csv_name = CONTROL_LOG_CSV
    html_text = f"""<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Run Viewer {run_name}</title>
<style>
body{{margin:0;background:#111;color:#eee;font-family:system-ui,sans-serif}}
.bar{{display:flex;gap:10px;align-items:center;padding:10px;background:#1b1b1b;border-bottom:1px solid #333;flex-wrap:wrap}}
button,input{{font:inherit}}
#wrap{{display:grid;grid-template-columns:1fr 360px;gap:10px;padding:10px}}
#mask{{width:100%;max-height:calc(100vh - 90px);object-fit:contain;background:#000;border:1px solid #333}}
.box{{background:#1b1b1b;border:1px solid #333;padding:10px}}
.row{{display:grid;grid-template-columns:150px 1fr;gap:8px;line-height:1.55;font-size:14px}}
.label{{color:#aaa}}
.speedmeter{{height:18px;background:#2a2a2a;border:1px solid #444;overflow:hidden;margin:8px 0 4px}}
.speedfill{{height:100%;width:0%;background:#35d07f}}
.speedscale{{display:flex;justify-content:space-between;color:#aaa;font-size:12px;margin-bottom:8px}}
pre{{white-space:pre-wrap;word-break:break-all;max-height:45vh;overflow:auto}}
@media(max-width:900px){{#wrap{{grid-template-columns:1fr}}}}
</style>
</head>
<body>
<div class="bar">
  <button id="load">Load CSV</button>
  <input id="idx" type="range" min="0" max="0" value="0" style="flex:1;min-width:220px">
  <span id="pos">0 / 0</span>
</div>
<div id="wrap">
  <video id="mask" controls src="{MASK_LOG_VIDEO}"></video>
  <div class="box">
    <div class="row"><div class="label">CSV</div><div>{csv_name}</div></div>
    <div class="row"><div class="label">frame_index</div><div id="frame">-</div></div>
    <div class="row"><div class="label">video</div><div>{MASK_LOG_VIDEO}</div></div>
    <div class="row"><div class="label">mask frame</div><div id="maskFile">-</div></div>
    <div class="row"><div class="label">mode</div><div id="mode">-</div></div>
    <div class="row"><div class="label">speed</div><div id="speedMeter">-</div></div>
    <div class="speedmeter"><div id="speedFill" class="speedfill"></div></div>
    <div class="speedscale"><span>0</span><span>17.5</span><span>35 km/h</span></div>
    <div class="row"><div class="label">target_x</div><div id="target">-</div></div>
    <div class="row"><div class="label">raw / used</div><div id="err">-</div></div>
    <div class="row"><div class="label">PD d-term</div><div id="pd">-</div></div>
    <div class="row"><div class="label">steer / pwm</div><div id="steer">-</div></div>
    <div class="row"><div class="label">straight / hf</div><div id="hf">-</div></div>
    <pre id="detail"></pre>
  </div>
</div>
<script>
const csvName = "{csv_name}";
let rows = [];
function parseCsv(text) {{
  const lines = text.trim().split(/\\r?\\n/);
  const header = lines.shift().split(",");
  return lines.map(line => {{
    const cells = [];
    let cur = "", q = false;
    for (let i=0;i<line.length;i++) {{
      const ch = line[i];
      if (ch === '"' && line[i+1] === '"') {{ cur += '"'; i++; }}
      else if (ch === '"') q = !q;
      else if (ch === "," && !q) {{ cells.push(cur); cur = ""; }}
      else cur += ch;
    }}
    cells.push(cur);
    const row = {{}};
    header.forEach((h,i)=>row[h]=cells[i] ?? "");
    return row;
  }});
}}
function show(i) {{
  if (!rows.length) return;
  const r = rows[i];
  document.getElementById("idx").value = i;
  document.getElementById("pos").textContent = `${{i+1}} / ${{rows.length}}`;
  document.getElementById("frame").textContent = r.frame_index || "-";
  document.getElementById("maskFile").textContent = r.mask_log_file || "-";
  document.getElementById("mode").textContent = r.mode || "-";
  const speedKph = Math.max(0, Math.min(35, Number(r.encoder_speed_kph || 0)));
  document.getElementById("speedMeter").textContent = `${{r.encoder_speed_kph || "-"}} km/h`;
  document.getElementById("speedFill").style.width = `${{speedKph / 35 * 100}}%`;
  document.getElementById("target").textContent = `${{r.target_x}} d=${{r.d_target_x}}`;
  document.getElementById("err").textContent = `${{r.raw_center_error}} / ${{r.used_center_error}}`;
  document.getElementById("pd").textContent = `${{r.d_center_error}} / ${{r.filtered_d_center_error}} / ${{r.pd_derivative_term}}`;
  document.getElementById("steer").textContent = `${{r.output_steer}} / ${{r.steer_pwm}}`;
  document.getElementById("hf").textContent = `${{r.straight_score}} / ${{r.hf_suppression}}`;
  document.getElementById("detail").textContent = JSON.stringify(r, null, 2);
}}
async function load() {{
  const text = await (await fetch(csvName, {{cache:"no-store"}})).text();
  rows = parseCsv(text);
  const idx = document.getElementById("idx");
  idx.max = Math.max(0, rows.length - 1);
  show(0);
}}
document.getElementById("load").onclick = load;
document.getElementById("idx").oninput = e => show(Number(e.target.value));
load().catch(e => document.getElementById("detail").textContent = String(e));
</script>
</body>
</html>
"""
    with open(os.path.join(run_dir, "log_viewer.html"), "w", encoding="utf-8") as f:
        f.write(html_text)


def make_mask_log_frame(bird, gated):
    gray = np.hstack((bird, gated))
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def create_mask_video_writer(run_dir):
    path = os.path.join(run_dir, MASK_LOG_VIDEO)
    fourcc = cv2.VideoWriter_fourcc(*MASK_LOG_FOURCC)
    frame_size = (FRAME_W * 2, FRAME_H)
    writer = cv2.VideoWriter(path, fourcc, MASK_LOG_HZ, frame_size, True)
    if not writer.isOpened():
        raise RuntimeError(
            f"mask video writer could not open: {path} fourcc={MASK_LOG_FOURCC}"
        )
    return writer, path


def write_run_archive(run_dir, run_name):
    output_root = os.path.dirname(run_dir)
    archive_path = os.path.join(output_root, f"{run_name}.zip")
    wanted = [
        (os.path.join(run_dir, CONTROL_LOG_CSV), CONTROL_LOG_CSV),
        (os.path.join(run_dir, MASK_LOG_VIDEO), MASK_LOG_VIDEO),
    ]
    with zipfile.ZipFile(archive_path, "w", compression=zipfile.ZIP_STORED) as zf:
        for path, arcname in wanted:
            if os.path.exists(path):
                zf.write(path, arcname)
    return archive_path


def apply_drive_roi(mask):
    roi = np.zeros_like(mask)
    roi[ROI_Y_TOP:ROI_Y_BOTTOM, :] = 255

    # Fixed body masks. These areas are never used for lane detection.
    x0, y0, x1, y1 = BODY_MASK_CENTER
    cv2.rectangle(roi, (x0, y0), (x1, y1), 0, -1)
    x0, y0, x1, y1 = BODY_MASK_LEFT
    cv2.rectangle(roi, (x0, y0), (x1, y1), 0, -1)
    x0, y0, x1, y1 = BODY_MASK_RIGHT
    cv2.rectangle(roi, (x0, y0), (x1, y1), 0, -1)
    return cv2.bitwise_and(mask, roi)


def fill_bright_line_interiors(lightness, delta_mask):
    """Fill qualifying horizontal gaps using vectorized run statistics."""
    roi_seed = delta_mask[ROI_Y_TOP:ROI_Y_BOTTOM] != 0
    padded = np.pad(roi_seed, ((0, 0), (1, 1)), mode="constant")
    start_y, starts = np.nonzero(padded[:, 1:] & ~padded[:, :-1])
    end_y, ends = np.nonzero(padded[:, :-1] & ~padded[:, 1:])
    if starts.size < 2:
        return delta_mask.copy()
    # Starts and ends are in the same raster order, one pair per white run.
    left = np.flatnonzero(
        (start_y[:-1] == start_y[1:])
        & (ends[1:] - starts[:-1] <= LINE_WIDTH_MAX * 2)
        & (starts[1:] > ends[:-1])
    )
    if not left.size:
        return delta_mask.copy()
    roi_lightness = lightness[ROI_Y_TOP:ROI_Y_BOTTOM]
    width = roi_lightness.shape[1]
    gap_starts = start_y[left] * width + ends[left]
    gap_ends = start_y[left + 1] * width + starts[left + 1]
    # Disjoint gaps are already sorted, so reduceat gives every gap minimum.
    boundaries = np.column_stack((gap_starts, gap_ends)).ravel()
    gap_minimum = np.minimum.reduceat(roi_lightness.ravel(), boundaries)[::2]
    bright = gap_minimum >= HLS_L_MIN
    if not np.any(bright):
        return delta_mask.copy()
    left, gap_starts, gap_ends, gap_minimum = (
        array[bright] for array in (left, gap_starts, gap_ends, gap_minimum)
    )
    run_lengths = ends - starts
    run_ids = np.repeat(np.arange(starts.size), run_lengths)
    values = roi_lightness[roi_seed]
    ordered = values[np.lexsort((values, run_ids))].astype(np.float32)
    offsets = np.cumsum(run_lengths) - run_lengths
    medians = (ordered[offsets + (run_lengths - 1) // 2]
               + ordered[offsets + run_lengths // 2]) * 0.5
    edge_level = np.minimum(medians[left], medians[left + 1])
    accepted = gap_minimum >= np.maximum(HLS_L_MIN, edge_level - WHITE_DELTA_MIN)
    if not np.any(accepted):
        return delta_mask.copy()
    changes = np.zeros(roi_seed.size + 1, dtype=np.int16)
    np.add.at(changes, gap_starts[accepted], 1)
    np.add.at(changes, gap_ends[accepted], -1)
    filled = np.cumsum(changes[:-1]).reshape(roi_seed.shape) > 0
    result = delta_mask.copy()
    result[ROI_Y_TOP:ROI_Y_BOTTOM][filled] = 255
    return result


def make_white_mask(frame):
    return adaptive_white_mask(frame, ROI_Y_TOP, ROI_Y_BOTTOM, LINE_WIDTH_MAX)[0]


def previous_frame_fit(state: TrackerState):
    if len(state.last_points) >= 3:
        pts = np.array(state.last_points[-1], dtype=np.float32)
        if len(pts) >= 3:
            return np.polyfit(pts[:, 1], pts[:, 0], 2)
    return None


def predict_previous_x(fit, y: int, fallback_target_x) -> int:
    if fit is not None:
        return int(clamp(fit[0] * y * y + fit[1] * y + fit[2], 0, FRAME_W - 1))
    return int(fallback_target_x)


def predicted_x_at_y(state: TrackerState, y: int) -> int:
    return predict_previous_x(previous_frame_fit(state), y, state.last_target_x)


def make_search_gate(state: TrackerState, lost_frames: int):
    """Limit search around the predicted line; widen gradually while lost."""
    gate = np.zeros((FRAME_H, FRAME_W), dtype=np.uint8)

    if state.mode in ("INIT", "STOP") or len(state.last_points) == 0:
        # Startup still avoids far/top areas where reflections are common.
        cv2.rectangle(gate, (25, 105), (FRAME_W - 25, FRAME_H), 255, -1)
        return gate

    base_margin = 42 if state.mode == "TRACK" else 62
    margin = int(clamp(base_margin + lost_frames * 12, 42, 135))

    for y0 in range(80, FRAME_H, 12):
        y1 = min(FRAME_H, y0 + 14)
        px = predicted_x_at_y(state, (y0 + y1) // 2)
        x0 = clamp(px - margin, 0, FRAME_W - 1)
        x1 = clamp(px + margin, 0, FRAME_W - 1)
        cv2.rectangle(gate, (x0, y0), (x1, y1), 255, -1)

    # If last curve direction is known, keep the edge area on that side open.
    # This is the key for edge blobs in sharp turns.
    if state.last_side < 0:
        cv2.rectangle(gate, (0, 95), (90 + lost_frames * 8, FRAME_H), 255, -1)
    elif state.last_side > 0:
        cv2.rectangle(
            gate,
            (FRAME_W - 90 - lost_frames * 8, 95),
            (FRAME_W, FRAME_H),
            255,
            -1,
        )
    return gate


def component_shape_metrics(component_mask):
    area = int(cv2.countNonZero(component_mask))
    if area == 0:
        return {"stroke_width": 999.0, "elongation": 0.0}

    # Connected-component crops can contain no background at their borders.
    # Padding prevents distanceTransform from reporting an unbounded width.
    padded = cv2.copyMakeBorder(component_mask, 1, 1, 1, 1, cv2.BORDER_CONSTANT, value=0)
    dist = cv2.distanceTransform(padded, cv2.DIST_L2, 3)
    stroke_width = float(np.max(dist) * 2.0)
    ys, xs = component_mask.nonzero()
    width = int(xs.max() - xs.min() + 1)
    height = int(ys.max() - ys.min() + 1)
    elongation = max(width, height) / max(1, min(width, height))
    return {"stroke_width": stroke_width, "elongation": float(elongation)}


def line_row_width(component_mask):
    """Measure tape across image rows, independently of strip height."""
    occupied = component_mask != 0
    rows = np.any(occupied, axis=1)
    if not np.any(rows):
        return 0.0
    left = np.argmax(occupied[rows], axis=1)
    right = occupied.shape[1] - np.argmax(occupied[rows, ::-1], axis=1)
    widths = np.sort(right - left)
    middle = len(widths) // 2
    if len(widths) % 2:
        return float(widths[middle])
    return (float(widths[middle - 1]) + float(widths[middle])) * 0.5


def trackable_line_mask(bird):
    """Keep longitudinal tape, grouping aligned pieces across small gaps."""
    def select_components(source):
        # Assess aligned pieces together across at most 16 px of interruption.
        # This is classification only; the dark gaps remain dark in the output.
        linked = cv2.morphologyEx(source, cv2.MORPH_CLOSE, np.ones((17, 1), np.uint8))
        count, labels, stats, _ = cv2.connectedComponentsWithStats(linked, 8)
        keep = np.zeros(count, dtype=np.uint8)
        for index in range(1, count):
            x, y, w, h, area = stats[index]
            if h < 40 or area < WINDOW_MIN_PIXELS:
                continue
            component = ((labels[y:y + h, x:x + w] == index)
                         & (source[y:y + h, x:x + w] != 0)).astype(np.uint8)
            width = line_row_width(component)
            if (LINE_WIDTH_MIN <= width <= LINE_WIDTH_MAX
                    and h / max(1.0, width) >= MIN_LINE_ELONGATION):
                keep[index] = 255
        return cv2.bitwise_and(keep[labels], source)

    source = cv2.compare(bird, 0, cv2.CMP_GT)
    selected = select_components(source)
    if np.any(selected):
        return selected
    # Fine bright floor texture can attach tape to a broad carpet component.
    # Only when no normal candidate survives, break short texture bridges;
    # then apply the same width/longitudinal tests to the remaining evidence.
    cleaned = cv2.morphologyEx(source, cv2.MORPH_OPEN, np.ones((13, 3), np.uint8))
    return select_components(cleaned)

def histogram_line_center(histogram, offset=0, preferred_x=CENTER_X):
    """Use a histogram plateau's centre, not its first (left) maximum."""
    if not histogram.size or float(np.max(histogram)) < HIST_MIN:
        return None
    selected = histogram >= max(HIST_MIN, float(np.max(histogram)) * 0.5)
    boundaries = np.flatnonzero(np.diff(np.r_[False, selected, False]))
    best = None
    for start, end in zip(boundaries[::2], boundaries[1::2]):
        weights = histogram[start:end].astype(np.float64)
        center = float(np.average(np.arange(start, end), weights=weights)) + offset
        score = float(np.max(weights)) * (
            1.0 - min(abs(center - preferred_x) / FRAME_W, 0.8)
        )
        if best is None or score > best[0]:
            best = (score, center)
    return int(round(best[1])) if best else None


def estimate_window_line_width(crop, expected_x):
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(crop, 8)
    if num_labels <= 1:
        return None

    best = None
    for i in range(1, num_labels):
        x, y, w, h, area = stats[i]
        if area < 12:
            continue
        component = ((labels[y:y + h, x:x + w] == i).astype(np.uint8)) * 255
        row_width = line_row_width(component)
        if not LINE_WIDTH_MIN <= row_width <= LINE_WIDTH_MAX:
            continue
        cx = float(centroids[i][0])
        distance = abs(cx - expected_x)
        score = area - distance * 22.0
        if best is None or score > best["score"]:
            metrics = {"elongation": max(w, h) / max(1, min(w, h))}
            best = {
                "score": score,
                "stroke_width": row_width,
                "elongation": metrics["elongation"],
                "centroid_x": cx,
                "area": int(area),
            }
    return best


def width_matches_expected(width, expected_width):
    if width < LINE_WIDTH_MIN or width > LINE_WIDTH_MAX:
        return False
    if expected_width <= 0:
        return True
    tolerance = max(LINE_WIDTH_TOL_ABS, expected_width * LINE_WIDTH_TOL_RATIO)
    return abs(width - expected_width) <= tolerance


def extrapolated_x_from_points(points, fallback_x):
    if len(points) >= 2:
        return int(clamp(points[-1][0] + (points[-1][0] - points[-2][0]), 0, FRAME_W - 1))
    if len(points) == 1:
        return int(points[-1][0])
    return int(fallback_x)


def window_prediction_tolerance(window_index):
    ratio = window_index / 5.0
    return int(MAX_PREDICT_ERROR_NEAR + (MAX_PREDICT_ERROR_FAR - MAX_PREDICT_ERROR_NEAR) * ratio)


def window_step_delta_tolerance(window_index):
    ratio = window_index / 5.0
    return int(MAX_STEP_DELTA_NEAR + (MAX_STEP_DELTA_FAR - MAX_STEP_DELTA_NEAR) * ratio)


def trajectory_step_is_continuous(points, candidate_x, window_index):
    if len(points) < 2:
        return True, 0, 0
    prev_step = points[-1][0] - points[-2][0]
    candidate_step = candidate_x - points[-1][0]
    step_delta = abs(candidate_step - prev_step)
    if step_delta > window_step_delta_tolerance(window_index):
        return False, candidate_step, step_delta
    if (
        window_index >= 2
        and abs(prev_step) >= MIN_REVERSAL_STEP
        and abs(candidate_step) >= MIN_REVERSAL_STEP
        and prev_step * candidate_step < 0
    ):
        return False, candidate_step, step_delta
    return True, candidate_step, step_delta


def remove_single_point_outliers(points):
    if len(points) < 5:
        return points
    filtered = [points[0]]
    for i in range(1, len(points) - 1):
        prev_x, _ = points[i - 1]
        x, _ = points[i]
        next_x, _ = points[i + 1]
        expected_x = (prev_x + next_x) * 0.5
        if abs(x - expected_x) > SINGLE_POINT_OUTLIER_ERROR:
            continue
        filtered.append(points[i])
    filtered.append(points[-1])
    return filtered


def score_component(x, y, w, h, area, state: TrackerState, metrics):
    cx = x + w / 2.0
    cy = y + h / 2.0
    pred = predicted_x_at_y(state, int(cy))
    distance = abs(cx - pred)
    touches_left = x <= 3
    touches_right = x + w >= FRAME_W - 4
    touches_edge = touches_left or touches_right
    bottom_near = y + h > FRAME_H * 0.72
    vertical_span = h / FRAME_H
    density = area / max(1, w * h)
    stroke_width = metrics["stroke_width"]
    elongation = metrics["elongation"]
    side = -1 if touches_left else (1 if touches_right else 0)
    edge_allowed = (
        touches_edge
        and side != 0
        and (state.last_side == 0 or side == state.last_side)
        and y + h > FRAME_H * 0.62
    )

    # Hard rejections before scoring. These are usually floor reflections,
    # wall edges, labels, or isolated white objects rather than tape.
    if area > 7000 and not edge_allowed:
        return 0.0, side
    if density > 0.90 and area > 1100 and not edge_allowed:
        return 0.0, side
    if w > 180 and h < 55:
        return 0.0, side
    if not edge_allowed and (h < 18 or w > 115):
        return 0.0, side
    if not bottom_near and area < 260:
        return 0.0, side
    if THIN_LINE_ONLY:
        if stroke_width > MAX_LINE_STROKE_WIDTH:
            return 0.0, side
        if elongation < MIN_LINE_ELONGATION and area > 180:
            return 0.0, side

    area_score = clamp((area - 60) / 900.0, 0.0, 1.0)
    dist_score = 1.0 - clamp(distance / 125.0, 0.0, 1.0)
    bottom_score = 1.0 if bottom_near else 0.45
    shape_score = clamp(vertical_span * 2.2, 0.0, 1.0)

    score = (
        0.30 * area_score
        + 0.34 * dist_score
        + 0.18 * bottom_score
        + 0.18 * shape_score
    )

    # Edge blob bonus only when it is consistent with previous curve direction.
    if edge_allowed:
        score += 0.16

    # Large bright rectangles are often floor reflection unless they are edge
    # connected or temporally predicted.
    if density > 0.78 and area > 1000 and not edge_allowed:
        score -= 0.26

    return clamp(score, 0.0, 1.0), side


def extract_points_from_mask(gated_mask, start_x):
    """Sliding-window extraction around a candidate, bottom to top."""
    points = []
    current_x = int(start_x)
    window_h = 24
    half_w = 42

    for win in range(7):
        y_high = FRAME_H - win * window_h
        y_low = max(80, y_high - window_h)
        x_low = max(0, current_x - half_w)
        x_high = min(FRAME_W, current_x + half_w)
        crop = gated_mask[y_low:y_high, x_low:x_high]
        ys, xs = crop.nonzero()
        if len(xs) >= 22:
            current_x = int(np.mean(xs) + x_low)
            points.append((current_x, (y_low + y_high) // 2))
        else:
            # Do not jump to another white object; stop this chain.
            break
    return points


def detect_line(frame, state: TrackerState):
    raw_mask, white_stats = adaptive_white_mask(frame, ROI_Y_TOP, ROI_Y_BOTTOM, LINE_WIDTH_MAX)
    def observed(detection):
        detection.white_stats = dict(white_stats)
        # The bright subset can omit dim distant tape. Follow the same
        # independently verified line in the full adaptive mask, without
        # changing any base detection points or confidence gates.
        if detection.kind == "line" and detection.confidence >= 0.90:
            detection.far_preview = extract_far_preview(raw_mask, detection.points, M_TRANS)
        return detection
    lightness = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)[:, :, 1]
    floor = white_stats["floor"]
    normal = None
    cutoff = max(white_stats["minimum"], white_stats["white_measured"] * (225.0 / 255.0))
    if (state.bright_cutoff_reference > 0 and state.bright_floor_reference > 0
            and state.frame_index - state.bright_reference_frame <= FULL_REACQUIRE_AFTER_FRAMES):
        # Floor brightness estimates exposure; line occlusion alone must not
        # lower the held white reference onto unchanged carpet.
        floor_ratio = floor / state.bright_floor_reference
        reference_white = state.bright_cutoff_reference * (255.0 / 225.0)
        white_ratio = white_stats["white_measured"] / max(1.0, reference_white)
        same_exposure = abs(floor_ratio - white_ratio) <= .12 * max(floor_ratio, white_ratio, .25)
        exposure_ratio = float(np.clip(white_ratio, .2, 2.0)) if same_exposure else 1.0
        cutoff = max(cutoff, state.bright_cutoff_reference * exposure_ratio)
        if not same_exposure:
            raw_mask, white_stats = adaptive_white_mask(frame, ROI_Y_TOP, ROI_Y_BOTTOM, LINE_WIDTH_MAX,
                                               reference_white, state.bright_floor_reference)
    if cutoff > 0:
        bright_mask = cv2.bitwise_and(raw_mask, cv2.inRange(lightness, cutoff, 255))
        if not np.array_equal(bright_mask, raw_mask):
            bright = detect_line_from_mask(bright_mask, state, enforce_width_continuity=False)
            if state.last_points and len(bright.points) < 6:
                # Recheck independently when the old prediction gate clips a
                # moved line, while retaining the better supported trajectory.
                independent = detect_line_from_mask(bright_mask, TrackerState(), enforce_width_continuity=False)
                if len(independent.points) > len(bright.points):
                    bright = independent
            supported = (
                bright.kind == "line" and len(bright.points) >= 4
                and bright.confidence >= 0.80
                and max(y for _, y in bright.points) - min(y for _, y in bright.points) >= 60
            )
            if supported:
                if len(bright.points) < 6:
                    normal = detect_line_from_mask(raw_mask, state)
                    # A complete dim tape path outranks a short bright patch
                    # edge. Bright partial paths still beat absent/weak floor
                    # evidence when the visible tape is clipped by motion.
                    if normal.kind == "line" and len(normal.points) >= 6 and normal.confidence >= 0.90:
                        return observed(normal)
                state.bright_cutoff_reference = cutoff
                state.bright_floor_reference = floor
                state.bright_reference_frame = state.frame_index
                return observed(bright)
    return observed(normal if normal is not None else detect_line_from_mask(raw_mask, state))


def detect_line_from_mask(raw_mask, state: TrackerState, enforce_width_continuity=True):
    bird = cv2.warpPerspective(raw_mask, M_TRANS, (FRAME_W, FRAME_H))
    bird = trackable_line_mask(bird)
    gated = bird.copy()
    histogram = np.sum(bird[HIST_Y_START:HIST_Y_END, :], axis=0)
    max_hist = float(np.max(histogram)) if histogram.size else 0.0
    candidates = []

    if max_hist < HIST_MIN:
        return Detection(
            confidence=0.0,
            kind="none",
            reason=f"hist_low:{max_hist:.0f}",
            mask=raw_mask,
            bird=bird,
            gated=gated,
        )

    prev_base_x = None
    if state.last_points:
        prev_base_x = int(state.last_points[-1][0][0])

    if prev_base_x is not None:
        search_min = max(0, prev_base_x - HIST_SEARCH_MARGIN)
        search_max = min(FRAME_W, prev_base_x + HIST_SEARCH_MARGIN)
        local_histogram = histogram[search_min:search_max]
        if len(local_histogram) and float(np.max(local_histogram)) >= HIST_MIN:
            lane_base = histogram_line_center(local_histogram, search_min, prev_base_x)
        elif state.lost_frames < FULL_REACQUIRE_AFTER_FRAMES:
            return Detection(
                confidence=0.0,
                kind="none",
                reason="hist_outside_prev_gate",
                mask=raw_mask,
                bird=bird,
                gated=gated,
            )
        else:
            lane_base = histogram_line_center(histogram, preferred_x=prev_base_x)
    else:
        lane_base = histogram_line_center(histogram)

    points = []
    width_samples = []
    current_x = lane_base
    window_count = 8
    window_h = max(1, (WINDOW_Y_BOTTOM - WINDOW_Y_TOP) // window_count)
    frame_expected_width = (
        0.0 if state.lost_frames >= FULL_REACQUIRE_AFTER_FRAMES
        else state.expected_line_width
    )
    rejected_in_row = 0
    # Every window uses the same previous frame; fit it once for this search.
    previous_fit = previous_frame_fit(state)

    for window in range(window_count):
        y_high = WINDOW_Y_BOTTOM - window * window_h
        y_low = max(WINDOW_Y_TOP, y_high - window_h)
        if y_high <= WINDOW_Y_TOP:
            break
        y_mid = (y_low + y_high) // 2
        predicted_x = extrapolated_x_from_points(points, current_x)
        prev_frame_x = (
            predict_previous_x(previous_fit, y_mid, state.last_target_x)
            if state.last_points else predicted_x
        )
        if state.last_points:
            predicted_x = int(round(predicted_x * 0.65 + prev_frame_x * 0.35))
        half_width = max(
            18, WINDOW_SEARCH_HALF_WIDTH - window * 3,
            int(frame_expected_width * 0.75 + 10) if frame_expected_width > 0 else 0,
        )
        x_low = max(0, predicted_x - half_width)
        x_high = min(FRAME_W, predicted_x + half_width)
        crop = bird[y_low:y_high, x_low:x_high]
        ys, xs = crop.nonzero()
        pixel_count = len(xs)
        width_info = estimate_window_line_width(crop, predicted_x - x_low)
        stroke_width = float(width_info["stroke_width"]) if width_info else 0.0
        candidate_x = int(width_info["centroid_x"] + x_low) if width_info else predicted_x
        predict_error = abs(candidate_x - predicted_x)
        prev_error = abs(candidate_x - prev_frame_x)
        predict_tolerance = window_prediction_tolerance(window)
        step_ok, candidate_step, step_delta = trajectory_step_is_continuous(
            points, candidate_x, window
        )
        if frame_expected_width <= 0 and width_samples:
            frame_expected_width = float(np.median(width_samples))
        accepted = (
            width_info is not None
            and WINDOW_MIN_PIXELS < width_info["area"] < WINDOW_MAX_PIXELS
            and (not enforce_width_continuity or width_matches_expected(stroke_width, frame_expected_width))
            and predict_error <= predict_tolerance
            and (not state.last_points or prev_error <= MAX_PREV_FRAME_ERROR)
            and step_ok
        )
        score = 1.0 if accepted else 0.15
        candidates.append(
            {
                "label": window,
                "bbox": (int(x_low), int(y_low), int(x_high - x_low), int(y_high - y_low)),
                "area": int(pixel_count),
                "score": float(score),
                "side": -1 if predicted_x < CENTER_X - 12 else (1 if predicted_x > CENTER_X + 12 else 0),
                "centroid": (float(candidate_x), float(y_mid)),
                "stroke_width": round(stroke_width, 1),
                "elongation": round(float(width_info["elongation"]), 2) if width_info else 0.0,
                "predict_error": int(predict_error),
                "prev_error": int(prev_error),
                "step": int(candidate_step),
                "step_delta": int(step_delta),
            }
        )
        if accepted:
            if abs(candidate_x - current_x) <= MAX_WINDOW_STEP:
                current_x = candidate_x
                points.append((current_x, y_mid))
                width_samples.append(stroke_width)
                rejected_in_row = 0
            else:
                rejected_in_row += 1
        else:
            rejected_in_row += 1
        if rejected_in_row > MAX_CONSECUTIVE_REJECTS and len(points) >= 2:
            break

    points = remove_single_point_outliers(points)

    if len(points) >= 3:
        target_x = points[min(1, len(points) - 1)][0]
        confidence = clamp(0.48 + 0.08 * len(points), 0.0, 0.98)
        side = -1 if target_x < CENTER_X - 12 else (1 if target_x > CENTER_X + 12 else 0)
        kind = "line"
        reason = "tt02_sliding_window"
        line_width = float(np.median(width_samples)) if width_samples else 0.0
    elif len(points) > 0:
        target_x = points[-1][0]
        confidence = clamp(0.18 + 0.08 * len(points), 0.18, 0.34)
        side = -1 if target_x < CENTER_X - 12 else (1 if target_x > CENTER_X + 12 else 0)
        kind = "weak"
        reason = f"few_window_points:{len(points)}"
        line_width = float(np.median(width_samples)) if width_samples else 0.0
    else:
        return Detection(
            confidence=0.0,
            kind="none",
            reason="no_valid_window",
            mask=raw_mask,
            bird=bird,
            gated=gated,
            candidates=candidates,
        )

    return Detection(
        points=points,
        confidence=confidence,
        kind=kind,
        side=side,
        target_x=int(clamp(target_x, 0, FRAME_W - 1)),
        reason=reason,
        mask=raw_mask,
        bird=bird,
        gated=gated,
        candidates=candidates,
        line_width=line_width,
    )


def update_state(state: TrackerState, det: Detection):
    state.frame_index += 1
    reacquiring_width = state.lost_frames >= FULL_REACQUIRE_AFTER_FRAMES

    if det.confidence >= 0.62 and det.kind == "line":
        state.mode = "TRACK"
        state.lost_frames = 0
        state.good_frames += 1
    elif det.confidence >= 0.28:
        state.mode = "DEGRADED"
        state.lost_frames = max(0, state.lost_frames - 1)
        state.good_frames = 0
    else:
        state.lost_frames += 1
        state.good_frames = 0
        state.mode = "LOST" if state.lost_frames <= 18 else "STOP"

    if state.lost_frames == FULL_REACQUIRE_AFTER_FRAMES:
        # Expire the prediction gate as well as widening the histogram
        # search. Retain the steering/side used by recovery control.
        state.last_points.clear()

    if det.confidence >= 0.28:
        state.last_target_x = det.target_x
        if det.points:
            state.last_points.append(det.points)
        if det.side != 0:
            state.last_side = det.side
        if det.line_width > 0:
            if state.expected_line_width <= 0 or reacquiring_width:
                state.expected_line_width = det.line_width
            else:
                state.expected_line_width = state.expected_line_width * 0.82 + det.line_width * 0.18


def point_position_for_y(points, target_y):
    if len(points) < 2:
        return 0.0
    ordered = sorted(
        ((float(p[1]), float(i)) for i, p in enumerate(points)),
        reverse=True,
    )
    if target_y >= ordered[0][0]:
        return ordered[0][1]
    if target_y <= ordered[-1][0]:
        return ordered[-1][1]
    for (y0, i0), (y1, i1) in zip(ordered, ordered[1:]):
        if y0 >= target_y >= y1:
            span = max(1.0, y0 - y1)
            blend = (y0 - target_y) / span
            return i0 * (1.0 - blend) + i1 * blend
    return ordered[-1][1]


def interpolated_point(points, position, fallback_x):
    if not points:
        return float(fallback_x), 0.0
    position = clamp(float(position), 0.0, float(len(points) - 1))
    lo = int(np.floor(position))
    hi = min(len(points) - 1, lo + 1)
    blend = position - lo
    x = points[lo][0] * (1.0 - blend) + points[hi][0] * blend
    y = points[lo][1] * (1.0 - blend) + points[hi][1] * blend
    return float(x), float(y)


def center_error_from_control_point(state: TrackerState):
    return float(state.control_point_x - CENTER_X)


def fitted_path_point(points, position, target_y, fallback_target_x, curve_strength):
    interp_x, interp_y = interpolated_point(points, position, fallback_target_x)
    if len(points) < 4:
        return interp_x, interp_y, 0.0, 0.0

    idxs = np.arange(len(points), dtype=np.float32)
    xs = np.array([p[0] for p in points], dtype=np.float32)
    try:
        coeff = np.polyfit(idxs, xs, 2)
        fit_x = float(np.polyval(coeff, position))
        heading_step = float(2.0 * coeff[0] * position + coeff[1])
    except Exception:
        return interp_x, interp_y, 0.0, 0.0

    min_x = float(np.min(xs) - 18.0)
    max_x = float(np.max(xs) + 18.0)
    fit_x = clamp(fit_x, min_x, max_x)
    blend = (
        PATH_FIT_BLEND_STRAIGHT * (1.0 - curve_strength)
        + PATH_FIT_BLEND_CURVE * curve_strength
    )
    control_x = interp_x * (1.0 - blend) + fit_x * blend
    return float(control_x), float(target_y), heading_step, float(blend)


def choose_control_point(
    state: TrackerState,
    points,
    metrics,
    speed_boost,
    fallback_target_x,
    force_last_point=False,
):
    if not points:
        state.control_point_index = 0
        state.control_point_position = 0.0
        state.control_point_x = float(fallback_target_x)
        state.control_point_y = 0
        state.lookahead_blend = 0.0
        state.path_heading_step = 0.0
        state.path_fit_blend = 0.0
        return

    straight_score = metrics["straight_score"] if state.mode == "TRACK" else 0.0
    speed_blend = clamp(
        (speed_boost - LOOKAHEAD_SPEED_MIN_BOOST)
        / max(0.01, LOOKAHEAD_SPEED_FULL_BOOST - LOOKAHEAD_SPEED_MIN_BOOST),
        0.0,
        1.0,
    )
    curve_from_steps = clamp(
        (abs(metrics["mean_step"]) - LOOKAHEAD_CURVE_STEP_START)
        / max(0.01, LOOKAHEAD_CURVE_STEP_FULL - LOOKAHEAD_CURVE_STEP_START),
        0.0,
        1.0,
    )
    curve_blend = max(1.0 - straight_score, curve_from_steps)
    far_blend = speed_blend * (1.0 - curve_blend)
    if FORCE_FARTHEST_LOOKAHEAD or (
        BOOST_MODE_FORCE_FARTHEST_LOOKAHEAD
        and speed_boost >= LOOKAHEAD_SPEED_FULL_BOOST
    ):
        far_blend = 1.0
        curve_blend = 0.0

    if force_last_point:
        position = float(max(0, len(points) - 2))
        target_y = float(points[int(position)][1])
        far_blend = 1.0
        curve_blend = 0.0
    else:
        target_y = (
            LOOKAHEAD_Y_BASE
            + (LOOKAHEAD_Y_FAR - LOOKAHEAD_Y_BASE) * far_blend
            + (LOOKAHEAD_Y_NEAR - LOOKAHEAD_Y_BASE) * curve_blend
        )
        target_y = clamp(target_y, WINDOW_Y_TOP, WINDOW_Y_BOTTOM)
        position = point_position_for_y(points, target_y)
    control_x, control_y, heading_step, path_fit_blend = fitted_path_point(
        points, position, target_y, fallback_target_x, curve_blend
    )
    idx = int(round(position))
    idx = min(len(points) - 1, max(0, idx))

    state.control_point_index = idx
    state.control_point_position = float(position)
    state.control_point_x = float(control_x)
    state.control_point_y = float(control_y)
    state.lookahead_blend = float(far_blend)
    state.path_heading_step = float(heading_step)
    state.path_fit_blend = float(path_fit_blend)


def curve_gain_scale_from_metrics(metrics):
    curve_from_steps = clamp(
        (abs(metrics["mean_step"]) - LOOKAHEAD_CURVE_STEP_START)
        / max(0.01, LOOKAHEAD_CURVE_STEP_FULL - LOOKAHEAD_CURVE_STEP_START),
        0.0,
        1.0,
    )
    curve_from_shape = 1.0 - clamp(metrics["straight_score"], 0.0, 1.0)
    curve_strength = max(curve_from_steps, curve_from_shape)
    return 1.0 + (CURVE_GAIN_SCALE_MAX - 1.0) * curve_strength


def steer_curve_strength(state: TrackerState):
    curve_from_steps = clamp(
        (abs(state.mean_step) - LOOKAHEAD_CURVE_STEP_START)
        / max(0.01, LOOKAHEAD_CURVE_STEP_FULL - LOOKAHEAD_CURVE_STEP_START),
        0.0,
        1.0,
    )
    curve_from_shape = 1.0 - clamp(state.straight_score, 0.0, 1.0)
    return max(curve_from_steps, curve_from_shape)


def smooth_steer_command(state: TrackerState, target_steer):
    target_steer = clamp(
        target_steer, STR_NEUTRAL - STEER_LIMIT, STR_NEUTRAL + STEER_LIMIT
    )
    dt = float(state.control_dt)
    if not math.isfinite(dt) or dt <= 0.0:
        dt = STEER_REFERENCE_DT
    dt = clamp(dt, STEER_MIN_DT, STEER_MAX_DT)
    rate_limited = rate_limit(
        target_steer, state.last_steer, STEER_RATE * dt / STEER_REFERENCE_DT
    )
    if state.mode in ("LOST", "STOP"):
        alpha = STEER_SMOOTH_ALPHA_RECOVERY
    else:
        curve_strength = steer_curve_strength(state)
        reference_alpha = (
            STEER_SMOOTH_ALPHA_STRAIGHT * (1.0 - curve_strength)
            + STEER_SMOOTH_ALPHA_CURVE * curve_strength
        )
        timed_alpha = 1.0 - (1.0 - reference_alpha) ** (dt / STEER_REFERENCE_DT)
        quiet_alpha = min(reference_alpha, timed_alpha)
        activity = clamp(state.steering_response_activity, 0.0, 1.0)
        alpha = quiet_alpha * (1.0 - activity) + timed_alpha * activity
    smoothed = state.last_steer + (rate_limited - state.last_steer) * alpha
    state.target_steer = float(target_steer)
    state.steer_smoothing_alpha = float(alpha)
    return clamp(smoothed, STR_NEUTRAL - STEER_LIMIT, STR_NEUTRAL + STEER_LIMIT)


def select_pd_gains_for_speed(state: TrackerState, encoder_state: EncoderState | None,
                             speed_boost=0.0):
    """Retain the driven high-speed gain when wheel feedback is unavailable.

    Commanded PWM is not a velocity estimate. It prevents the position gain
    from jumping back to 0.025 while a car with missing pulses is accelerating.
    Gain depends on neither lateral error nor curve shape, so crossing the
    centerline cannot itself change the selected gain.
    """
    try:
        speed_kph = float(encoder_state.speed_kph) if encoder_state is not None and encoder_state.enabled else 0.0
    except (TypeError, ValueError, OverflowError):
        speed_kph = 0.0
    if not math.isfinite(speed_kph):
        speed_kph = 0.0
    try:
        boost = float(speed_boost)
    except (TypeError, ValueError, OverflowError):
        boost = 0.0
    if not math.isfinite(boost):
        boost = 0.0
    command = clamp(SPEED_TRACK - max(0.0, boost), SPD_MIN, SPD_NEUTRAL)
    requested = round((SPD_NEUTRAL - command) / (SPD_NEUTRAL - SPD_MIN) * 100.0, 6)
    command_blend = clamp((requested - SPEED_PD_COMMAND_START_PERCENT)
                          / (SPEED_PD_COMMAND_FULL_PERCENT - SPEED_PD_COMMAND_START_PERCENT),
                          0.0, 1.0)
    wheel_blend = float(speed_kph >= SPEED_PD_SWITCH_KPH)
    target = max(command_blend, wheel_blend) if SPEED_PD_GAIN_ENABLED else 0.0
    dt = float(state.control_dt)
    if not math.isfinite(dt) or dt <= 0.0:
        dt = STEER_REFERENCE_DT
    dt = clamp(dt, STEER_MIN_DT, STEER_MAX_DT)
    if not state.gain_schedule_initialized or not SPEED_PD_GAIN_ENABLED:
        blend = target
    else:
        blend = target + (state.speed_pd_blend - target) * math.exp(-dt / SPEED_PD_BLEND_TIME_SEC)
    blend = clamp(blend, 0.0, 1.0)
    state.gain_schedule_initialized = True
    state.gain_schedule_target_blend = float(target)
    state.gain_schedule_command_percent = float(requested)
    state.gain_schedule_source = ("disabled" if not SPEED_PD_GAIN_ENABLED else
                                  "wheel_feedback" if wheel_blend >= command_blend and wheel_blend > 0 else
                                  "commanded_pwm" if command_blend > 0 else "low_command")
    state.speed_pd_active = bool(blend > 1e-6)
    state.active_center_error_gain = float(CENTER_ERROR_GAIN * (1.0 - blend)
                                           + CENTER_ERROR_GAIN_HIGH_SPEED * blend)
    state.active_center_error_d_gain = float(CENTER_ERROR_D_GAIN * (1.0 - blend)
                                             + CENTER_ERROR_D_GAIN_HIGH_SPEED * blend)
    state.speed_pd_blend = float(blend)
    state.curve_pd_blend = 0.0
    return state.active_center_error_gain, state.active_center_error_d_gain


def update_pd_derivative(state: TrackerState, used_error: float, d_gain: float):
    if state.mode not in ("TRACK", "DEGRADED") or state.good_frames <= 1:
        state.prev_used_center_error = float(used_error)
        state.d_center_error = 0.0
        state.filtered_d_center_error = 0.0
        state.pd_derivative_term = 0.0
        state.steering_derivative_unbounded = 0.0
        return 0.0

    dt = float(state.control_dt)
    if not math.isfinite(dt) or dt <= 0.0:
        dt = STEER_REFERENCE_DT
    dt = clamp(dt, STEER_MIN_DT, STEER_MAX_DT)
    d_error = (used_error - state.prev_used_center_error) * STEER_REFERENCE_DT / dt
    d_error = clamp(d_error, -CENTER_ERROR_D_LIMIT, CENTER_ERROR_D_LIMIT)
    alpha = CENTER_ERROR_D_FILTER_ALPHA ** (dt / STEER_REFERENCE_DT)
    state.filtered_d_center_error = (
        state.filtered_d_center_error * alpha
        + d_error * (1.0 - alpha)
    )
    state.prev_used_center_error = float(used_error)
    state.d_center_error = float(d_error)
    derivative_term = float(state.filtered_d_center_error * d_gain)
    state.steering_derivative_unbounded = derivative_term
    if state.hf_suppression > 0.0:
        # The D term may lead a filtered position up to the currently observed
        # line, but must not create a turn beyond that observed position. This
        # avoids extra overshoot when the adaptive filter responds faster.
        observed_error = math.copysign(
            max(0.0, abs(state.raw_center_error) - STRAIGHT_DEADBAND_PX),
            state.raw_center_error,
        )
        headroom = (observed_error - used_error) * state.active_center_error_gain
        derivative_term = clamp(
            derivative_term, min(0.0, headroom), max(0.0, headroom)
        )
    state.pd_derivative_term = derivative_term
    return state.pd_derivative_term


def steer_from_center_error(
    error,
    center_error_gain,
    gain_scale=1.0,
    heading_step=0.0,
    derivative_term=0.0,
):
    excess = max(0.0, abs(error) - CENTER_ERROR_BOOST_THRESHOLD)
    boost = np.sign(error) * excess * CENTER_ERROR_BOOST_GAIN
    heading_boost = heading_step * PATH_HEADING_GAIN
    return STR_NEUTRAL + (
        error * center_error_gain + derivative_term + boost + heading_boost
    ) * gain_scale


def inverse_lerp_score(value, good, bad):
    if bad <= good:
        return 0.0
    return clamp((bad - value) / (bad - good), 0.0, 1.0)


def straightness_metrics(points, confidence):
    if len(points) < 4:
        return {
            "straight_score": 0.0,
            "fit_rms": 0.0,
            "step_std": 0.0,
            "mean_step": 0.0,
        }

    pts = np.array(points, dtype=np.float32)
    xs = pts[:, 0]
    ys = pts[:, 1]
    steps = np.diff(xs)
    mean_step = float(np.mean(steps)) if len(steps) else 0.0
    step_std = float(np.std(steps)) if len(steps) else 0.0
    try:
        fit = np.polyfit(ys, xs, 1)
        fit_x = fit[0] * ys + fit[1]
        fit_rms = float(np.sqrt(np.mean((xs - fit_x) ** 2)))
    except Exception:
        fit_rms = 99.0

    residual_score = inverse_lerp_score(
        fit_rms, STRAIGHT_FIT_RMS_GOOD, STRAIGHT_FIT_RMS_BAD
    )
    step_std_score = inverse_lerp_score(
        step_std, STRAIGHT_STEP_STD_GOOD, STRAIGHT_STEP_STD_BAD
    )
    mean_step_score = inverse_lerp_score(
        abs(mean_step), STRAIGHT_MEAN_STEP_GOOD, STRAIGHT_MEAN_STEP_BAD
    )
    confidence_score = clamp((confidence - 0.58) / 0.30, 0.0, 1.0)
    straight_score = (
        residual_score * 0.34
        + step_std_score * 0.26
        + mean_step_score * 0.25
        + confidence_score * 0.15
    )
    return {
        "straight_score": float(clamp(straight_score, 0.0, 1.0)),
        "fit_rms": fit_rms,
        "step_std": step_std,
        "mean_step": mean_step,
    }


def filtered_center_error(state: TrackerState, det: Detection, raw_error: float, metrics=None):
    if metrics is None:
        metrics = straightness_metrics(det.points, det.confidence)
    straight_score = metrics["straight_score"] if state.mode == "TRACK" else 0.0
    dt = float(state.control_dt)
    if not math.isfinite(dt) or dt <= 0.0:
        dt = STEER_REFERENCE_DT
    dt = clamp(dt, STEER_MIN_DT, STEER_MAX_DT)
    if state.frame_index <= 1 or state.good_frames <= 1:
        # A newly verified line replaces the filter retained through LOST.
        state.filtered_center_error = raw_error
        state.steering_response_activity = 1.0 if state.frame_index > 1 else 0.0

    reference_alpha = (
        HF_FILTER_ALPHA_STRAIGHT * straight_score
        + HF_FILTER_ALPHA_CURVE * (1.0 - straight_score)
    )
    innovation = abs(raw_error - state.filtered_center_error)
    motion = clamp(
        (innovation - HF_RESPONSE_START_PX)
        / (HF_RESPONSE_FULL_PX - HF_RESPONSE_START_PX),
        0.0,
        1.0,
    )
    activity = max(
        motion, state.steering_response_activity * math.exp(-dt / HF_RESPONSE_RELEASE_SEC)
    )
    # Preserve the former rejection of +/-2 px jitter. A coherent displacement
    # speeds the filter up, then releases smoothly after the correction.
    quiet_alpha = max(reference_alpha, reference_alpha ** (dt / STEER_REFERENCE_DT))
    fast_alpha = min(quiet_alpha, math.exp(-dt / HF_RESPONSE_TIME_SEC))
    alpha = quiet_alpha * (1.0 - activity) + fast_alpha * activity
    state.steering_response_activity = float(activity)
    state.steering_filter_alpha = float(alpha)
    state.filtered_center_error = (
        state.filtered_center_error * alpha + raw_error * (1.0 - alpha)
    )
    high_freq = raw_error - state.filtered_center_error
    suppression = (
        HF_SUPPRESSION_MAX * straight_score
        if HF_SUPPRESSION_ENABLED and straight_score >= STRAIGHT_SCORE_ENABLE
        else 0.0
    )
    used_error = raw_error - suppression * high_freq
    if suppression > 0.0:
        # Keep the deadband continuous so tiny boundary noise cannot kick the D term.
        used_error = float(np.sign(used_error)) * max(
            0.0, abs(used_error) - STRAIGHT_DEADBAND_PX
        )

    state.raw_center_error = float(raw_error)
    state.used_center_error = float(used_error)
    state.high_freq_error = float(high_freq)
    state.straight_score = float(straight_score)
    state.hf_suppression = float(suppression)
    state.fit_rms = float(metrics["fit_rms"])
    state.step_std = float(metrics["step_std"])
    state.mean_step = float(metrics["mean_step"])
    return used_error


def center_error_steer(
    state: TrackerState,
    det: Detection,
    points,
    fallback_target_x,
    speed_boost,
    encoder_state=None,
    boost_mode_enabled=False,
    gain_scale=1.0,
):
    metrics = straightness_metrics(points, det.confidence)
    lookahead_boost = speed_boost
    if boost_mode_enabled and state.mode == "TRACK":
        lookahead_boost = max(lookahead_boost, BOOST_MODE_LOOKAHEAD_BOOST)
    choose_control_point(
        state,
        points,
        metrics,
        lookahead_boost,
        fallback_target_x,
        boost_mode_enabled and state.mode == "TRACK",
    )
    raw_error = center_error_from_control_point(state)
    used_error = filtered_center_error(state, det, raw_error, metrics)
    p_gain, d_gain = select_pd_gains_for_speed(state, encoder_state, speed_boost)
    derivative_term = update_pd_derivative(state, used_error, d_gain)
    gain_scale *= curve_gain_scale_from_metrics(metrics)
    baseline_target = steer_from_center_error(
        used_error, p_gain, gain_scale, state.path_heading_step, derivative_term
    )
    state.baseline_target_steer = float(baseline_target)
    state.steering_preview = state.preview_controller.update(
        points, det.confidence, state.mode, state.gain_schedule_command_percent,
        state.control_dt, base_control_y=state.control_point_y,
        base_control_x=state.control_point_x, base_heading_step=state.path_heading_step,
        center_gain=p_gain, heading_gain=PATH_HEADING_GAIN, gain_scale=gain_scale,
        path_fit_blend=state.path_fit_blend, far_observation=det.far_preview,
    )
    state.return_damping = state.return_controller.update(
        raw_error,
        used_error * p_gain * gain_scale,
        state.path_heading_step * PATH_HEADING_GAIN * gain_scale,
        derivative_term * gain_scale,
        state.steering_derivative_unbounded * gain_scale,
        det.confidence, state.mode, state.control_dt,
        points=points, far_observation=det.far_preview,
        control_y=state.control_point_y,
        preview_extra=state.steering_preview["extra_duty"],
        enabled=state.steering_autonomy_active,
        frame_index=state.frame_index,
    )
    # The separate pace reference keeps its original position/heading/D target.
    # Only a verified return phase can release part of the existing turn early.
    return (baseline_target + state.steering_preview["extra_duty"]
            + state.return_damping["delta_duty"])


def update_pace_steering_reference(state, target):
    """Keep the prior steering signal as the input to existing speed policies.

    Preview is a steering advance, not a new reason to reduce throttle. The
    existing smoother/LOST policy is evaluated on the unchanged base target,
    with its own previous output, solely for the old speed-margin checks.
    """
    reference = copy.copy(state)
    reference.last_steer = state.pace_reference_steer
    reference.last_reliable_steer = state.pace_reference_reliable_steer
    if state.mode == "TRACK":
        steering = smooth_steer_command(reference, target)
        state.pace_reference_reliable_steer = float(steering)
    else:
        steering = unreliable_steer_command(reference)
    state.pace_reference_steer = float(steering)
    state.pace_reference_ready = True


def reset_preview_reference(state):
    state.preview_controller.reset()
    state.steering_preview = {}
    state.return_controller.reset()
    state.return_damping = {}
    state.pace_reference_steer = state.last_steer
    state.pace_reference_reliable_steer = state.last_reliable_steer
    state.pace_reference_ready = False


def unreliable_steer_command(state: TrackerState):
    """Briefly retain a verified line command, then return gently to neutral."""
    state.unreliable_steer_frames += 1
    target_steer = (
        state.last_reliable_steer
        if state.unreliable_steer_frames <= UNRELIABLE_STEER_HOLD_FRAMES
        else STR_NEUTRAL
    )
    target_steer = clamp(
        target_steer, STR_NEUTRAL - STEER_LIMIT, STR_NEUTRAL + STEER_LIMIT
    )
    # This is already limited at the actual-output level. Applying the normal
    # recovery smoother again would lengthen the return to neutral.
    steer_cmd = rate_limit(target_steer, state.last_steer, UNRELIABLE_STEER_RATE)
    state.target_steer = float(target_steer)
    state.steer_smoothing_alpha = 1.0
    return clamp(steer_cmd, STR_NEUTRAL - STEER_LIMIT, STR_NEUTRAL + STEER_LIMIT)


def compute_control(
    state: TrackerState,
    det: Detection,
    speed_boost,
    boost_mode_enabled=False,
    encoder_state=None,
):
    if state.mode == "TRACK":
        steer_cmd = center_error_steer(
            state,
            det,
            det.points,
            det.target_x,
            speed_boost,
            encoder_state,
            boost_mode_enabled,
        )
        update_pace_steering_reference(state, state.baseline_target_steer)
        steer_cmd = smooth_steer_command(state, steer_cmd)
        state.last_reliable_steer = steer_cmd
        state.unreliable_steer_frames = 0
        speed_cmd = SPEED_TRACK
    else:
        # Weak points, an edge blob, or a stale side cannot create a new turn.
        state.preview_controller.reset()
        state.steering_preview = {"active": False, "reason": "unreliable_line", "extra_duty": 0.0}
        state.return_controller.reset()
        state.return_damping = {"active": False, "reason": "unreliable_line", "delta_duty": 0.0}
        update_pace_steering_reference(state, STR_NEUTRAL)
        steer_cmd = unreliable_steer_command(state)
        if state.mode == "DEGRADED":
            speed_cmd = SPEED_DEGRADED
        elif state.mode == "LOST":
            speed_cmd = SPEED_LOST
        else:
            speed_cmd = SPEED_STOP

    state.last_steer = steer_cmd
    return steer_cmd, speed_cmd


def is_high_speed(speed_boost, encoder_state: EncoderState | None):
    encoder_fast = (
        encoder_state is not None
        and encoder_state.enabled
        and encoder_state.speed_kph >= HIGH_SPEED_ENCODER_KPH_THRESHOLD
    )
    boost_fast = speed_boost >= HIGH_SPEED_BOOST_THRESHOLD
    return encoder_fast or boost_fast


def apply_high_speed_curve_speed_policy(
    state: TrackerState,
    speed_cmd,
    steer_cmd,
    speed_boost,
    encoder_state: EncoderState | None,
):
    if state.mode != "TRACK":
        return speed_cmd, False
    if not is_high_speed(speed_boost, encoder_state):
        return speed_cmd, False
    reference_steer = state.pace_reference_steer if state.pace_reference_ready else steer_cmd
    if abs(reference_steer - STR_NEUTRAL) <= HIGH_SPEED_CURVE_STEER_THRESHOLD:
        return speed_cmd, False
    return max(speed_cmd, HIGH_SPEED_CURVE_SPEED), True


def apply_speed_boost(speed_cmd, speed_boost):
    if speed_cmd >= SPD_NEUTRAL:
        if speed_boost <= 0:
            return SPD_NEUTRAL
        return clamp(SPD_NEUTRAL - speed_boost, SPD_MIN, SPD_NEUTRAL)
    return clamp(speed_cmd - speed_boost, SPD_MIN, SPD_NEUTRAL)


def apply_straight_speed_mode(
    state: TrackerState,
    det: Detection,
    output_speed,
    enabled,
    reverse_mode,
    manual_mode,
    boost_mode_enabled,
):
    now = time.time()
    can_use = (
        STRAIGHT_SPEED_MODE_ENABLED
        and boost_mode_enabled
        and enabled
        and not reverse_mode
        and not manual_mode
        and state.mode == "TRACK"
        and det.kind == "line"
        and det.confidence >= STRAIGHT_SPEED_CONFIDENCE_MIN
    )
    straight_now = can_use and state.straight_score >= STRAIGHT_SPEED_SCORE_THRESHOLD
    if straight_now:
        if state.straight_speed_start_time <= 0.0:
            state.straight_speed_start_time = now
        state.straight_speed_duration = now - state.straight_speed_start_time
    else:
        state.straight_speed_start_time = 0.0
        state.straight_speed_duration = 0.0

    state.straight_speed_active = (
        straight_now and state.straight_speed_duration >= STRAIGHT_SPEED_HOLD_SEC
    )
    state.straight_speed_curve_cut = (
        can_use
        and not state.straight_speed_active
        and state.straight_score <= STRAIGHT_SPEED_CURVE_SCORE_MAX
    )

    if state.straight_speed_curve_cut:
        output_speed = max(output_speed, STRAIGHT_SPEED_CURVE_SPEED)
    elif state.straight_speed_active:
        output_speed = clamp(
            output_speed - STRAIGHT_SPEED_EXTRA_BOOST, SPD_MIN, SPD_NEUTRAL
        )
    return output_speed


def pop_control_commands():
    with control_lock:
        commands = list(pending_control_commands)
        pending_control_commands.clear()
    return commands


def limit_auto_corner_output(pace, state, det, steering, speed, control, requested_percent,
                             motor_test_lock=False):
    """Apply the corner limit after boost, launch floor and mode arbitration."""
    active = (control.get("mode") == "AUTO" and control.get("gear") == "D"
              and not control.get("emergency", False) and not motor_test_lock)
    if not active:
        pace.reset()
        return speed, {"active": False, "phase": "INACTIVE", "reason": "mode_not_auto_drive",
                       "cap_percent": 100.0, "requested_percent": requested_percent}
    reference_steer = state.pace_reference_steer if state.pace_reference_ready else steering
    profile = pace.update(det.points, det.confidence, state.mode, state.control_dt,
                          requested_percent, steering_duty=reference_steer, neutral=STR_NEUTRAL,
                          far_observation=det.far_preview)
    limited = apply_forward_cap(speed, profile["cap_percent"], SPD_NEUTRAL, SPD_MIN)
    profile.update(active=True, command_before_cap=float(speed), command_after_cap=float(limited),
                   applied_percent=round(clamp((SPD_NEUTRAL-limited)/(SPD_NEUTRAL-SPD_MIN)*100., 0., 100.), 3),
                   limited=bool(limited > speed + 1e-8),
                   output_is_pwm_cap_not_measured_speed=True)
    return limited, profile


def signed_speed_for_odometry(
    output_speed,
    encoder_state,
    enabled,
    reverse_mode,
    manual_mode,
    manual_drive,
):
    if encoder_state is None:
        return 0.0
    speed = float(encoder_state.speed_mps)
    if manual_mode:
        if manual_drive == "forward":
            return speed
        if manual_drive == "reverse":
            return -speed
        return 0.0
    if reverse_mode:
        return -speed
    if enabled and output_speed < SPD_NEUTRAL:
        return speed
    return 0.0


def make_odometry_panel(odom_state):
    panel = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)
    if odom_state is None:
        return panel

    trail = list(odom_state.trail)
    if not trail:
        trail = [(odom_state.x_m, odom_state.y_m)]
    draw_step = max(1, int(np.ceil(len(trail) / max(1, ODOM_DRAW_MAX_POINTS))))
    draw_trail = trail[::draw_step]
    if draw_trail[-1] != trail[-1]:
        draw_trail.append(trail[-1])
    xs = [pt[0] for pt in trail] + [odom_state.x_m]
    ys = [pt[1] for pt in trail] + [odom_state.y_m]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    view_x_m = (min_x + max_x) * 0.5
    view_y_m = (min_y + max_y) * 0.5
    span_x = max(0.2, max_x - min_x)
    span_y = max(0.2, max_y - min_y)
    usable_w = max(1, FRAME_W - ODOM_VIEW_MARGIN_PX * 2)
    usable_h = max(1, FRAME_H - ODOM_VIEW_MARGIN_PX * 2)
    scale = min(usable_w / span_x, usable_h / span_y)
    scale = clamp(scale, ODOM_VIEW_MIN_SCALE, ODOM_VIEW_MAX_SCALE)
    cx = FRAME_W // 2
    cy = FRAME_H // 2

    grid_color = (45, 45, 45)
    grid_step_m = 1.0
    grid_step_px = max(8, int(round(grid_step_m * scale)))
    offset_x = int(round((-view_x_m * scale) % grid_step_px))
    offset_y = int(round((view_y_m * scale) % grid_step_px))
    for x in range(offset_x - grid_step_px, FRAME_W, grid_step_px):
        cv2.line(panel, (x, 0), (x, FRAME_H - 1), grid_color, 1)
    for y in range(offset_y - grid_step_px, FRAME_H, grid_step_px):
        cv2.line(panel, (0, y), (FRAME_W - 1, y), grid_color, 1)

    def to_px(pt):
        x_m, y_m = pt
        return (
            int(round(cx + (x_m - view_x_m) * scale)),
            int(round(cy - (y_m - view_y_m) * scale)),
        )

    pts = [to_px(pt) for pt in draw_trail]
    if len(pts) >= 2:
        pts_arr = np.array(pts, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(panel, [pts_arr], False, (60, 220, 120), 2)

    car_x, car_y = to_px((odom_state.x_m, odom_state.y_m))
    cv2.circle(panel, (car_x, car_y), 5, (0, 0, 255), -1)
    head_x = int(round(car_x + np.cos(odom_state.yaw_rad) * 18))
    head_y = int(round(car_y - np.sin(odom_state.yaw_rad) * 18))
    cv2.line(panel, (car_x, car_y), (head_x, head_y), (0, 190, 255), 2)
    cv2.circle(panel, (cx, cy), 9, (90, 90, 90), 1)
    cv2.putText(
        panel,
        f"OVERVIEW x={odom_state.x_m:+.2f} y={odom_state.y_m:+.2f}",
        (8, 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (220, 220, 220),
        1,
    )
    cv2.putText(
        panel,
        f"yaw={np.rad2deg(odom_state.yaw_rad):+.1f} deg points={len(trail)}",
        (8, 38),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (220, 220, 220),
        1,
    )
    return panel


def draw_dashboard(frame, det: Detection, state: TrackerState, steer, speed, odom_state=None):
    bird_bgr = cv2.cvtColor(det.bird, cv2.COLOR_GRAY2BGR)
    gated_bgr = cv2.cvtColor(det.gated, cv2.COLOR_GRAY2BGR)
    mask_bgr = cv2.cvtColor(det.mask, cv2.COLOR_GRAY2BGR)

    for img in (frame, mask_bgr):
        cv2.line(img, (0, ROI_Y_TOP), (FRAME_W - 1, ROI_Y_TOP), (255, 0, 255), 1)
        cv2.line(img, (0, ROI_Y_BOTTOM), (FRAME_W - 1, ROI_Y_BOTTOM), (255, 0, 255), 1)
        for rect in (BODY_MASK_CENTER, BODY_MASK_LEFT, BODY_MASK_RIGHT):
            x0, y0, x1, y1 = rect
            cv2.rectangle(img, (x0, y0), (x1, y1), (80, 80, 255), 1)
    cv2.line(bird_bgr, (0, WINDOW_Y_TOP), (FRAME_W - 1, WINDOW_Y_TOP), (255, 0, 255), 1)
    cv2.line(bird_bgr, (0, WINDOW_Y_BOTTOM), (FRAME_W - 1, WINDOW_Y_BOTTOM), (255, 0, 255), 1)
    cv2.line(gated_bgr, (0, HIST_Y_START), (FRAME_W - 1, HIST_Y_START), (255, 0, 255), 1)
    cv2.line(gated_bgr, (0, HIST_Y_END), (FRAME_W - 1, HIST_Y_END), (255, 0, 255), 1)

    for c in det.candidates:
        x, y, w, h = c["bbox"]
        color = (0, 210, 0) if c["score"] >= 1.0 else (60, 60, 180)
        cv2.rectangle(gated_bgr, (x, y), (x + w, y + h), color, 1)
        cv2.putText(
            gated_bgr,
            f"{c['label']} w{c.get('stroke_width', 0):.0f}",
            (x, max(12, y - 3)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            color,
            1,
        )

    if len(det.points) >= 2:
        pts = np.array(det.points, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(bird_bgr, [pts], False, (0, 255, 0), 3)
        cv2.polylines(gated_bgr, [pts], False, (0, 255, 0), 2)
        pts_cam = cv2.perspectiveTransform(np.array([det.points], dtype=np.float32), M_INV)
        cv2.polylines(frame, np.int32(pts_cam), False, (0, 255, 0), 3)

        p0 = det.points[-2]
        p1 = det.points[-1]
        dy = max(1, p0[1] - p1[1])
        dx_per_y = (p1[0] - p0[0]) / dy
        extended = list(det.points)
        y = p1[1] - 20
        while y >= WINDOW_Y_TOP:
            x = int(clamp(p1[0] - dx_per_y * (p1[1] - y), 0, FRAME_W - 1))
            extended.append((x, y))
            y -= 20
        if len(extended) > len(det.points):
            ext_pts = np.array(extended, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(bird_bgr, [ext_pts], False, (0, 160, 0), 1)
            cv2.polylines(gated_bgr, [ext_pts], False, (0, 160, 0), 1)
            ext_cam = cv2.perspectiveTransform(np.array([extended], dtype=np.float32), M_INV)
            cv2.polylines(frame, np.int32(ext_cam), False, (0, 160, 0), 1)

    for pt in det.points:
        cv2.circle(bird_bgr, pt, 5, (0, 255, 0), -1)
        cv2.circle(gated_bgr, pt, 4, (0, 255, 0), -1)
    # Cyan is a measured continuation above the old bird-view crop. These
    # points are drawn back on the camera image; none are an extrapolation.
    if det.far_preview.get("valid") and det.far_preview.get("points"):
        far_cam = cv2.perspectiveTransform(
            np.array([det.far_preview["points"]], dtype=np.float32), M_INV)
        cv2.polylines(frame, np.int32(far_cam), False, (255, 255, 0), 2)
    if det.points and 0 <= state.control_point_index < len(det.points):
        control_pt = (int(round(state.control_point_x)), int(round(state.control_point_y)))
        cv2.circle(bird_bgr, control_pt, 8, (255, 0, 255), 2)
        cv2.circle(gated_bgr, control_pt, 7, (255, 0, 255), 2)
    cv2.circle(bird_bgr, (det.target_x, FRAME_H - 20), 7, (0, 255, 255), -1)
    cv2.line(bird_bgr, (CENTER_X, FRAME_H), (det.target_x, FRAME_H - 20), (0, 255, 255), 2)

    steer_px = int((steer - STR_NEUTRAL) * 70 + CENTER_X)
    cv2.rectangle(frame, (8, 214), (312, 234), (255, 255, 255), 1)
    cv2.line(frame, (CENTER_X, 210), (CENTER_X, 238), (255, 255, 255), 1)
    cv2.circle(frame, (clamp(steer_px, 0, FRAME_W - 1), 224), 6, (0, 0, 255), -1)

    text = (
        f"{state.mode} conf={det.confidence:.2f} {det.kind} "
        f"lost={state.lost_frames} steer={steer:.2f} spd={speed:.2f}"
    )
    cv2.putText(frame, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
    cv2.putText(frame, det.reason, (8, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
    hf_text = (
        f"err raw={state.raw_center_error:+.1f} filt={state.filtered_center_error:+.1f} "
        f"use={state.used_center_error:+.1f} d={state.filtered_d_center_error:+.1f} "
        f"look={state.control_point_index} "
        f"hdg={state.path_heading_step:+.1f} straight={state.straight_score:.2f}"
    )
    cv2.putText(frame, hf_text, (8, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 220, 255), 1)

    top = np.hstack((frame, mask_bgr))
    odom_bgr = make_odometry_panel(odom_state)
    bottom = np.hstack((gated_bgr, odom_bgr))
    return np.vstack((top, bottom))


def make_status_payload(
    state: TrackerState,
    det: Detection,
    plan_steer,
    plan_speed,
    output_steer,
    output_speed,
    speed_boost,
    enabled,
    reverse_mode=False,
    manual_mode=False,
    manual_drive="stop",
    manual_steer=STR_NEUTRAL,
    manual_forward_speed=MANUAL_FORWARD_SPEED,
    manual_reverse_speed=MANUAL_REVERSE_SPEED,
    boost_mode_enabled=False,
    encoder_state=None,
    odom_state=None,
    odom_signed_speed_mps=0.0,
    high_speed_active=False,
    high_speed_curve_speed_cut=False,
):
    steer_pwm = duty100(output_steer)
    speed_pwm = duty100(output_speed)
    best_candidate = None
    if det.candidates:
        best_candidate = max(det.candidates, key=lambda c: float(c.get("score", 0.0)))
    mask_ratio = float(np.count_nonzero(det.mask)) / max(1, det.mask.size)
    gated_ratio = float(np.count_nonzero(det.gated)) / max(1, det.gated.size)

    current_values = {
        "target_x": float(det.target_x),
        "raw_center_error": float(state.raw_center_error),
        "filtered_center_error": float(state.filtered_center_error),
        "used_center_error": float(state.used_center_error),
        "filtered_d_center_error": float(state.filtered_d_center_error),
        "pd_derivative_term": float(state.pd_derivative_term),
        "output_steer": float(output_steer),
        "steer_pwm": float(steer_pwm),
    }
    prev_values = state.prev_status_values

    def delta(name):
        if name not in prev_values:
            return None
        return current_values[name] - prev_values[name]

    payload = {
        "time": round(time.time(), 3),
        "frame_index": int(state.frame_index),
        "mode": state.mode,
        "enabled": bool(enabled),
        "reverse_mode": bool(reverse_mode),
        "manual_mode": bool(manual_mode),
        "manual_drive": str(manual_drive),
        "manual_steer": round(float(manual_steer), 4),
        "manual_forward_speed": round(float(manual_forward_speed), 4),
        "manual_reverse_speed": round(float(manual_reverse_speed), 4),
        "run_name": current_run_name,
        "run_dir": current_run_dir,
        "mask_log_file": latest_mask_log_file,
        "mask_log_frame_index": latest_mask_log_frame_index,
        "mask_log_time": latest_mask_log_time,
        "confidence": round(float(det.confidence), 3),
        "white_extraction": det.white_stats,
        "kind": det.kind,
        "reason": det.reason,
        "side": int(det.side),
        "target_x": int(det.target_x),
        "last_target_x": int(state.last_target_x),
        "lost_frames": int(state.lost_frames),
        "good_frames": int(state.good_frames),
        "fit_rms": round(float(state.fit_rms), 3),
        "step_std": round(float(state.step_std), 3),
        "raw_center_error": round(float(state.raw_center_error), 3),
        "filtered_center_error": round(float(state.filtered_center_error), 3),
        "used_center_error": round(float(state.used_center_error), 3),
        "d_center_error": round(float(state.d_center_error), 3),
        "filtered_d_center_error": round(float(state.filtered_d_center_error), 3),
        "pd_derivative_term": round(float(state.pd_derivative_term), 4),
        "speed_pd_active": bool(state.speed_pd_active),
        "speed_pd_blend": round(float(state.speed_pd_blend), 4),
        "gain_schedule_source": state.gain_schedule_source,
        "gain_schedule_target_blend": round(float(state.gain_schedule_target_blend), 4),
        "gain_schedule_command_percent": round(float(state.gain_schedule_command_percent), 3),
        "steering_preview": copy.deepcopy(state.steering_preview),
        "return_damping": copy.deepcopy(state.return_damping),
        "far_preview": copy.deepcopy(det.far_preview),
        "baseline_target_steer": round(float(state.baseline_target_steer), 4),
        "pace_reference_steer": round(float(state.pace_reference_steer), 4),
        "curve_pd_blend": round(float(state.curve_pd_blend), 4),
        "speed_pd_switch_kph": round(float(SPEED_PD_SWITCH_KPH), 3),
        "center_error_gain_active": round(float(state.active_center_error_gain), 4),
        "center_error_d_gain_active": round(float(state.active_center_error_d_gain), 4),
        "center_error_gain_high_speed": round(float(CENTER_ERROR_GAIN_HIGH_SPEED), 4),
        "center_error_d_gain_high_speed": round(float(CENTER_ERROR_D_GAIN_HIGH_SPEED), 4),
        "high_freq_error": round(float(state.high_freq_error), 3),
        "d_target_x": None if delta("target_x") is None else round(float(delta("target_x")), 3),
        "d_raw_center_error": None if delta("raw_center_error") is None else round(float(delta("raw_center_error")), 3),
        "d_filtered_center_error": None if delta("filtered_center_error") is None else round(float(delta("filtered_center_error")), 3),
        "d_used_center_error": None if delta("used_center_error") is None else round(float(delta("used_center_error")), 3),
        "d_filtered_d_center_error": None if delta("filtered_d_center_error") is None else round(float(delta("filtered_d_center_error")), 3),
        "d_pd_derivative_term": None if delta("pd_derivative_term") is None else round(float(delta("pd_derivative_term")), 4),
        "target_steer": round(float(state.target_steer), 4),
        "steer_smoothing_alpha": round(float(state.steer_smoothing_alpha), 3),
        "steering_response_activity": round(float(state.steering_response_activity), 3),
        "steering_filter_alpha": round(float(state.steering_filter_alpha), 4),
        "steering_derivative_unbounded": round(float(state.steering_derivative_unbounded), 4),
        "plan_steer": round(float(plan_steer), 4),
        "plan_speed": round(float(plan_speed), 4),
        "output_steer": round(float(output_steer), 4),
        "steer_pwm": int(steer_pwm),
        "d_output_steer": None if delta("output_steer") is None else round(float(delta("output_steer")), 4),
        "d_steer_pwm": None if delta("steer_pwm") is None else int(round(delta("steer_pwm"))),
        "output_speed": round(float(output_speed), 4),
        "speed_boost": round(float(speed_boost), 4),
        "boost_mode_enabled": bool(boost_mode_enabled),
        "force_farthest_lookahead": bool(FORCE_FARTHEST_LOOKAHEAD),
        "boost_mode_lookahead_boost": round(float(BOOST_MODE_LOOKAHEAD_BOOST), 3),
        "boost_mode_force_farthest_lookahead": bool(BOOST_MODE_FORCE_FARTHEST_LOOKAHEAD),
        "straight_speed_active": bool(state.straight_speed_active),
        "straight_speed_duration": round(float(state.straight_speed_duration), 3),
        "straight_speed_curve_cut": bool(state.straight_speed_curve_cut),
        "straight_speed_score_threshold": round(float(STRAIGHT_SPEED_SCORE_THRESHOLD), 3),
        "straight_speed_hold_sec": round(float(STRAIGHT_SPEED_HOLD_SEC), 3),
        "straight_speed_extra_boost": round(float(STRAIGHT_SPEED_EXTRA_BOOST), 3),
        "straight_speed_curve_score_max": round(float(STRAIGHT_SPEED_CURVE_SCORE_MAX), 3),
        "reverse_speed": round(float(REVERSE_SPEED), 4),
        "speed_pwm": int(speed_pwm),
        "encoder_gpio": int(GPPIN_ENCODER),
        "encoder_enabled": bool(encoder_state.enabled) if encoder_state is not None else False,
        "encoder_pulse_count": int(encoder_state.pulse_count) if encoder_state is not None else 0,
        "encoder_pulse_rate_hz": (
            round(float(encoder_state.pulse_rate_hz), 3)
            if encoder_state is not None
            else 0.0
        ),
        "encoder_speed_mps": (
            round(float(encoder_state.speed_mps), 4)
            if encoder_state is not None
            else 0.0
        ),
        "encoder_speed_kph": (
            round(float(encoder_state.speed_kph), 4)
            if encoder_state is not None
            else 0.0
        ),
        "wheel_diameter_m": round(float(WHEEL_DIAMETER_M), 4),
        "encoder_pulses_per_rev": round(float(ENCODER_PULSES_PER_REV), 3),
        "encoder_speed_scale": round(float(ENCODER_SPEED_SCALE), 4),
        "odom_x_m": round(float(odom_state.x_m), 4) if odom_state is not None else 0.0,
        "odom_y_m": round(float(odom_state.y_m), 4) if odom_state is not None else 0.0,
        "odom_yaw_rad": round(float(odom_state.yaw_rad), 6) if odom_state is not None else 0.0,
        "odom_yaw_deg": (
            round(float(np.rad2deg(odom_state.yaw_rad)), 3)
            if odom_state is not None
            else 0.0
        ),
        "odom_signed_speed_mps": round(float(odom_signed_speed_mps), 4),
        "odom_wheelbase_m": round(float(ODOM_WHEELBASE_M), 4),
        "odom_max_steer_angle_deg": round(float(ODOM_MAX_STEER_ANGLE_DEG), 3),
        "odom_steer_gain": round(float(ODOM_STEER_GAIN), 3),
        "odom_steer_deadband": round(float(ODOM_STEER_DEADBAND), 3),
        "odom_steer_sign": round(float(ODOM_STEER_SIGN), 3),
        "odom_trail": (
            [[round(float(x), 3), round(float(y), 3)] for x, y in odom_state.trail]
            if odom_state is not None
            else []
        ),
        "high_speed_active": bool(high_speed_active),
        "high_speed_curve_speed_cut": bool(high_speed_curve_speed_cut),
        "high_speed_boost_threshold": round(float(HIGH_SPEED_BOOST_THRESHOLD), 3),
        "high_speed_encoder_kph_threshold": round(float(HIGH_SPEED_ENCODER_KPH_THRESHOLD), 3),
        "high_speed_encoder_pps_threshold": round(float(HIGH_SPEED_ENCODER_PPS_THRESHOLD), 3),
        "hf_suppression": round(float(state.hf_suppression), 3),
        "hf_suppression_enabled": bool(HF_SUPPRESSION_ENABLED),
        "straight_score": round(float(state.straight_score), 3),
        "mean_step": round(float(state.mean_step), 3),
        "mask_ratio": round(mask_ratio, 5),
        "gated_ratio": round(gated_ratio, 5),
        "line_width": round(float(det.line_width), 3),
        "expected_line_width": round(float(state.expected_line_width), 3),
        "control_point_index": int(state.control_point_index),
        "control_point_position": round(float(state.control_point_position), 3),
        "control_point_x": round(float(state.control_point_x), 3),
        "control_point_y": round(float(state.control_point_y), 3),
        "lookahead_blend": round(float(state.lookahead_blend), 3),
        "path_heading_step": round(float(state.path_heading_step), 3),
        "path_fit_blend": round(float(state.path_fit_blend), 3),
        "points_count": int(len(det.points)),
        "candidates_count": int(len(det.candidates)),
        "candidates": det.candidates,
        "best_candidate_score": (
            None if best_candidate is None else round(float(best_candidate.get("score", 0.0)), 3)
        ),
        "best_candidate_area": (
            None if best_candidate is None else int(best_candidate.get("area", 0))
        ),
        "best_candidate_centroid_x": (
            None
            if best_candidate is None
            else round(float(best_candidate.get("centroid", (0.0, 0.0))[0]), 3)
        ),
    }
    state.prev_status_values = current_values
    return payload


def preview_refresh_due(now, last_refresh, force=False):
    return force or last_refresh is None or now - last_refresh >= 1.0 / PREVIEW_HZ


def render_visual_task(task):
    """Only the visual worker draws/encodes/writes these optional artifacts."""
    global latest_jpeg, latest_mask_log_frame_index, latest_mask_log_time
    frame, det, tracker, odom = task["frame"].copy(), task["det"], task["state"], task["odom"]
    if task["preview"]:
        dashboard = draw_dashboard(frame, det, tracker, task["steer"], task["speed"], odom)
        ok, jpeg = cv2.imencode(".jpg", dashboard, [cv2.IMWRITE_JPEG_QUALITY, 82])
        if ok:
            latest_jpeg = jpeg.tobytes()
        for path in task["image_paths"]:
            if not cv2.imwrite(path, dashboard):
                raise OSError("preview image write failed")
        if task["image_paths"]:
            manage_storage(task["run_dir"])
    if task["mask"]:
        task["writer"].write(make_mask_log_frame(det.bird, det.gated))
        latest_mask_log_frame_index = int(tracker.frame_index)
        latest_mask_log_time = task["frame_time"]


def capture_metadata(payload, det, camera_metadata):
    """Keep frame-specific evidence, excluding the large odometry trail."""
    fields = (
        "time", "frame_index", "mode", "confidence", "kind", "reason", "side",
        "target_x", "lost_frames", "good_frames", "fit_rms", "step_std",
        "raw_center_error", "filtered_center_error", "used_center_error",
        "pd_derivative_term", "target_steer", "plan_steer", "output_steer",
        "plan_speed", "output_speed", "control_point_x", "control_point_y",
        "path_heading_step", "straight_score", "hf_suppression",
        "steering_response_activity", "steering_filter_alpha", "steering_derivative_unbounded",
        "line_width", "expected_line_width", "mask_ratio", "gated_ratio",
        "encoder_pulse_count", "encoder_pulse_rate_hz", "encoder_speed_kph",
        "auto_output_setting_percent", "actuator_output", "vehicle_control", "fast_control_output",
        "loop_timing", "steering_response", "run_name",
        "white_extraction",
        "speed_pd_active", "speed_pd_blend", "curve_pd_blend",
        "gain_schedule_source", "gain_schedule_target_blend", "gain_schedule_command_percent",
        "steering_preview", "return_damping", "far_preview", "baseline_target_steer", "pace_reference_steer",
        "center_error_gain_active", "center_error_d_gain_active", "corner_pace",
    )
    result = {key: payload.get(key) for key in fields}
    result["points"] = [[int(x), int(y)] for x, y in det.points]
    result["candidates"] = det.candidates
    result["camera"] = {
        key: camera_metadata.get(key) for key in
        ("SensorTimestamp", "ExposureTime", "FrameDuration", "AnalogueGain")
    }
    result["mask_layout"] = "masks: camera_white_mask; extra_masks/bird: bird_mask; extra_masks/gated: gated_mask"
    result["perspective_matrix"] = M_TRANS.tolist()
    result["build"] = recording_build
    return result


def main():
    global latest_jpeg, latest_status, current_run_dir, current_run_name
    global latest_mask_log_file, latest_mask_log_frame_index, latest_mask_log_time
    global _legacy_pi_output, _legacy_pi_connection, dualsense_reader
    global drive_capture, visual_worker, recording_build, fast_control_output

    import pigpio

    try:
        from picamera2 import Picamera2
    except ImportError as e:
        print(f"Error: picamera2が見つかりません -> {e}")
        raise SystemExit(1)

    pi = pigpio.pi()
    if not pi.connected:
        print("Error: pigpioデーモンに接続できません。sudo pigpiod を実行してください。")
        return
    _legacy_pi_connection = pi
    _legacy_pi_output = LegacyPiOutput(pi)
    encoder_state = EncoderState()
    pi.set_mode(GPPIN_ENCODER, pigpio.INPUT)
    pi.set_pull_up_down(GPPIN_ENCODER, pigpio.PUD_UP)
    encoder_callback = pi.callback(
        GPPIN_ENCODER, pigpio.RISING_EDGE, encoder_state.pulse_callback
    )

    threading.Thread(
        target=lambda: ThreadedHTTPServer(("0.0.0.0", 8000), MJPEGHandler).serve_forever(),
        daemon=True,
    ).start()

    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"format": "XRGB8888", "size": (FRAME_W, FRAME_H)},
        queue=False,
    )
    picam2.configure(config)
    picam2.start()

    keyboard_enabled = sys.stdin.isatty()
    old_term_settings = termios.tcgetattr(sys.stdin) if keyboard_enabled else None
    if keyboard_enabled:
        tty.setcbreak(sys.stdin.fileno())

    run_dir, run_name = create_run_dir()
    write_run_meta(run_dir, run_name)
    write_log_manifest(run_dir, run_name)
    write_log_viewer(run_dir, run_name)
    current_run_dir = run_dir
    current_run_name = run_name
    mask_video_writer, mask_video_path = create_mask_video_writer(run_dir)
    latest_mask_log_file = MASK_LOG_VIDEO
    with open(__file__, "rb") as source:
        recording_build = {"source_sha256": hashlib.sha256(source.read()).hexdigest(),
                           "steering_response": "adaptive_dt_v1",
                           "frame_size": [FRAME_W, FRAME_H],
                           "p_gain": CENTER_ERROR_GAIN,
                           "p_gain_high_speed": CENTER_ERROR_GAIN_HIGH_SPEED,
                           "d_gain": CENTER_ERROR_D_GAIN,
                           "force_farthest_lookahead": FORCE_FARTHEST_LOOKAHEAD}
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "adaptive_white.py"), "rb") as source:
        recording_build["white_extraction_sha256"] = hashlib.sha256(source.read()).hexdigest()
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "curve_pace.py"), "rb") as source:
        recording_build["curve_pace_sha256"] = hashlib.sha256(source.read()).hexdigest()
    for module in ("drive_capture.py", "drive_capture_finalize.py", "fast_control_output.py", "dualsense_control.py", "legacy_pi_output.py",
                   "far_preview.py", "steering_preview.py", "return_damping.py"):
        with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), module), "rb") as source:
            recording_build[module.removesuffix(".py") + "_sha256"] = hashlib.sha256(source.read()).hexdigest()
    recording_build["recorder_finalization"] = "isolated_process_v1"
    recording_build["manual_output_scheduling"] = "complete_report_event_v1"
    recording_build["steering_gain_schedule"] = "command_fallback_v1"
    recording_build["steering_preview"] = "connected_far_road_sharp_entry_v2"
    recording_build["return_damping"] = "coherent_convergence_bounded_v1"
    recording_build["auto_output_pacing"] = "verified_far_straight_v1"
    drive_capture = DriveCapture(os.path.join(OUTPUT_DIR, "drive_capture"))
    visual_worker = LatestWorker(render_visual_task, name="visual-preview")

    dualsense_reader = DualSenseReader(vehicle_control)
    dualsense_reader.start()
    # The camera only publishes AUTO plans. This is the sole actuation owner;
    # manual, gear, braking and stop decisions run independently at 100 Hz.
    fast_control_output = FastControlOutput(vehicle_control, _legacy_pi_output,
                                           motor_test_lock=MOTOR_TEST_LOCK)
    fast_control_output.start()
    state = TrackerState()
    curve_pace = CurvePace()
    odom_state = DeadReckoningState()
    odom_state.reset()
    enabled = False
    reverse_mode = False
    manual_mode = False
    manual_drive = "stop"
    manual_steer = STR_NEUTRAL
    manual_forward_speed = MANUAL_FORWARD_SPEED
    manual_reverse_speed = MANUAL_REVERSE_SPEED
    boost_mode_enabled = False
    prev_dashboard = None
    last_preview_time = None
    last_error_save_frame = -ERROR_SAVE_INTERVAL_FRAMES
    manual_capture_count = 0
    speed_boost = 0.0
    last_speed_request_id = None
    applied_speed_request_ids = deque(maxlen=128)
    last_mask_log_time = 0.0
    mask_log_interval = 1.0 / max(0.1, MASK_LOG_HZ)
    last_control_at = None

    print("--- ROBUST LANE FAILSAFE START ---")
    print(f"Run output: {run_dir}")
    print("Access http://[RaspberryPi IP]:8000")
    print("Keys: [w] drive/faster  [s] slower  [space/n] neutral  [c] capture  [q] quit")
    print("Web: Drive ON / Neutral / Speed +/- are available from the browser")

    try:
        while True:
            fast_control_output.check_error()
            loop_started = time.monotonic()
            request = picam2.capture_request()
            try:
                camera_metadata = request.get_metadata()
                frame_raw = request.make_array("main")
                frame = cv2.rotate(cv2.cvtColor(frame_raw, cv2.COLOR_BGRA2BGR), cv2.ROTATE_180)
            finally:
                request.release()
            captured_at = time.monotonic()
            state.control_dt = (1.0 / 30.0 if last_control_at is None
                                else captured_at - last_control_at)
            last_control_at = captured_at
            det = detect_line(frame, state)
            update_state(state, det)
            encoder_state.update_idle()
            detected_at = time.monotonic()

            key = (
                sys.stdin.read(1)
                if keyboard_enabled and select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])
                else None
            )
            capture_requested = False
            if key == "q":
                break
            commands = []
            if key == " " or key == "n":
                commands.append("neutral")
            elif key == "c":
                capture_requested = True
            elif key == "w":
                commands.append("faster")
            elif key == "s":
                commands.append("slower")
            commands.extend(pop_control_commands())
            for command in commands:
                if isinstance(command, tuple) and command[0] == "speed_setting":
                    if command[2] not in applied_speed_request_ids:
                        speed_boost = clamp(speed_boost + command[1] * SPEED_STEP, 0.0, SPEED_BOOST_MAX)
                        applied_speed_request_ids.append(command[2])
                    last_speed_request_id = command[2]
                    continue
                if command == "neutral":
                    vehicle_control.stop("web_stop")
                    enabled = False
                    reverse_mode = False
                    manual_mode = False
                    manual_drive = "stop"
                    manual_steer = STR_NEUTRAL
                    speed_boost = 0.0
                    state.last_steer = STR_NEUTRAL
                    state.last_reliable_steer = STR_NEUTRAL
                    state.unreliable_steer_frames = 0
                    reset_preview_reference(state)
                    print("drive_enabled=False")
                elif command == "drive_on":
                    vehicle_control.request_mode("AUTO")
                    enabled = True
                    reverse_mode = False
                    manual_mode = False
                    manual_drive = "stop"
                    print(f"drive_enabled=True speed_boost={speed_boost:.2f}")
                elif command == "faster":
                    speed_boost = clamp(speed_boost + SPEED_STEP, 0.0, SPEED_BOOST_MAX)
                    print(f"drive_enabled=True speed_boost={speed_boost:.2f}")
                elif command == "slower":
                    speed_boost = clamp(speed_boost - SPEED_STEP, 0.0, SPEED_BOOST_MAX)
                    print(f"speed_boost={speed_boost:.2f}")
                elif command == "reverse":
                    enabled = False
                    reverse_mode = True
                    manual_mode = False
                    manual_drive = "stop"
                    speed_boost = 0.0
                    state.last_steer = STR_NEUTRAL
                    state.last_reliable_steer = STR_NEUTRAL
                    state.unreliable_steer_frames = 0
                    reset_preview_reference(state)
                    print(f"reverse_mode=True reverse_speed={REVERSE_SPEED:.2f}")
                elif command == "boost_on":
                    boost_mode_enabled = True
                    print("boost_mode=True")
                elif command == "boost_off":
                    boost_mode_enabled = False
                    state.straight_speed_start_time = 0.0
                    state.straight_speed_duration = 0.0
                    state.straight_speed_active = False
                    state.straight_speed_curve_cut = False
                    print("boost_mode=False")
                elif command == "manual_on":
                    vehicle_control.request_mode("MANUAL")
                    manual_mode = True
                    enabled = False
                    reverse_mode = False
                    speed_boost = 0.0
                    manual_drive = "stop"
                    manual_steer = STR_NEUTRAL
                    state.last_steer = STR_NEUTRAL
                    state.last_reliable_steer = STR_NEUTRAL
                    state.unreliable_steer_frames = 0
                    reset_preview_reference(state)
                    print("manual_mode=True")
                elif command == "manual_off":
                    manual_mode = False
                    manual_drive = "stop"
                    manual_steer = STR_NEUTRAL
                    print("manual_mode=False")
                elif command == "manual_left":
                    manual_mode = True
                    manual_steer = MANUAL_STEER_LEFT
                    print("manual_steer=left")
                elif command == "manual_right":
                    manual_mode = True
                    manual_steer = MANUAL_STEER_RIGHT
                    print("manual_steer=right")
                elif command == "manual_center":
                    manual_mode = True
                    manual_steer = STR_NEUTRAL
                    print("manual_steer=center")
                elif command == "manual_forward":
                    manual_mode = True
                    manual_drive = "forward"
                    print("manual_drive=forward")
                elif command == "manual_reverse":
                    manual_mode = True
                    manual_drive = "reverse"
                    print("manual_drive=reverse")
                elif command == "manual_stop":
                    manual_mode = True
                    manual_drive = "stop"
                    print("manual_drive=stop")
                elif command == "manual_forward_faster":
                    manual_mode = True
                    manual_forward_speed = clamp(
                        manual_forward_speed - MANUAL_SPEED_STEP,
                        MANUAL_FORWARD_SPEED_MIN,
                        MANUAL_FORWARD_SPEED_MAX,
                    )
                    print(f"manual_forward_speed={manual_forward_speed:.2f}")
                elif command == "manual_forward_slower":
                    manual_mode = True
                    manual_forward_speed = clamp(
                        manual_forward_speed + MANUAL_SPEED_STEP,
                        MANUAL_FORWARD_SPEED_MIN,
                        MANUAL_FORWARD_SPEED_MAX,
                    )
                    print(f"manual_forward_speed={manual_forward_speed:.2f}")
                elif command == "manual_reverse_faster":
                    manual_mode = True
                    manual_reverse_speed = clamp(
                        manual_reverse_speed + MANUAL_SPEED_STEP,
                        MANUAL_REVERSE_SPEED_MIN,
                        MANUAL_REVERSE_SPEED_MAX,
                    )
                    print(f"manual_reverse_speed={manual_reverse_speed:.2f}")
                elif command == "manual_reverse_slower":
                    manual_mode = True
                    manual_reverse_speed = clamp(
                        manual_reverse_speed - MANUAL_SPEED_STEP,
                        MANUAL_REVERSE_SPEED_MIN,
                        MANUAL_REVERSE_SPEED_MAX,
                    )
                    print(f"manual_reverse_speed={manual_reverse_speed:.2f}")
                elif command == "map_reset":
                    odom_state.reset()
                    print("dead_reckoning_map_reset")

            speed_boost = vehicle_control.apply_speed_steps(speed_boost, SPEED_STEP, 0.0, SPEED_BOOST_MAX)
            steering_control = vehicle_control.snapshot()
            state.steering_autonomy_active = (
                steering_control["mode"] in ("AUTO", "STEER_AUTO")
                and not steering_control["emergency"])
            plan_steer, plan_speed = compute_control(
                state, det, speed_boost, boost_mode_enabled, encoder_state
            )
            plan_speed, high_speed_curve_speed_cut = apply_high_speed_curve_speed_policy(
                state, plan_speed, plan_steer, speed_boost, encoder_state
            )
            high_speed_active = is_high_speed(speed_boost, encoder_state)
            output_steer = plan_steer
            output_speed = apply_speed_boost(plan_speed, speed_boost)
            output_speed = apply_straight_speed_mode(
                state,
                det,
                output_speed,
                enabled,
                reverse_mode,
                manual_mode,
                boost_mode_enabled,
            )

            if state.mode == "STOP":
                # Perception STOP describes a long line loss, not an operator
                # stop. Keep the existing AUTO crawl; ControlState below still
                # gives operator STOP/N, emergency and L2 braking priority.
                output_speed = min(output_speed, SPEED_TRACK)
            autonomous_steer, autonomous_speed = output_steer, output_speed
            if not enabled:
                output_speed = SPD_NEUTRAL
            if reverse_mode:
                output_speed = REVERSE_SPEED
            if manual_mode:
                output_steer = manual_steer
                if manual_drive == "forward":
                    output_speed = manual_forward_speed
                elif manual_drive == "reverse":
                    output_speed = manual_reverse_speed
                else:
                    output_speed = SPD_NEUTRAL
            control_snapshot = vehicle_control.snapshot()
            if control_snapshot["active"]:
                output_steer, output_speed, control_snapshot = vehicle_control.preview_and_snapshot(autonomous_steer, autonomous_speed)
                enabled = control_snapshot["mode"] == "AUTO" and not control_snapshot["emergency"]
                manual_mode = control_snapshot["mode"] in ("MANUAL", "STEER_AUTO")
                reverse_mode = control_snapshot["gear"] == "R" and manual_mode
                manual_drive = "forward" if output_speed < SPD_NEUTRAL - 0.01 else "reverse" if output_speed > SPD_NEUTRAL + 0.01 else "stop"
                manual_steer = output_steer
            if MOTOR_TEST_LOCK:
                output_speed = SPD_NEUTRAL
            odom_signed_speed_mps = signed_speed_for_odometry(
                output_speed,
                encoder_state,
                enabled,
                reverse_mode,
                manual_mode,
                manual_drive,
            )
            requested_percent = round(clamp(
                (SPD_NEUTRAL - apply_speed_boost(SPEED_TRACK, speed_boost))
                / (SPD_NEUTRAL - SPD_MIN) * 100., 0., 100.), 1)
            output_speed, pace_snapshot = limit_auto_corner_output(
                curve_pace, state, det, output_steer, output_speed,
                control_snapshot, requested_percent, MOTOR_TEST_LOCK)
            odom_state.update(output_steer, odom_signed_speed_mps)

            output_write_started = time.monotonic()
            # Keep the existing AUTO plan and final corner cap. Re-arbitrate
            # against current controls inside the one output owner, so a mode
            # change during image processing cannot replay an old AUTO write.
            forward_limit = (SPD_NEUTRAL - (SPD_NEUTRAL - SPD_MIN)
                             * pace_snapshot["cap_percent"] / 100.
                             if pace_snapshot["active"] else None)
            fast_control_output.publish_auto(autonomous_steer, autonomous_speed,
                                             forward_limit=forward_limit,
                                             mode_generation=control_snapshot["mode_generation"])
            applied_output = fast_control_output.refresh()
            output_steer, output_speed = applied_output["steering"], applied_output["throttle"]
            control_snapshot = applied_output["vehicle_control"]
            enabled = control_snapshot["mode"] == "AUTO" and not control_snapshot["emergency"]
            manual_mode = control_snapshot["mode"] in ("MANUAL", "STEER_AUTO")
            reverse_mode = control_snapshot["gear"] == "R" and manual_mode
            manual_drive = "forward" if output_speed < SPD_NEUTRAL - 0.01 else "reverse" if output_speed > SPD_NEUTRAL + 0.01 else "stop"
            manual_steer = output_steer
            output_written_at = time.monotonic()
            sensor_timestamp = camera_metadata.get("SensorTimestamp")
            sensor_age_ms = ((time.monotonic_ns() - int(sensor_timestamp)) / 1e6
                             if isinstance(sensor_timestamp, (int, float)) else None)
            if sensor_age_ms is not None and not 0 <= sensor_age_ms < 10000:
                sensor_age_ms = None
            loop_timing = {
                "capture_wait_ms": round((captured_at - loop_started) * 1000, 3),
                "vision_ms": round((detected_at - captured_at) * 1000, 3),
                "control_and_output_ms": round((output_written_at - detected_at) * 1000, 3),
                "output_write_ms": round((output_written_at - output_write_started) * 1000, 3),
                "processing_to_output_ms": round((output_written_at - captured_at) * 1000, 3),
                "camera_to_output_ms": None if sensor_age_ms is None else round(sensor_age_ms, 3),
                "frame_interval_ms": round(state.control_dt * 1000, 3),
                "camera_queue": False,
            }
            now_mono = output_written_at
            should_save_error = (
                enabled
                and state.mode in ("LOST", "STOP")
                and last_preview_time is not None
                and state.frame_index - last_error_save_frame >= ERROR_SAVE_INTERVAL_FRAMES
            )
            refresh_preview = preview_refresh_due(
                now_mono, last_preview_time, capture_requested or should_save_error
            )
            refresh_mask = now_mono - last_mask_log_time >= mask_log_interval
            image_paths = []
            if capture_requested:
                image_paths.append(os.path.join(run_dir,
                    f"robust_manual_{int(time.time())}_{manual_capture_count:03d}.jpg"))
                manual_capture_count += 1
            if should_save_error:
                image_paths.append(os.path.join(run_dir,
                    f"robust_error_{int(time.time())}_{state.frame_index:06d}.jpg"))
                last_error_save_frame = state.frame_index
            if refresh_preview or refresh_mask:
                # Scalar tracker fields are copied; the worker never reads
                # mutable control state or holds the control/status locks.
                preview_state = copy.copy(state)
                preview_odom = copy.copy(odom_state)
                preview_odom.trail = deque(odom_state.trail, maxlen=ODOM_TRAIL_MAX_POINTS)
                visual_worker.submit({
                    "frame": frame, "det": det, "state": preview_state,
                    "odom": preview_odom, "steer": output_steer, "speed": output_speed,
                    "preview": refresh_preview, "mask": refresh_mask,
                    "image_paths": image_paths, "run_dir": run_dir,
                    "writer": mask_video_writer, "frame_time": round(time.time(), 3),
                })
            if refresh_preview:
                last_preview_time = now_mono
            if refresh_mask:
                last_mask_log_time = now_mono

            with status_lock:
                latest_status = make_status_payload(
                    state,
                    det,
                    plan_steer,
                    plan_speed,
                    output_steer,
                    output_speed,
                    speed_boost,
                    enabled,
                    reverse_mode,
                    manual_mode,
                    manual_drive,
                    manual_steer,
                    manual_forward_speed,
                    manual_reverse_speed,
                    boost_mode_enabled,
                    encoder_state,
                    odom_state,
                    odom_signed_speed_mps,
                    high_speed_active,
                    high_speed_curve_speed_cut,
                )
                latest_status.update(
                    system_variant="original_20260523_direct_pi",
                    original_source_sha256="879e48c2f2f1b7c5c0890a99c7e1d206b781a621991e3875e88dd21abb423dd5",
                    actuator_output=applied_output["actuator_output"],
                    vehicle_control=control_snapshot,
                    fast_control_output=applied_output,
                    motor_test_lock=MOTOR_TEST_LOCK,
                    last_speed_request_id=last_speed_request_id,
                    auto_output_setting_percent=requested_percent,
                    encoder_sample_age_ms=round(max(0.0, time.time() - encoder_state.last_update_time) * 1000, 1) if encoder_state.last_update_time else None,
                    forward_output_percent=round(clamp(
                        (SPD_NEUTRAL - output_speed) / (SPD_NEUTRAL - SPD_MIN) * 100.0,
                        0.0, 100.0), 1),
                    forward_output_limit_percent=pace_snapshot["cap_percent"],
                    speed_step_percent=5.0,
                    loop_timing=loop_timing,
                    visual_worker=visual_worker.status(),
                    steering_response="adaptive_dt_v1",
                    corner_pace=pace_snapshot,
                )
                # This is the same sample and image used for the actual output
                # decision, not a separate HTTP poll with an uncertain match.
                record_payload = capture_metadata(latest_status, det, camera_metadata)
            drive_capture.submit(frame, det.mask, record_payload,
                                 extra_masks={"bird": det.bird, "gated": det.gated})

            print(
                f"{state.mode:8s} conf={det.confidence:.2f} kind={det.kind:9s} "
                f"pts={len(det.points)} side={det.side:+d} target={det.target_x:3d} "
                f"steer={output_steer:.2f} d={output_steer - STR_NEUTRAL:+.2f} plan_spd={plan_speed:.2f} "
                f"out_spd={output_speed:.2f} boost={speed_boost:.2f} rev={int(reverse_mode)} man={int(manual_mode)} "
                f"raw={state.raw_center_error:+.1f} use={state.used_center_error:+.1f} "
                f"de={state.filtered_d_center_error:+.1f} dt={state.pd_derivative_term:+.2f} "
                f"enc={encoder_state.pulse_rate_hz:.1f}Hz hs={int(high_speed_active)} cut={int(high_speed_curve_speed_cut)} "
                f"hdg={state.path_heading_step:+.1f} look={state.control_point_index} "
                f"st={state.straight_score:.2f} hf={state.hf_suppression:.2f} "
                f"enabled={enabled}\r",
                end="",
            )

    except KeyboardInterrupt:
        pass
    finally:
        output_close_error = None
        if fast_control_output is not None:
            try:
                fast_control_output.close()
            except Exception as error:
                # A stored writer error must not skip input/camera/recorder
                # cleanup after the worker has already neutralized and exited.
                output_close_error = error
                print(f"Driver output shutdown: {error}")
        if dualsense_reader is not None:
            dualsense_reader.close()
        try:
            if fast_control_output is None or not fast_control_output.status()["running"]:
                _legacy_pi_output.close()
        finally:
            if old_term_settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term_settings)
        encoder_callback.cancel()
        if drive_capture is not None:
            drive_capture.close()
        if visual_worker is None or visual_worker.close(timeout=5):
            mask_video_writer.release()
        archive_start = time.monotonic()
        archive_path = write_run_archive(run_dir, run_name)
        picam2.stop()
        archive_elapsed = time.monotonic() - archive_start
        print(f"\nSaved archive: {archive_path} ({archive_elapsed:.2f}s)")
        print(f"Mask video: {mask_video_path}")
        print("Shutdown complete.")
        if output_close_error is not None:
            raise output_close_error


if __name__ == "__main__":
    def stop_original(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop_original)
    try:
        main()
    finally:
        writer_joined = (fast_control_output is None
                         or not fast_control_output.status()["running"])
        try:
            if _legacy_pi_output is not None and writer_joined:
                _legacy_pi_output.close()
        finally:
            if _legacy_pi_connection is not None and writer_joined:
                _legacy_pi_connection.stop()
