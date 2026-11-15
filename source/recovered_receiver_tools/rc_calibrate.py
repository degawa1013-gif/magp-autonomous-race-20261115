#!/usr/bin/env python3
"""Measure receiver inputs and save a disabled calibration draft, without actuation.

Connects to existing local pigpiod or a Pico USB capture device. No Pi GPIO
modes, pulls, outputs, camera, or daemon lifecycle are changed. Steering positive means right in
robust_lane_failsafe.py; throttle positive means forward in rc_receiver.py.
"""

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from numbers import Real
from pathlib import Path
from statistics import median
import sys
import time
from typing import Mapping

from rc_receiver import (FRESHNESS_S, MAX_PERIOD_US, MAX_PULSE_US, MIN_PERIOD_US,
                         MIN_PULSE_US, RCConfig, ReceiverInput)


PHASES = ("neutral", "steer_left", "steer_right", "throttle_forward", "throttle_reverse")
CHANNELS = ("steering", "throttle")
MIN_SAMPLES = 20
MAX_SPREAD_US = 40
MIN_ENDPOINT_SPAN_US = 100
MIN_READY_FRACTION = 0.8
MIN_MEASURED_FRACTION = 0.7
POLL_S = 0.005
PROMPTS = {
    "neutral": "ハンドルとスロットルを中立で保持してください。",
    "steer_left": "スロットルを中立に保ち、ハンドルを左いっぱいで保持してください。",
    "steer_right": "スロットルを中立に保ち、ハンドルを右いっぱいで保持してください。",
    "throttle_forward": "ハンドルを中立に保ち、スロットルを前進いっぱいで保持してください。",
    "throttle_reverse": "ハンドルを中立に保ち、スロットルを後退いっぱいで保持してください。",
}


class CalibrationError(ValueError):
    pass


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise CalibrationError(f"{name} must be a finite number")
    return value


def summarize_samples(samples):
    """Summarize distinct qualified frames, rejecting stale or unstable input."""
    if not isinstance(samples, list) or len(samples) < MIN_SAMPLES:
        raise CalibrationError(f"Need at least {MIN_SAMPLES} distinct qualified frames")
    widths, periods, generated = [], [], []
    for sample in samples:
        if not isinstance(sample, Mapping):
            raise CalibrationError("Frame must be an object")
        width = _finite(sample.get("pulse_us"), "pulse_us")
        period = _finite(sample.get("period_us"), "period_us")
        at = _finite(sample.get("generated_at"), "generated_at")
        observed = _finite(sample.get("observed_at"), "observed_at")
        if not MIN_PULSE_US <= width <= MAX_PULSE_US:
            raise CalibrationError("Pulse width is outside the qualified receiver range")
        if not MIN_PERIOD_US <= period <= MAX_PERIOD_US:
            raise CalibrationError("Pulse period is outside the qualified receiver range")
        if not 0 <= observed - at <= FRESHNESS_S:
            raise CalibrationError("Frame is stale or its timestamp is in the future")
        if generated and at <= generated[-1]:
            raise CalibrationError("Frames must have distinct increasing generation timestamps")
        widths.append(width)
        periods.append(period)
        generated.append(at)
    spread = max(widths) - min(widths)
    if spread > MAX_SPREAD_US:
        raise CalibrationError(f"Unstable hold: pulse spread {spread:g} us exceeds {MAX_SPREAD_US} us")
    return {"count": len(widths), "min_us": min(widths), "max_us": max(widths),
            "median_us": median(widths), "spread_us": spread,
            "period_min_us": min(periods), "period_max_us": max(periods),
            "first_generated_at": generated[0], "last_generated_at": generated[-1],
            "measured_span_s": generated[-1] - generated[0]}


def summarize_phase(observation):
    if not isinstance(observation, Mapping):
        raise CalibrationError("Phase observation must be an object")
    duration = _finite(observation.get("duration_s"), "duration_s")
    started = _finite(observation.get("started_at"), "started_at")
    ended = _finite(observation.get("ended_at"), "ended_at")
    if not 1 <= duration <= 5 or ended < started + duration:
        raise CalibrationError("Phase must contain its full bounded measurement duration")
    polls, ready = observation.get("polls"), observation.get("ready_polls")
    if (type(polls) is not int or type(ready) is not int or polls <= 0
            or not 0 <= ready <= polls or ready / polls < MIN_READY_FRACTION):
        raise CalibrationError("Receiver was missing or unqualified for too much of the phase")
    if observation.get("ready_at_end") is not True:
        raise CalibrationError("Both receiver channels must remain qualified at the end of the phase")
    channels = observation.get("channels")
    if not isinstance(channels, Mapping) or set(channels) != set(CHANNELS):
        raise CalibrationError("Both steering and throttle channel measurements are required")
    result = {"ready_fraction": ready / polls, "channels": {}}
    for name in CHANNELS:
        try:
            summary = summarize_samples(channels[name])
            if (summary["first_generated_at"] < started or summary["last_generated_at"] > ended
                    or summary["measured_span_s"] < duration * MIN_MEASURED_FRACTION):
                raise CalibrationError("Frames do not cover enough of this measurement phase")
            if ended - summary["last_generated_at"] > FRESHNESS_S:
                raise CalibrationError("Signal was stale at the end of the measurement phase")
        except CalibrationError as exc:
            raise CalibrationError(f"{name}: {exc}") from exc
        result["channels"][name] = summary
    return result


def build_draft_config(base_config, measurements):
    """Return (disabled RCConfig, summaries) from all five raw phase records."""
    if not isinstance(base_config, RCConfig):
        raise CalibrationError("base_config must be a validated RCConfig")
    if not isinstance(measurements, Mapping) or set(measurements) != set(PHASES):
        raise CalibrationError("All five guided measurement phases are required")
    summaries = {}
    for phase in PHASES:
        try:
            summaries[phase] = summarize_phase(measurements[phase])
        except CalibrationError as exc:
            raise CalibrationError(f"{phase}: {exc}") from exc

    draft = base_config.to_dict()
    draft["calibration_confirmed"] = False
    for channel, negative_phase, positive_phase in (
            ("steering", "steer_left", "steer_right"),
            ("throttle", "throttle_reverse", "throttle_forward")):
        neutral = summaries["neutral"]["channels"][channel]
        center = neutral["median_us"]
        deadband = max(20, math.ceil(max(center - neutral["min_us"], neutral["max_us"] - center) + 10))
        negative = summaries[negative_phase]["channels"][channel]["median_us"]
        positive = summaries[positive_phase]["channels"][channel]["median_us"]
        if (negative - center) * (positive - center) >= 0:
            raise CalibrationError(f"{channel}: guided endpoints must lie on opposite sides of neutral")
        if min(abs(negative - center), abs(positive - center)) < max(MIN_ENDPOINT_SPAN_US, 3 * deadband):
            raise CalibrationError(f"{channel}: endpoint travel is too small to distinguish from neutral")
        for phase in PHASES[1:]:
            operated = "steering" if phase.startswith("steer_") else "throttle"
            if channel != operated:
                held = summaries[phase]["channels"][channel]
                if held["min_us"] < center - deadband or held["max_us"] > center + deadband:
                    raise CalibrationError(f"{phase}: {channel} moved; check neutral hold, channel wiring or mixing")
        draft[channel].update(min_us=min(negative, positive), neutral_us=center,
                              max_us=max(negative, positive), deadband_us=deadband,
                              invert=positive < center, calibrated=True)
    # Output duties are inherited unchanged. These measurements establish input
    # mapping only; global confirmation continues to prohibit all motion.
    return RCConfig.from_dict(draft), summaries


def collect_phase(receiver, duration, settle, now=time.monotonic, sleep=time.sleep):
    """Poll qualified snapshots once per new frame; retain raw data for review."""
    sleep(settle)
    started = now()
    deadline = started + duration
    result = {"duration_s": duration, "started_at": started, "ended_at": None,
              "polls": 0, "ready_polls": 0,
              "channels": {name: [] for name in CHANNELS}, "unready_reasons": {}}
    seen = {name: None for name in CHANNELS}
    errors = Counter()
    while now() < deadline:
        observed = now()
        snapshot = receiver.snapshot(now=observed)
        result["polls"] += 1
        result["ready_polls"] += int(snapshot.ready)
        for name in CHANNELS:
            channel = getattr(snapshot, name)
            if not channel.valid or not channel.fresh:
                errors[f"{name}:{channel.error or 'unqualified'}"] += 1
            elif (channel.last_valid_at >= started
                  and channel.last_valid_at != seen[name]):
                result["channels"][name].append({"pulse_us": channel.pulse_us,
                                                "period_us": channel.period_us,
                                                "generated_at": channel.last_valid_at,
                                                "observed_at": observed})
                seen[name] = channel.last_valid_at
        sleep(min(POLL_S, max(0.0, deadline - now())))
    result["ended_at"] = now()
    result["ready_at_end"] = receiver.snapshot(now=result["ended_at"]).ready
    result["unready_reasons"] = dict(errors)
    return result


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("rc_manual_config.example.json"),
                        help="Existing calibration/config to copy output duties from (default: example)")
    parser.add_argument("--pico-port", help="Pico USB serial device; does not require pigpiod")
    parser.add_argument("--output-dir", type=Path, help="New report directory; must not already exist")
    parser.add_argument("--duration", type=float, default=2.5, help="Seconds per hold, 1..5 (default: 2.5)")
    parser.add_argument("--settle", type=float, default=0.25, help="Seconds after Enter before sampling, 0..1")
    args = parser.parse_args(argv)
    if args.pico_port is not None and not args.pico_port.strip():
        parser.error("--pico-port requires a nonempty USB device path")
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 5:
        parser.error("--duration must be finite and between 1 and 5 seconds")
    if not math.isfinite(args.settle) or not 0 <= args.settle <= 1:
        parser.error("--settle must be finite and between 0 and 1 second")
    return args


def main(argv=None):
    args = parse_args(argv)  # --help does not import or connect to pigpio.
    try:
        base = RCConfig.from_dict(json.loads(args.config.read_text(encoding="utf-8")))
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        output_dir = args.output_dir or Path(__file__).resolve().parent / "output" / f"rc_calibration_{stamp}"
        output_dir.mkdir(parents=True, exist_ok=False)
    except (OSError, ValueError, TypeError) as exc:
        print(f"Calibration setup failed: {exc}", file=sys.stderr)
        return 2

    report = {"status": "incomplete", "created_at_utc": stamp,
              "source_config": str(args.config.resolve()), "base_config": base.to_dict(),
              "duration_s": args.duration, "settle_s": args.settle,
              "radio_failsafe_verified": False, "physical_output_verified": False,
              "calibration_confirmed": False, "phases": {}, "errors": [],
              "input_transport": "pico_usb" if args.pico_port else "pigpio",
              "acceptance": {"min_distinct_frames": MIN_SAMPLES, "max_spread_us": MAX_SPREAD_US,
                             "min_endpoint_span_us": MIN_ENDPOINT_SPAN_US,
                             "min_ready_fraction": MIN_READY_FRACTION,
                             "min_measured_fraction": MIN_MEASURED_FRACTION}}
    pi = receiver = None
    try:
        if args.pico_port:
            from rc_pico_receiver import PicoReceiverInput

            receiver = PicoReceiverInput(args.pico_port, base)
        else:
            import pigpio

            pi = pigpio.pi("localhost", 8888, show_errors=False)
            if not pi.connected:
                raise CalibrationError("Cannot connect to existing local pigpiod on port 8888")
            receiver = ReceiverInput(pi, base)
        print("受信機入力だけを測定します。走行プロセスを終了し、モーターを外した状態で行ってください。")
        print("出力 duty は変更しません。中断は Ctrl+C。測定後も走行許可は有効になりません。", flush=True)
        for phase in PHASES:
            input(f"\n{PROMPTS[phase]} Enter で {args.duration:g} 秒測定: ")
            observation = collect_phase(receiver, args.duration, args.settle)
            report["phases"][phase] = observation
            summary = summarize_phase(observation)  # Fail before asking for further holds.
            print("  " + ", ".join(f"{name}: {summary['channels'][name]['median_us']:g} us "
                                     f"({summary['channels'][name]['count']} frames)" for name in CHANNELS), flush=True)
        draft, summaries = build_draft_config(base, report["phases"])
        report["summaries"] = summaries
        report["draft_config"] = "draft_config.json"
        (output_dir / "draft_config.json").write_text(json.dumps(draft.to_dict(), indent=2) + "\n", encoding="utf-8")
        report["status"] = "draft_created"
    except (KeyboardInterrupt, EOFError):
        report["status"] = "interrupted"
        report["errors"].append("Operator interrupted calibration")
    except Exception as exc:
        report["status"] = "failed"
        report["errors"].append(str(exc))
    finally:
        for resource, method in ((receiver, "close"), (pi, "stop")):
            if resource is not None:
                try:
                    getattr(resource, method)()
                except Exception as exc:
                    report["errors"].append(f"Cleanup: {exc}")
                    report["status"] = "failed"
        (output_dir / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    if report["status"] != "draft_created":
        print("Calibration stopped: " + "; ".join(report["errors"]), file=sys.stderr)
        print(f"Raw report: {output_dir / 'report.json'}", file=sys.stderr)
        return 1
    print(f"\nDraft: {output_dir / 'draft_config.json'}\nRaw report: {output_dir / 'report.json'}")
    print("calibration_confirmed=false を維持しています。実際の舵角・駆動方向と送信断時の動作は未確認です。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
