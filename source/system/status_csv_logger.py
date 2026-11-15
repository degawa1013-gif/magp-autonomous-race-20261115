#!/usr/bin/env python3
import argparse
import csv
import fcntl
import json
import os
from pathlib import Path
import time
import urllib.request


DEFAULT_URL = "http://127.0.0.1:8000/status.json"
DEFAULT_HZ = 5.0


class StatusLoggerBusy(RuntimeError):
    pass


class StatusLoggerLock:
    """Keep concurrent launchers from appending duplicate rows to one run."""

    path = Path("/tmp/magp-status-csv-logger.lock")

    def __init__(self, path=None):
        self.path = self.path if path is None else Path(path)
        self._fd = None

    def __enter__(self):
        fd = os.open(self.path, os.O_RDWR | os.O_CREAT | os.O_CLOEXEC | os.O_NOFOLLOW, 0o600)
        try:
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise StatusLoggerBusy("status_csv_logger: already running; existing logger retained") from exc
            os.ftruncate(fd, 0)
            os.write(fd, f"{os.getpid()}\n".encode("ascii"))
        except BaseException:
            os.close(fd)
            raise
        self._fd = fd
        return self

    def __exit__(self, *_):
        if self._fd is not None:
            fd, self._fd = self._fd, None
            os.close(fd)
        # Never unlink: a new inode would allow two simultaneous owners.

FIELDS = [
    "time",
    "frame_index",
    "rc_mode",
    "rc_ready",
    "rc_reason",
    "rc_auto_entry_reason",
    "rc_auto_exit_reason",
    "rc_auto_exit_sequence",
    "rc_auto_exit_detail",
    "rc_auto_exit_context",
    "rc_perception_age_s",
    "rc_trigger_required_pulls",
    "rc_trigger_direction",
    "rc_encoder_guard",
    "mode",
    "enabled",
    "reverse_mode",
    "manual_mode",
    "manual_drive",
    "manual_steer",
    "manual_forward_speed",
    "manual_reverse_speed",
    "run_name",
    "run_dir",
    "mask_log_file",
    "mask_log_frame_index",
    "mask_log_time",
    "target_x",
    "last_target_x",
    "confidence",
    "kind",
    "reason",
    "side",
    "lost_frames",
    "good_frames",
    "fit_rms",
    "step_std",
    "raw_center_error",
    "filtered_center_error",
    "used_center_error",
    "d_center_error",
    "filtered_d_center_error",
    "pd_derivative_term",
    "speed_pd_active",
    "speed_pd_switch_kph",
    "center_error_gain_active",
    "center_error_d_gain_active",
    "center_error_gain_high_speed",
    "center_error_d_gain_high_speed",
    "high_freq_error",
    "d_target_x",
    "d_raw_center_error",
    "d_filtered_center_error",
    "d_used_center_error",
    "d_filtered_d_center_error",
    "d_pd_derivative_term",
    "target_steer",
    "steer_smoothing_alpha",
    "plan_steer",
    "plan_speed",
    "output_steer",
    "steer_pwm",
    "d_output_steer",
    "d_steer_pwm",
    "output_speed",
    "speed_boost",
    "boost_mode_enabled",
    "force_farthest_lookahead",
    "force_control_point_index_enabled",
    "force_control_point_index",
    "boost_mode_lookahead_boost",
    "boost_mode_force_farthest_lookahead",
    "straight_speed_active",
    "straight_speed_duration",
    "straight_speed_curve_cut",
    "straight_speed_score_threshold",
    "straight_speed_hold_sec",
    "straight_speed_extra_boost",
    "straight_speed_curve_score_max",
    "reverse_speed",
    "speed_pwm",
    "encoder_gpio",
    "encoder_enabled",
    "encoder_pulse_count",
    "encoder_pulse_rate_hz",
    "encoder_speed_mps",
    "encoder_speed_kph",
    "wheel_diameter_m",
    "encoder_pulses_per_rev",
    "encoder_speed_scale",
    "odom_x_m",
    "odom_y_m",
    "odom_yaw_rad",
    "odom_yaw_deg",
    "odom_signed_speed_mps",
    "odom_wheelbase_m",
    "odom_max_steer_angle_deg",
    "high_speed_active",
    "high_speed_curve_speed_cut",
    "high_speed_boost_threshold",
    "high_speed_encoder_kph_threshold",
    "high_speed_encoder_pps_threshold",
    "hf_suppression",
    "hf_suppression_enabled",
    "straight_score",
    "mean_step",
    "mask_ratio",
    "gated_ratio",
    "line_width",
    "expected_line_width",
    "control_point_index",
    "control_point_position",
    "control_point_x",
    "control_point_y",
    "lookahead_blend",
    "path_heading_step",
    "path_fit_blend",
    "points_count",
    "candidates_count",
    "best_candidate_score",
    "best_candidate_area",
    "best_candidate_centroid_x",
]


def fetch_json(url, timeout):
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def csv_value(value):
    if value is None:
        return ""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (dict, list, tuple)):
        return json.dumps(value, ensure_ascii=False, separators=(",", ":"))
    return value


def choose_output_path(status, fallback_dir):
    run_dir = status.get("run_dir") or fallback_dir
    os.makedirs(run_dir, exist_ok=True)
    return os.path.join(run_dir, "control_log.csv")


def main():
    parser = argparse.ArgumentParser(description="CSV logger for robust_lane_failsafe /status.json")
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument("--hz", type=float, default=DEFAULT_HZ)
    parser.add_argument("--timeout", type=float, default=0.5)
    parser.add_argument("--fallback-dir", default="/home/pi/robust_lane_failsafe/output/manual_logger")
    args = parser.parse_args()

    try:
        with StatusLoggerLock():
            run_logger(args)
    except StatusLoggerBusy as exc:
        print(exc)


def run_logger(args):

    period = 1.0 / max(0.1, args.hz)
    output_path = None
    csv_file = None
    writer = None
    last_frame_index = None

    print(f"status_csv_logger: url={args.url} hz={args.hz:.1f}")
    try:
        while True:
            loop_start = time.monotonic()
            try:
                status = fetch_json(args.url, args.timeout)
            except Exception as exc:
                print(f"status fetch failed: {exc}")
                time.sleep(period)
                continue

            frame_index = status.get("frame_index")
            if frame_index == last_frame_index:
                time.sleep(period)
                continue
            last_frame_index = frame_index

            next_output_path = choose_output_path(status, args.fallback_dir)
            if output_path != next_output_path:
                if csv_file is not None:
                    csv_file.close()
                output_path = next_output_path
                exists = os.path.exists(output_path) and os.path.getsize(output_path) > 0
                csv_file = open(output_path, "a", newline="", encoding="utf-8")
                writer = csv.DictWriter(csv_file, fieldnames=FIELDS, extrasaction="ignore")
                if not exists:
                    writer.writeheader()
                    csv_file.flush()
                print(f"logging to {output_path}")

            row = {field: csv_value(status.get(field)) for field in FIELDS}
            writer.writerow(row)
            csv_file.flush()

            elapsed = time.monotonic() - loop_start
            time.sleep(max(0.0, period - elapsed))
    except KeyboardInterrupt:
        pass
    finally:
        if csv_file is not None:
            csv_file.close()
        print("status_csv_logger: stopped")


if __name__ == "__main__":
    main()
