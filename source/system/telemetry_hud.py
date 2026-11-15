"""Small read-only HUD samples; timestamps describe the sampled control frame.

``enabled`` means the selected control mode can accept commands, not that a
wheel is moving. Output pulse widths are accepted commands, never electrical
measurements. This module does not read a clock, hardware, or mutable state.
"""
import math
from collections.abc import Mapping

MAX_SENSOR_AGE_MS = 1000.0
_NORMAL_REASONS = {"", "mode_selected", "ready", "active", "running"}
_EMERGENCY_RECOVERY_TEXT = {
    "release_stop_buttons": "停止ボタンを離してください",
    "release_controls": "R2・L2とスティック・十字キーを離す",
    "press_cross": "×で解除・手動Nへ復帰",
    "controller_disconnected": "コントローラーを接続してください",
    "input_stale": "コントローラー入力の復旧待ち",
}
_REASON_TEXT = {
    "startup_stop": "起動後のモード選択待ち",
    "web_stop": "Webから停止を選択",
    "emergency_button": "緊急停止を保持中",
    "emergency_stop": "緊急停止を保持中",
    "controller_disconnected": "コントローラー未接続",
    "controller_closed": "コントローラー入力を終了",
    "controller_reader_stale": "コントローラー入力の更新停止",
    "input_sync_lost": "コントローラー入力の同期待ち",
    "controls_not_released": "R2・L2を離して待機",
    "awaiting_mode": "走行モードの選択待ち",
    "motor_test_lock": "モーター出力を中立に固定中",
    "neutral_gear": "Nギヤ選択中",
    "gear_neutral": "Nギヤ選択中",
    "reverse_requires_manual": "後退は手動モードで操作",
    "invalid_auto_output": "自動の出力指令が未確定",
    "emergency_cleared": "緊急停止を解除・モード選択待ち",
    "brake_active": "L2ブレーキ操作中",
    "throttle_released": "R2待ち・アクセル中立",
    "mode_stopped": "停止モード",
    "control_inactive": "操作の有効化待ち",
    "vision_neutral": "認識状態による中立指令",
    "output_neutral": "アクセル指令は中立",
    "output_unknown": "出力指令を確認中",
    "command_active": "指令を出力中",
    "status_unavailable": "状態サンプルを確認中",
}


def _number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        result = float(value)
    except (ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _mapping(value):
    return value if isinstance(value, Mapping) else {}


def _text(value, limit=32):
    return str(value)[:limit] if value is not None else ""


def _ascii_text(value, limit=48):
    return _text(value, limit).encode("ascii", "replace").decode("ascii")


def _pulse(channel):
    value = _number(_mapping(channel).get("commanded_pulse_us"))
    return int(round(value)) if value is not None and 750 <= value <= 2250 else None


def _percent(value):
    value = _number(value)
    return round(value, 1) if value is not None and 0 <= value <= 100 else None


def compact_status(full_latest_status, control_snapshot):
    """Build a compact sample without inventing freshness or physical motion.

    Encoder speed requires an explicitly enabled sensor and an actual sensor
    sample age no older than one second. Missing speed is represented by null,
    including while the sensor is disabled; zero is kept only as valid data.
    """
    full = _mapping(full_latest_status)
    control = _mapping(control_snapshot)
    axes = _mapping(control.get("axes"))
    actuator = _mapping(full.get("actuator_output"))
    emergency = control.get("emergency") is True or control.get("reason") in {"emergency_button", "emergency_stop"}
    mode = _text(control.get("mode"), 16).upper()
    if mode not in {"STOP", "MANUAL", "STEER_AUTO", "AUTO"} or emergency:
        mode = "STOP"
    gear = _text(control.get("gear"), 1).upper()
    if gear not in {"D", "N", "R"}:
        gear = "N"
    connected = control.get("connected") is True
    input_age = _number(control.get("input_age_ms"))
    input_age = round(input_age, 1) if input_age is not None and input_age >= 0 else None
    device = control.get("device")
    device_name = _mapping(device).get("name", "") if isinstance(device, Mapping) else device
    sample_time = _number(full.get("time"))
    sample_time_ms = int(round(sample_time * 1000)) if sample_time is not None and 0 <= sample_time < 1e13 else None
    sequence = _number(full.get("frame_index"))
    sequence = int(sequence) if sequence is not None and 0 <= sequence < 1e15 else None
    perception_mode = _text(full.get("mode"), 16)
    confidence = _number(full.get("confidence"))
    confidence = round(confidence, 3) if confidence is not None and 0 <= confidence <= 1 else None
    steer_us = _pulse(actuator.get("pi_steering"))
    throttle_us = _pulse(actuator.get("pi_throttle"))
    command_neutral = throttle_us is not None and abs(throttle_us - 1531) <= 3
    reason = _ascii_text(control.get("reason"), 40)
    manual = mode in {"MANUAL", "STEER_AUTO"}
    enabled = bool(control.get("active") is True and mode != "STOP" and not emergency)
    if manual and (not connected or reason in {"controller_reader_stale", "input_sync_lost", "controls_not_released"}):
        enabled = False
    if full.get("motor_test_lock") is True:
        enabled = False
    if reason == "invalid_auto_output":
        enabled = False

    # Explicit blocks take precedence over a neutral trigger or selected gear.
    if emergency or reason in {"emergency_button", "emergency_stop"}:
        reason = "emergency_stop"
    elif full.get("motor_test_lock") is True:
        reason = "motor_test_lock"
    elif manual and not connected:
        reason = "controller_disconnected"
    elif reason not in _NORMAL_REASONS:
        pass
    elif sample_time_ms is None or sequence is None:
        reason = "status_unavailable"
    elif mode == "STOP":
        reason = "mode_stopped"
    elif not enabled:
        reason = "control_inactive"
    elif manual and gear == "N":
        reason = "neutral_gear"
    elif manual and (_number(axes.get("brake")) or 0) > 0.02:
        reason = "brake_active"
    elif manual and _number(axes.get("throttle")) is not None and _number(axes.get("throttle")) <= 0.02:
        reason = "throttle_released"
    elif command_neutral and mode == "AUTO" and perception_mode in {"LOST", "STOP", "DEGRADED"}:
        reason = "vision_neutral"
    elif command_neutral:
        reason = "output_neutral"
    elif throttle_us is None:
        reason = "output_unknown"
    else:
        reason = "command_active"
    reason_text = _REASON_TEXT.get(reason, "制御状態を確認してください")
    # Recovery hints describe the physical controller's latch; rendering never
    # clears the latch or selects a mode. Keep emergency_stop as the priority key.
    recovery = control.get("emergency_recovery") if emergency else None
    recovery = recovery if isinstance(recovery, str) and recovery in _EMERGENCY_RECOVERY_TEXT else None
    if emergency:
        instruction = _EMERGENCY_RECOVERY_TEXT.get(recovery, "停止ボタンと操作入力を離し、×で解除")
        reason_text = "緊急停止中：" + instruction
    if mode == "AUTO" and reason == "controller_disconnected":
        reason_text = "全自動を継続・コントローラー未接続" if enabled else "コントローラー未接続"

    sensor_age = _number(full.get("encoder_sample_age_ms"))
    speed_kph = _number(full.get("encoder_speed_kph"))
    speed_valid = bool(
        full.get("encoder_enabled") is True
        and sample_time_ms is not None
        and sensor_age is not None and 0 <= sensor_age <= MAX_SENSOR_AGE_MS
        and speed_kph is not None and 0 <= speed_kph <= 1000
    )
    pace = _mapping(full.get("corner_pace"))
    pace_active = bool(mode == "AUTO" and enabled and gear == "D" and pace.get("active") is True)
    phase = pace.get("phase")
    corner_pace = {
        "active": True,
        "phase": phase if isinstance(phase, str) and phase in {"BASELINE", "APPROACH", "CORNER", "RECOVERY", "EXIT", "STRAIGHT"} else None,
        "reason": _ascii_text(pace.get("reason"), 24) or None,
        "cap_percent": _percent(pace.get("cap_percent")),
        "requested_percent": _percent(pace.get("requested_percent")),
        "applied_percent": _percent(pace.get("applied_percent")),
        "limited": pace.get("limited") is True,
    } if pace_active else None
    return {
        "seq": sequence,
        "server_time_ms": sample_time_ms,
        "mode": mode,
        "gear": gear,
        "enabled": enabled,
        "reason": reason,
        "reason_text": _text(reason_text),
        "emergency_recovery": recovery,
        "controller": {"connected": connected, "input_age_ms": input_age, "device": _text(device_name)},
        "perception": {"mode": perception_mode, "confidence": confidence},
        "speed": {"kph": round(speed_kph, 2) if speed_valid else None, "valid": speed_valid},
        "output": {
            "steer_us": steer_us, "throttle_us": throttle_us,
            "forward_percent": _percent(full.get("forward_output_percent")), "measured": False,
        },
        "auto_output_setting_percent": _percent(full.get("auto_output_setting_percent")),
        "corner_pace": corner_pace,
        # API request IDs accept up to 100 characters; keep acknowledgements exact.
        "last_request_id": _text(control.get("request_id"), 100) or None,
        "last_speed_request_id": _text(full.get("last_speed_request_id"), 100) or None,
    }
