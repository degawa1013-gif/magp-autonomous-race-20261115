"""DualSense input and control arbitration; this module never writes GPIO.

The reader uses the Linux main gamepad event device, not the motion sensor or
touchpad device. A live reader heartbeat is separate from the last input event:
holding a trigger is valid indefinitely. AUTO can also be requested over the
Web/G2 and does not depend on a connected gamepad.
"""

import errno
import copy
from collections import OrderedDict, deque
import fcntl
import glob
import math
import os
import select
import struct
import threading
import time


EV_SYN, EV_KEY, EV_ABS = 0, 1, 3
SYN_REPORT, SYN_DROPPED = 0, 3
ABS_X, ABS_Z, ABS_RZ = 0, 2, 5
ABS_HAT0X, ABS_HAT0Y = 16, 17
BTN_SOUTH, BTN_EAST, BTN_NORTH, BTN_WEST = 0x130, 0x131, 0x133, 0x134
BTN_TL, BTN_TR, BTN_SELECT, BTN_START = 0x136, 0x137, 0x13A, 0x13B
EMERGENCY_KEYS = frozenset((BTN_TL, BTN_TR, BTN_SELECT, BTN_START))
MODE_KEYS = {BTN_SOUTH: "MANUAL", BTN_WEST: "STEER_AUTO", BTN_NORTH: "AUTO"}
SONY_VENDOR, DUALSENSE_PRODUCT = 0x054C, 0x0CE6
STR_LEFT, STR_NEUTRAL, STR_RIGHT = 7.0, 10.0, 13.0
SPD_FORWARD, SPD_NEUTRAL, SPD_REVERSE = 7.413, 10.48, 13.86
READER_MAX_AGE = 1.0
_EVENT = struct.Struct("@llHHi")
_ABSINFO = struct.Struct("@iiiiii")
_INPUT_ID = struct.Struct("@HHHH")
_EVIOCSCLOCKID = (1 << 30) | (struct.calcsize("@i") << 16) | (ord("E") << 8) | 0xA0


def _clip(value, low, high):
    return max(low, min(high, value))


class ControlState:
    """Thread-safe, hardware-independent mode/gear/input state."""

    def __init__(self, clock=time.monotonic, auto_min_forward=None):
        if auto_min_forward is not None:
            try:
                auto_min_forward = float(auto_min_forward)
            except (TypeError, ValueError, OverflowError) as error:
                raise ValueError("auto_min_forward must be a finite forward duty") from error
            if (not math.isfinite(auto_min_forward)
                    or not SPD_FORWARD <= auto_min_forward < SPD_NEUTRAL):
                raise ValueError("auto_min_forward must be within the forward duty range")
        self._auto_min_forward = auto_min_forward
        self._clock = clock
        self._lock = threading.RLock()
        self.mode = "STOP"
        self._mode_generation = 0
        self.gear = "N"
        self.active = False
        self.connected = False
        self.emergency = False
        self.reason = "startup_stop"
        self.request_id = None
        self._request_cache = OrderedDict()
        self._device = {}
        self._ranges = {}
        self._values = {}
        self._pressed = set()
        self._last_alive = None
        self._trigger_ready = True
        self._sync_lost = False
        self._pending_recovery = False
        self._speed_steps = deque()
        self._forward_since_brake = False
        self._brake_held = False
        self._brake_authorized = False
        self._output_error = False
        self._axes = {"steer": 0.0, "brake": 0.0, "throttle": 0.0}
        # Report activity is diagnostic only: unchanged controls need not emit
        # gamepad evdev reports, so their age must never gate held inputs.
        self._event_clock_monotonic = False
        self._last_report_received = None
        self._event_queue_delay_ms = None
        self._max_event_queue_delay_ms = None
        self._observed_reports = 0
        self._input_revision = 0
        self._input_change_monotonic = None
        self._output_wakeups = set()

    def register_output_wakeup(self, event):
        """Register a passive Event; output ownership remains with its worker."""
        if not isinstance(event, threading.Event):
            raise TypeError("output wakeup must be a threading.Event")
        with self._lock:
            self._output_wakeups.add(event)

    def unregister_output_wakeup(self, event):
        with self._lock:
            self._output_wakeups.discard(event)

    def _notify_output(self):
        # No user callbacks run under the arbitration lock. Event.set cannot
        # issue GPIO writes or wait for the output worker to acquire this lock.
        for event in self._output_wakeups:
            event.set()

    def _input_changed(self, wakeup=True):
        self._input_revision += 1
        self._input_change_monotonic = self._clock()
        if wakeup:
            self._notify_output()

    def record_report_timing(self, kernel_mono=None, received_mono=None):
        """Measure kernel event queueing, not Bluetooth radio round-trip time."""
        with self._lock:
            if not self.connected:
                return
            received = self._clock() if received_mono is None else received_mono
            if not isinstance(received, (int, float)) or not math.isfinite(received):
                return
            self._last_report_received = received
            self._observed_reports += 1
            self._event_queue_delay_ms = None
            if (self._event_clock_monotonic
                    and isinstance(kernel_mono, (int, float))
                    and math.isfinite(kernel_mono) and 0 <= kernel_mono <= received):
                delay = (received - kernel_mono) * 1000.0
                self._event_queue_delay_ms = delay
                self._max_event_queue_delay_ms = max(self._max_event_queue_delay_ms or 0.0, delay)

    def _update_axes(self):
        for code, name in ((ABS_X, "steer"), (ABS_Z, "brake"), (ABS_RZ, "throttle")):
            low, high = self._ranges.get(code, (0, 255))
            value = self._values.get(code, (low + high) / 2 if code == ABS_X else low)
            if high <= low:
                self._axes[name] = 0.0
                continue
            normalized = _clip((value - low) / (high - low), 0.0, 1.0)
            if code == ABS_X:
                normalized = normalized * 2 - 1
                magnitude = max(0.0, abs(normalized) - 0.04) / 0.96
                self._axes[name] = math.copysign(magnitude, normalized)
            else:
                self._axes[name] = max(0.0, normalized - 0.02) / 0.98

    def connect(self, device, axis_ranges=None, initial_values=None, pressed_keys=()):
        with self._lock:
            self._device = dict(device) if isinstance(device, dict) else {"name": str(device)}
            self._ranges = dict(axis_ranges or {})
            self._values = dict(initial_values or {})
            self._pressed = set(pressed_keys)
            self.connected = True
            self._event_clock_monotonic = self._device.get("event_clock") == "monotonic"
            self._last_report_received = None
            self._event_queue_delay_ms = self._max_event_queue_delay_ms = None
            self._observed_reports = 0
            self._input_changed()
            self._sync_lost = False
            self._pending_recovery = False
            self._speed_steps.clear()
            self._last_alive = self._clock()
            self._update_axes()
            self._trigger_ready = self._axes["throttle"] <= 0.03
            self._forward_since_brake = False
            self._brake_held = self._brake_authorized = False
            if self._pressed & EMERGENCY_KEYS:
                self._emergency_stop()
            elif not self.emergency:
                self.reason = "awaiting_mode" if self.mode == "STOP" else "mode_selected"

    def disconnect(self, reason="controller_disconnected"):
        with self._lock:
            self.connected = False
            self._pending_recovery = False
            self._speed_steps.clear()
            self._pressed.clear()
            self._axes = {"steer": 0.0, "brake": 0.0, "throttle": 0.0}
            self._trigger_ready = False
            self._forward_since_brake = False
            self._brake_held = self._brake_authorized = False
            if not self.emergency:
                self.reason = str(reason)
            self._input_changed()

    def heartbeat(self):
        """Called by the live event-fd loop even when no controls move."""
        with self._lock:
            if self.connected:
                was_available = self._reader_available()
                self._last_alive = self._clock()
                if not was_available and self._reader_available():
                    self._notify_output()

    def _emergency_stop(self):
        self._pending_recovery = False
        self._speed_steps.clear()
        self.emergency = True
        self.active = True
        if self.mode != "STOP":
            self._mode_generation += 1
        self.mode, self.gear = "STOP", "N"
        self.reason = "emergency_button"
        self.request_id = None
        self._forward_since_brake = False
        self._brake_held = self._brake_authorized = False
        self._notify_output()

    def _response(self, ok, reason=None):
        return {"ok": bool(ok), "accepted": bool(ok), "mode": self.mode,
                "gear": self.gear, "reason": reason or self.reason,
                "request_id": self.request_id}

    def request_mode(self, mode, request_id=None):
        with self._lock:
            cache_key = None if request_id is None else (type(request_id).__name__, str(request_id))
            if cache_key in self._request_cache:
                return dict(self._request_cache[cache_key])

            def respond(ok, reason=None):
                if ok:
                    self.request_id = request_id
                    self._notify_output()
                result = self._response(ok, reason)
                result["request_id"] = request_id
                if cache_key is not None:
                    self._request_cache[cache_key] = dict(result)
                    if len(self._request_cache) > 512:
                        self._request_cache.popitem(last=False)
                return result

            mode = str(mode).upper()
            if mode not in ("STOP", "MANUAL", "STEER_AUTO", "AUTO"):
                return respond(False, "unsupported_mode")
            if mode == "STOP":
                self.stop()
                return respond(True)
            if self.emergency:
                return respond(False, "emergency_button")
            # MANUAL must always be available to cancel autonomous motion.
            if mode != "MANUAL":
                if mode == "STEER_AUTO" and not self.connected:
                    return respond(False, "controller_disconnected")
                if self.connected and self._axes["throttle"] > 0.03:
                    return respond(False, "controls_not_released")
            if self.mode != mode or (mode == "AUTO" and self.gear != "D"):
                self._mode_generation += 1
                self._speed_steps.clear()
            self.mode, self.active = mode, True
            if mode == "AUTO":
                self.gear = "D"
            self._trigger_ready = self._axes["throttle"] <= 0.03
            self._forward_since_brake = False
            self._brake_held = self._brake_authorized = False
            self.reason = "mode_selected" if self.connected or mode == "AUTO" else "controller_disconnected"
            return respond(True)

    def stop(self, reason="web_stop"):
        with self._lock:
            self._pending_recovery = False
            self._speed_steps.clear()
            self.active = True
            if self.mode != "STOP":
                self._mode_generation += 1
            self.mode, self.gear = "STOP", "N"
            self.request_id = None
            self._forward_since_brake = False
            self._brake_held = self._brake_authorized = False
            if not self.emergency:
                self.reason = str(reason)
            self._notify_output()
            return self._response(True)

    def _emergency_recovery(self, ignore_pressed=()):
        if not self.connected:
            return "controller_disconnected"
        if not self._reader_available():
            return "input_stale"
        if self._pressed & EMERGENCY_KEYS:
            return "release_stop_buttons"
        if ((self._pressed - set(ignore_pressed)) or abs(self._axes["steer"]) > 0.05
                or self._axes["throttle"] > 0.03 or self._axes["brake"] > 0.03
                or self._values.get(ABS_HAT0X, 0) or self._values.get(ABS_HAT0Y, 0)):
            return "release_controls"
        return "press_cross"

    def clear_emergency(self, request_id=None):
        with self._lock:
            cache_key = None if request_id is None else (type(request_id).__name__, str(request_id))
            if cache_key in self._request_cache:
                return dict(self._request_cache[cache_key])

            def respond(ok, reason=None):
                result = self._response(ok, reason)
                result["request_id"] = request_id
                if cache_key is not None:
                    self._request_cache[cache_key] = dict(result)
                    if len(self._request_cache) > 512:
                        self._request_cache.popitem(last=False)
                return result

            recovery = self._emergency_recovery()
            if recovery != "press_cross":
                reason = {"controller_disconnected": "controller_disconnected",
                          "input_stale": "controller_reader_stale"}.get(recovery, "controls_not_released")
                return respond(False, reason)
            self.emergency = False
            self.stop("emergency_cleared")
            self.request_id = request_id
            return respond(True)

    def feed_event(self, event_type, code, value):
        with self._lock:
            if not self.connected:
                return
            if event_type == EV_SYN and code == SYN_DROPPED:
                self._sync_lost = True
                self._pending_recovery = False
                self._speed_steps.clear()
                self._trigger_ready = False
                self.reason = "input_sync_lost" if not self.emergency else self.reason
                self._input_changed()
                return
            if self._sync_lost:
                return
            if event_type == EV_SYN and code == SYN_REPORT:
                pending = self._pending_recovery
                self._pending_recovery = False
                if (pending and self.emergency
                        and self._emergency_recovery(ignore_pressed=(BTN_SOUTH,)) == "press_cross"):
                    self.emergency = False
                    self.stop("emergency_cleared")
                    self._mode_generation += 1
                    self.mode = "MANUAL"
                    self.reason = "mode_selected"
                    self._trigger_ready = True
                return
            if event_type == EV_ABS:
                previous_value = self._values.get(code, 0)
                if value != previous_value:
                    self._input_changed(code in (ABS_X, ABS_Z, ABS_RZ, ABS_HAT0X, ABS_HAT0Y))
                self._values[code] = value
                self._update_axes()
                if self._axes["throttle"] <= 0.03:
                    self._trigger_ready = True
                if not self.emergency:
                    previous_gear = self.gear
                    if code == ABS_HAT0Y and value in (-1, 1):
                        if self.mode == "AUTO":
                            if value != previous_value:
                                self._speed_steps.append(1 if value == -1 else -1)
                        else:
                            self.gear = "D" if value == -1 else "R"
                    elif code == ABS_HAT0X and value == -1:
                        self.gear = "N"
                        self._speed_steps.clear()
                    if self.gear != previous_gear:
                        self._notify_output()
                return
            if event_type != EV_KEY:
                return
            was_pressed = code in self._pressed
            if value == 0:
                if was_pressed:
                    self._input_changed()
                self._pressed.discard(code)
                return
            if not was_pressed:
                self._input_changed()
            self._pressed.add(code)
            if code in EMERGENCY_KEYS and value == 1:
                # The reader also accepts this edge urgently before the full
                # report. Replaying it in order must cancel a Cross recovery
                # that appeared earlier in that same completed report.
                self._emergency_stop()
                return
            if value != 1 or was_pressed:
                return
            if code == BTN_SOUTH and self.emergency:
                # Require neutral both at the new Cross edge and at the end
                # of this input report; later axes in the same report may move.
                self._pending_recovery = (
                    self._emergency_recovery(ignore_pressed=(BTN_SOUTH,)) == "press_cross"
                )
            elif code in MODE_KEYS:
                self.request_mode(MODE_KEYS[code])
            # Circle, PS, touchpad and microphone buttons do not select a mode.

    def feed_report(self, events):
        """Commit one complete evdev report without exposing partial axes.

        ``feed_event`` stays available for individual synthetic inputs and
        urgent stop/drop events. The real reader supplies complete reports,
        including their SYN_REPORT, so recovery sees all later axes together.
        """
        with self._lock:
            for event_type, code, value in events:
                self.feed_event(event_type, code, value)

    def _reader_available(self):
        return (self.connected and not self._sync_lost and self._last_alive is not None
                and self._clock() - self._last_alive <= READER_MAX_AGE)

    def snapshot(self):
        with self._lock:
            age = None if self._last_alive is None else max(0.0, (self._clock() - self._last_alive) * 1000)
            reason = self.reason
            if self.connected and not self._reader_available() and not self._sync_lost and not self.emergency:
                reason = "controller_reader_stale"
            if not self.emergency and self.mode == "AUTO" and self.gear != "D":
                reason = "reverse_requires_manual" if self.gear == "R" else "gear_neutral"
            if self._output_error and not self.emergency:
                reason = "invalid_auto_output"
            report_age = (None if self._last_report_received is None else
                          max(0.0, (self._clock() - self._last_report_received) * 1000))
            return {"mode": self.mode, "gear": self.gear, "mode_generation": self._mode_generation,
                    "connected": self.connected, "active": self.active,
                    "emergency": self.emergency, "reason": reason,
                    "input_age_ms": None if age is None else round(age, 1),
                    "input_timing": {
                        "clock_monotonic": self._event_clock_monotonic,
                        "report_age_ms": None if report_age is None else round(report_age, 3),
                        "event_queue_delay_ms": (None if self._event_queue_delay_ms is None else
                                                 round(self._event_queue_delay_ms, 3)),
                        "max_event_queue_delay_ms": (None if self._max_event_queue_delay_ms is None else
                                                     round(self._max_event_queue_delay_ms, 3)),
                        "observed_reports": self._observed_reports,
                        "input_revision": self._input_revision,
                        "input_change_monotonic": self._input_change_monotonic,
                    },
                    "axes": dict(self._axes), "device": dict(self._device),
                    "request_id": self.request_id,
                    "emergency_recovery": self._emergency_recovery() if self.emergency else None}

    def apply(self, auto_steer, auto_speed):
        with self._lock:
            self._output_error = False
            if self.emergency or self.mode == "STOP":
                return STR_NEUTRAL, SPD_NEUTRAL
            try:
                if self.mode != "MANUAL" and not math.isfinite(float(auto_steer)):
                    raise ValueError("non-finite autonomous steering")
                if self.mode == "AUTO" and not math.isfinite(float(auto_speed)):
                    raise ValueError("non-finite autonomous throttle")
            except (TypeError, ValueError, OverflowError):
                self._output_error = True
                return STR_NEUTRAL, SPD_NEUTRAL
            available = self._reader_available()
            if self.mode != "AUTO" and not available:
                return STR_NEUTRAL, SPD_NEUTRAL
            steer = (STR_NEUTRAL + 3 * self._axes["steer"]
                     if self.mode == "MANUAL" else float(auto_steer))
            steer = _clip(steer, STR_LEFT, STR_RIGHT)
            if self.mode == "AUTO":
                speed = (_clip(float(auto_speed), SPD_FORWARD, SPD_REVERSE)
                         if self.gear == "D" else SPD_NEUTRAL)
                if (self.gear == "D" and self._auto_min_forward is not None
                        and SPD_FORWARD <= speed < SPD_NEUTRAL):
                    speed = min(speed, self._auto_min_forward)
            else:
                throttle = self._axes["throttle"] if self._trigger_ready else 0.0
                endpoint = SPD_FORWARD if self.gear == "D" else SPD_REVERSE
                speed = SPD_NEUTRAL if self.gear == "N" else SPD_NEUTRAL + throttle * (endpoint - SPD_NEUTRAL)

            brake = self._axes["brake"] if available else 0.0
            if brake > 0.03:
                if not self._brake_held:
                    self._brake_authorized = self._forward_since_brake
                    self._forward_since_brake = False
                self._brake_held = True
                # TBLE's first reverse-side command brakes forward motion.
                # A new forward command must precede another brake press;
                # repeated L2 presses at rest cannot select reverse.
                speed = (SPD_NEUTRAL + brake * (SPD_REVERSE - SPD_NEUTRAL)
                         if self._brake_authorized else SPD_NEUTRAL)
            else:
                self._brake_held = self._brake_authorized = False
                if speed < SPD_NEUTRAL - 1e-6:
                    self._forward_since_brake = True
            return float(steer), float(speed)

    def apply_and_snapshot(self, auto_steer, auto_speed):
        """Return the output decision and its status under one state lock."""
        with self._lock:
            steer, speed = self.apply(auto_steer, auto_speed)
            return steer, speed, self.snapshot()

    def preview_and_snapshot(self, auto_steer, auto_speed):
        """Inspect arbitration without changing the applied brake history."""
        with self._lock:
            return copy.copy(self).apply_and_snapshot(auto_steer, auto_speed)

    def drain_speed_steps(self):
        """Consume AUTO D-pad speed steps; the main loop owns speed limits."""
        with self._lock:
            if self.mode != "AUTO" or self.emergency or not self._reader_available():
                self._speed_steps.clear()
                return []
            steps = list(self._speed_steps)
            self._speed_steps.clear()
            return steps

    def apply_speed_steps(self, current, step_size, minimum, maximum):
        """Consume and apply valid AUTO speed steps under one state lock."""
        current, step_size, minimum, maximum = (
            float(value) for value in (current, step_size, minimum, maximum)
        )
        if (not all(math.isfinite(value) for value in (current, step_size, minimum, maximum))
                or step_size < 0 or minimum > maximum):
            raise ValueError("speed setting and limits must be finite and ordered")
        with self._lock:
            value = current
            for direction in self.drain_speed_steps():
                value = _clip(value + direction * step_size, minimum, maximum)
            return value


def _ioc_read(fd, number, size):
    buffer = bytearray(size)
    fcntl.ioctl(fd, (2 << 30) | (size << 16) | (ord("E") << 8) | number, buffer, True)
    return buffer


def _has_bit(bits, code):
    return code // 8 < len(bits) and bool(bits[code // 8] & (1 << (code % 8)))


def _set_event_clock(fd):
    """Older kernels/devices may reject this diagnostic timestamp selection."""
    try:
        fcntl.ioctl(fd, _EVIOCSCLOCKID, struct.pack("@i", time.CLOCK_MONOTONIC))
    except OSError:
        return "realtime"
    return "monotonic"


def _device_details(fd, path):
    bus, vendor, product, version = _INPUT_ID.unpack(_ioc_read(fd, 0x02, _INPUT_ID.size))
    if (vendor, product) != (SONY_VENDOR, DUALSENSE_PRODUCT) or bus not in (3, 5):
        return None
    keys = _ioc_read(fd, 0x20 + EV_KEY, 96)
    axes = _ioc_read(fd, 0x20 + EV_ABS, 8)
    # Motion/touchpad event nodes share the USB/Bluetooth identity.
    if not _has_bit(keys, BTN_SOUTH) or not all(_has_bit(axes, c) for c in (ABS_X, ABS_Z, ABS_RZ)):
        return None
    name = bytes(_ioc_read(fd, 0x06, 256)).split(b"\0", 1)[0].decode("utf-8", "replace")
    return {"path": path, "name": name, "bus": "USB" if bus == 3 else "Bluetooth",
            "vendor": f"{vendor:04x}", "product": f"{product:04x}"}


def _current_values(fd):
    ranges, values = {}, {}
    for code in (ABS_X, ABS_Z, ABS_RZ, ABS_HAT0X, ABS_HAT0Y):
        value, low, high, fuzz, flat, resolution = _ABSINFO.unpack(_ioc_read(fd, 0x40 + code, _ABSINFO.size))
        if high <= low:
            raise ValueError("gamepad axis has no usable range")
        ranges[code], values[code] = (low, high), value
    keys = _ioc_read(fd, 0x18, 96)
    pressed = [code for code in range(len(keys) * 8) if _has_bit(keys, code)]
    return ranges, values, pressed


class DualSenseReader:
    """Discover and read a Sony DualSense main gamepad evdev node.

    No pairing, permissions, device grabbing, GPIO or external commands occur
    here. Linux removal/HUP/read errors are treated as disconnection. A live fd
    heartbeat does not certify delivery of physical Bluetooth radio packets.
    """

    def __init__(self, state, device_path=None):
        self.state = state
        self.device_path = device_path
        self._stop = threading.Event()
        self._thread = None
        self._fd = None
        self.error = None

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return self
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="dualsense-reader", daemon=True)
        self._thread.start()
        return self

    def close(self):
        self._stop.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=2.0)
        self.state.disconnect("controller_closed")

    def _open(self):
        denied = False
        paths = [self.device_path] if self.device_path else sorted(glob.glob("/dev/input/event*"))
        for path in paths:
            fd = None
            try:
                fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC)
                details = _device_details(fd, path)
                if details is None:
                    continue
                details["event_clock"] = _set_event_clock(fd)
                ranges, values, pressed = _current_values(fd)
                self.state.connect(details, ranges, values, pressed)
                self.error = None
                chosen = fd
                fd = None
                return chosen
            except (OSError, ValueError) as error:
                denied = denied or isinstance(error, PermissionError)
            finally:
                if fd is not None:
                    os.close(fd)
        self.error = "controller_permission_denied" if denied else "controller_unavailable"
        return None

    def _read(self, fd):
        poller = select.poll()
        poller.register(fd, select.POLLIN | select.POLLERR | select.POLLHUP | select.POLLNVAL)
        pending = b""
        report = []
        resync = False
        while not self._stop.is_set():
            events = poller.poll(100)
            for _, flags in events:
                if (flags & (select.POLLERR | select.POLLNVAL)
                        or (flags & select.POLLHUP and not flags & select.POLLIN)):
                    raise OSError(errno.ENODEV, "controller event device disconnected")
                try:
                    chunk = os.read(fd, _EVENT.size * 64)
                except BlockingIOError:
                    continue
                if not chunk:
                    raise OSError(errno.ENODEV, "controller event device closed")
                received_mono = self.state._clock()
                pending += chunk
                size = len(pending) // _EVENT.size * _EVENT.size
                # Multiple reports already queued in this read are committed
                # before output wakes, avoiding writes of intermediate axes.
                with self.state._lock:
                    for offset in range(0, size, _EVENT.size):
                        seconds, microseconds, event_type, code, value = _EVENT.unpack_from(pending, offset)
                        if event_type == EV_SYN and code == SYN_DROPPED:
                            report.clear()
                            self.state.feed_event(event_type, code, value)
                            resync = True
                        elif resync:
                            if event_type == EV_SYN and code == SYN_REPORT:
                                ranges, values, pressed = _current_values(fd)
                                self.state.connect(self.state.snapshot()["device"], ranges, values, pressed)
                                resync = False
                        else:
                            # A stop press never waits for a damaged or split
                            # report to finish. Replaying its edge in the
                            # completed report is harmless: STOP is latched.
                            if event_type == EV_KEY and code in EMERGENCY_KEYS and value == 1:
                                self.state.feed_event(event_type, code, value)
                            report.append((event_type, code, value))
                            if event_type == EV_SYN and code == SYN_REPORT:
                                self.state.feed_report(report)
                                report.clear()
                            elif len(report) > 1024:
                                raise OSError(errno.EOVERFLOW, "controller input report has no boundary")
                        if event_type == EV_SYN and code == SYN_REPORT:
                            self.state.record_report_timing(seconds + microseconds / 1e6, received_mono)
                pending = pending[size:]
            self.state.heartbeat()

    def _run(self):
        while not self._stop.is_set():
            fd = self._open()
            if fd is None:
                self._stop.wait(0.5)
                continue
            self._fd = fd
            try:
                self._read(fd)
            except (OSError, ValueError):
                self.state.disconnect()
            finally:
                self._fd = None
                os.close(fd)
        self.state.disconnect("controller_closed")
