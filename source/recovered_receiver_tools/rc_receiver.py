"""Read receiver PWM and arbitrate RC/AUTO permission without driving GPIO.

Only pulse presence is observable here. A receiver that keeps producing pulses
after transmitter loss can still look healthy; its failsafe must be tested on
the bench before calibration is confirmed.
"""

from dataclasses import asdict, dataclass, field, replace
import math
from numbers import Real
from threading import Lock
import time
from typing import Mapping, Optional

from rc_trigger_gesture import NEUTRAL_THRESHOLD, TriggerGestureResult, TriggerToggleGesture


MIN_PULSE_US = 750
MAX_PULSE_US = 2250
MIN_PERIOD_US = 10000
MAX_PERIOD_US = 40000
VALID_FRAMES = 3
FRESHNESS_S = 0.15
PROTECTED_GPIO = frozenset((12, 13, 17, 18, 19))
AUTO_NEUTRAL_QUALIFICATION_S = .150


def _mapping(value, name, allowed):
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be an object")
    unknown = set(value) - set(allowed)
    if unknown:
        raise ValueError(f"Unknown {name} field(s): {sorted(unknown, key=str)}")
    return value


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")


@dataclass(frozen=True)
class ChannelConfig:
    gpio: int
    min_us: float = 1000
    neutral_us: float = 1500
    max_us: float = 2000
    deadband_us: float = 40
    invert: bool = False
    calibrated: bool = False
    forward_saturation_us: float = 0

    def __post_init__(self):
        if type(self.gpio) is not int or self.gpio not in range(2, 28) or self.gpio in PROTECTED_GPIO:
            raise ValueError("Receiver GPIO must be BCM 2..27, excluding actuator 12/13/18/19 and encoder 17")
        for name in ("min_us", "neutral_us", "max_us", "deadband_us", "forward_saturation_us"):
            _number(getattr(self, name), name)
        if not MIN_PULSE_US <= self.min_us < self.neutral_us < self.max_us <= MAX_PULSE_US:
            raise ValueError("Channel calibration must satisfy 750 <= min < neutral < max <= 2250 us")
        if not 0 <= self.deadband_us < min(self.neutral_us - self.min_us, self.max_us - self.neutral_us):
            raise ValueError("deadband_us must fit strictly inside both calibrated spans")
        if type(self.invert) is not bool or type(self.calibrated) is not bool:
            raise ValueError("invert and calibrated must be booleans")
        forward_span = (self.neutral_us - self.min_us if self.invert
                        else self.max_us - self.neutral_us)
        if not 0 <= self.forward_saturation_us < forward_span - self.deadband_us:
            raise ValueError("forward_saturation_us must fit strictly outside the forward deadband")

    def normalize(self, pulse_us):
        offset = pulse_us - self.neutral_us
        if abs(offset) <= self.deadband_us:
            return 0.0
        span = self.max_us - self.neutral_us if offset > 0 else self.neutral_us - self.min_us
        value = math.copysign((abs(offset) - self.deadband_us) / (span - self.deadband_us), offset)
        value = max(-1.0, min(1.0, value))
        value = -value if self.invert else value
        # Opt-in endpoint tolerance retains measured calibration and the full
        # proportional curve below this narrow, forward-only saturation band.
        if value > 0 and span - abs(offset) <= self.forward_saturation_us:
            return 1.0
        return value


@dataclass(frozen=True)
class OutputConfig:
    steering_min: float = 9.4
    steering_neutral: float = 10.0
    steering_max: float = 10.6
    throttle_forward: float = 10.23
    throttle_neutral: float = 10.48
    throttle_reverse: float = 10.63

    def __post_init__(self):
        for name, value in asdict(self).items():
            _number(value, name)
            if not 0 <= value <= 100:
                raise ValueError(f"{name} must be a duty percentage in 0..100")
        if not self.steering_min < self.steering_neutral < self.steering_max:
            raise ValueError("Steering duties must satisfy min < neutral < max")
        if not self.throttle_forward < self.throttle_neutral < self.throttle_reverse:
            raise ValueError("Throttle duties must satisfy forward < neutral < reverse")


@dataclass(frozen=True)
class RCConfig:
    calibration_confirmed: bool = False
    steering: ChannelConfig = field(default_factory=lambda: ChannelConfig(23))
    throttle: ChannelConfig = field(default_factory=lambda: ChannelConfig(24))
    outputs: OutputConfig = field(default_factory=OutputConfig)

    def __post_init__(self):
        if type(self.calibration_confirmed) is not bool:
            raise ValueError("calibration_confirmed must be a boolean")
        if not isinstance(self.steering, ChannelConfig) or not isinstance(self.throttle, ChannelConfig):
            raise ValueError("steering and throttle must be ChannelConfig values")
        if not isinstance(self.outputs, OutputConfig):
            raise ValueError("outputs must be an OutputConfig value")
        if self.steering.gpio == self.throttle.gpio:
            raise ValueError("Steering and throttle receiver GPIOs must be distinct")

    @property
    def calibrated(self):
        return self.calibration_confirmed and self.steering.calibrated and self.throttle.calibrated

    @classmethod
    def from_dict(cls, value):
        value = _mapping(value, "RC config", cls.__dataclass_fields__)
        channels = {}
        for name, gpio in (("steering", 23), ("throttle", 24)):
            raw = _mapping(value.get(name, {}), name, ChannelConfig.__dataclass_fields__)
            channels[name] = ChannelConfig(**{"gpio": gpio, **raw})
        outputs = _mapping(value.get("outputs", {}), "outputs", OutputConfig.__dataclass_fields__)
        return cls(calibration_confirmed=value.get("calibration_confirmed", False),
                   outputs=OutputConfig(**outputs), **channels)

    def to_dict(self):
        return asdict(self)


@dataclass(frozen=True)
class ChannelSnapshot:
    gpio: int
    pulse_us: Optional[int]
    period_us: Optional[int]
    consecutive_frames: int
    neutral_frames: int
    last_valid_at: Optional[float]
    age_s: Optional[float]
    valid: bool
    fresh: bool
    normalized: float
    neutral: bool
    error: str


@dataclass(frozen=True)
class RCSnapshot:
    steering: ChannelSnapshot
    throttle: ChannelSnapshot
    captured_at: float

    @property
    def ready(self):
        return all(channel.valid and channel.fresh for channel in (self.steering, self.throttle))

    @property
    def neutral(self):
        return self.ready and self.steering.neutral and self.throttle.neutral

    def to_dict(self):
        return {**asdict(self), "ready": self.ready, "neutral": self.neutral}


class _ChannelState:
    def __init__(self, config):
        self.config = config
        self.rise = self.period = self.pulse = self.last_valid_at = None
        self.awaiting_fall = False
        self.consecutive = self.neutral_frames = 0
        self.error = "missing"

    def invalidate(self, reason):
        self.consecutive = self.neutral_frames = 0
        self.error = reason

    def edge(self, level, tick, now):
        if level == 1:
            if self.awaiting_fall:
                self.invalidate("missing_fall")
            if self.rise is not None:
                self.period = (tick - self.rise) & 0xFFFFFFFF
                if not MIN_PERIOD_US <= self.period <= MAX_PERIOD_US:
                    self.invalidate("invalid_period")
            self.rise = tick
            self.awaiting_fall = True
        elif level == 0:
            if not self.awaiting_fall:
                self.invalidate("unpaired_fall")
                return
            self.awaiting_fall = False
            self.pulse = (tick - self.rise) & 0xFFFFFFFF
            if not MIN_PULSE_US <= self.pulse <= MAX_PULSE_US:
                self.invalidate("invalid_width")
            elif self.period is None:
                self.invalidate("waiting_for_period")
            elif not MIN_PERIOD_US <= self.period <= MAX_PERIOD_US:
                self.invalidate("invalid_period")
            else:
                self.consecutive += 1
                self.neutral_frames = self.neutral_frames + 1 if self.config.normalize(self.pulse) == 0 else 0
                self.last_valid_at = now
                self.error = "" if self.consecutive >= VALID_FRAMES else "qualifying"
        else:
            self.awaiting_fall = False
            self.invalidate("invalid_level")

    def snapshot(self, now):
        age = None if self.last_valid_at is None else now - self.last_valid_at
        fresh = age is not None and 0 <= age <= FRESHNESS_S
        normalized = 0.0 if self.pulse is None else self.config.normalize(self.pulse)
        return ChannelSnapshot(self.config.gpio, self.pulse, self.period, self.consecutive,
                               self.neutral_frames, self.last_valid_at, age,
                               self.consecutive >= VALID_FRAMES, fresh, normalized,
                               self.neutral_frames >= VALID_FRAMES,
                               self.error if self.consecutive < VALID_FRAMES or fresh else "stale")


class ReceiverState:
    """Hardware independent, thread safe edge decoder (pigpio 32 bit ticks)."""

    def __init__(self, config, now=time.monotonic):
        self.config, self.now = config, now
        self._lock = Lock()
        self._channels = {channel.gpio: _ChannelState(channel)
                          for channel in (config.steering, config.throttle)}

    def edge(self, gpio, level, tick):
        self._edge_at(gpio, level, tick, self.now())

    def _edge_at(self, gpio, level, tick, generated_at):
        with self._lock:
            channel = self._channels.get(gpio)
            if channel is not None:
                channel.edge(level, tick & 0xFFFFFFFF, generated_at)

    def snapshot(self, now=None):
        now = self.now() if now is None else now
        with self._lock:
            return RCSnapshot(self._channels[self.config.steering.gpio].snapshot(now),
                              self._channels[self.config.throttle.gpio].snapshot(now), now)


class ReceiverInput(ReceiverState):
    """Attach callbacks to an existing pigpio connection and existing INPUTs.

    The caller owns the connection. This class never changes modes or pulls,
    writes output, starts pigpiod, or closes the shared connection.
    """

    def __init__(self, pi, config, now=time.monotonic):
        super().__init__(config, now)
        self._pi = pi
        self._callbacks = []
        for gpio in self._channels:
            if pi.get_mode(gpio) != 0:  # pigpio.INPUT
                raise ValueError(f"Receiver GPIO {gpio} is not already INPUT; refusing to change it")
        try:
            for gpio in self._channels:
                self._callbacks.append(pi.callback(gpio, 2, self.edge))  # pigpio.EITHER_EDGE
        except Exception:
            self.close()
            raise

    def _reject_edge(self, gpio, reason):
        with self._lock:
            channel = self._channels.get(gpio)
            if channel is not None:
                channel.invalidate(reason)
                channel.rise = channel.period = None
                channel.awaiting_fall = False

    def edge(self, gpio, level, tick):
        # Read the daemon's current tick instead of treating delivery as the
        # edge time: callbacks can arrive in a backlog. Taking host time before
        # this read conservatively includes command round-trip time in age.
        observed_at = self.now()
        try:
            current_tick = self._pi.get_current_tick()
            if type(current_tick) is not int or not 0 <= current_tick <= 0xFFFFFFFF:
                raise ValueError("Invalid pigpio current tick")
        except Exception:
            self._reject_edge(gpio, "tick_clock_unavailable")
            return
        age_us = (current_tick - tick) & 0xFFFFFFFF
        if age_us > FRESHNESS_S * 1_000_000:
            self._reject_edge(gpio, "overdue_callback")
            return
        self._edge_at(gpio, level, tick, observed_at - age_us / 1_000_000)

    def close(self):
        callbacks, self._callbacks = self._callbacks, []
        first_error = None
        for callback in callbacks:
            try:
                callback.cancel()
            except Exception as exc:
                first_error = first_error or exc
        if first_error is not None:
            raise first_error


@dataclass(frozen=True)
class RCDecision:
    mode: str
    steering_duty: float
    throttle_duty: float
    reason: str
    ready: bool
    fault_latched: bool
    takeover_latched: bool
    raw_steering: float = 0.0
    raw_throttle: float = 0.0
    effective_throttle: float = 0.0
    trigger_pull_count: int = 0
    trigger_toggle_active: bool = False
    toggle_to: Optional[str] = None
    gesture_reason: str = ""
    auto_entry_pending: bool = False
    gesture_neutral_hold: bool = False

    def to_dict(self):
        return asdict(self)


class RCController:
    """Latch mode changes; faults normally require an explicit new request.

    Request booleans are edge triggered. Send False between distinct requests.
    AUTO decisions grant permission only; their returned duties are neutral,
    and the caller must apply its autonomous commands through its output gate.
    Explicit-only mode leaves manual axes direct and changes AUTO only through
    requests; a reverse/brake input still latches a neutral safety stop.
    Opt-in neutral recovery restores only manual permission after new neutral
    source observations; it never restores AUTO permission.
    """

    def __init__(self, config, *, enable_trigger_toggle=False, gesture_only_auto_takeover=False,
                 explicit_mode_only=False, recover_manual_on_neutral=False,
                 trigger_pulls=2, trigger_direction="forward"):
        self.config = config
        if type(enable_trigger_toggle) is not bool:
            raise ValueError("enable_trigger_toggle must be a boolean")
        if (type(gesture_only_auto_takeover) is not bool
                or (gesture_only_auto_takeover and not enable_trigger_toggle)):
            raise ValueError("Gesture-only AUTO control requires trigger toggles")
        if (type(explicit_mode_only) is not bool
                or (explicit_mode_only and enable_trigger_toggle)):
            raise ValueError("Explicit-only mode control requires trigger toggles disabled")
        if (type(recover_manual_on_neutral) is not bool
                or recover_manual_on_neutral and not explicit_mode_only):
            raise ValueError("Neutral manual recovery requires explicit-only modes")
        self.enable_trigger_toggle = enable_trigger_toggle
        self.gesture_only_auto_takeover = gesture_only_auto_takeover
        self.explicit_mode_only = explicit_mode_only
        self.recover_manual_on_neutral = recover_manual_on_neutral
        self._gesture = TriggerToggleGesture(required_pulls=trigger_pulls, direction=trigger_direction)
        self.trigger_pulls = trigger_pulls
        self.trigger_direction = trigger_direction
        self._gesture_result = TriggerGestureResult(0.0)
        self._pending_auto_at = None
        self._raw_steering = self._raw_throttle = 0.0
        self._mode = "NEUTRAL"
        self._armed = self._ever_armed = False
        self._fault_latched = self._takeover_latched = False
        self._previous_auto = self._previous_manual = False
        self._manual_recovery_at = None
        self._manual_recovery_sources = [None, None]
        self._manual_recovery_counts = [0, 0]

    def _reset_manual_recovery(self, now):
        if self.recover_manual_on_neutral:
            if math.isfinite(now):
                self._manual_recovery_at = (now if self._manual_recovery_at is None
                                            else max(now, self._manual_recovery_at))
            self._manual_recovery_sources = [None, None]
            self._manual_recovery_counts = [0, 0]

    def _manual_neutral_qualified(self, snapshot, now):
        """Count new post-stop source frames, never repeated service snapshots."""
        if (not snapshot.neutral or self._raw_steering != 0 or self._raw_throttle != 0):
            self._reset_manual_recovery(now)
            return False
        if self._manual_recovery_at is None or now < self._manual_recovery_at:
            self._reset_manual_recovery(now)
            return False
        sources = (snapshot.steering.last_valid_at, snapshot.throttle.last_valid_at)
        if any(at is None or not math.isfinite(at) or at <= self._manual_recovery_at for at in sources):
            return False
        if any(previous is not None and at < previous
               for at, previous in zip(sources, self._manual_recovery_sources)):
            self._reset_manual_recovery(now)
            return False
        for index, at in enumerate(sources):
            previous = self._manual_recovery_sources[index]
            if previous is None or at > previous:
                self._manual_recovery_sources[index] = at
                self._manual_recovery_counts[index] += 1
        return all(count >= VALID_FRAMES for count in self._manual_recovery_counts)

    def _decision(self, mode, reason, snapshot=None):
        outputs = self.config.outputs
        steering, throttle = outputs.steering_neutral, outputs.throttle_neutral
        if mode == "RC":
            steer = snapshot.steering.normalized
            accel = self._gesture_result.effective_throttle
            steering += steer * (outputs.steering_max - steering if steer >= 0 else steering - outputs.steering_min)
            # Positive calibrated throttle intent is forward (lower ESC duty).
            throttle += accel * (outputs.throttle_forward - throttle if accel >= 0 else throttle - outputs.throttle_reverse)
        return RCDecision(mode, steering, throttle, reason, mode in ("RC", "AUTO"),
                          self._fault_latched, self._takeover_latched,
                          self._raw_steering, self._raw_throttle,
                          self._gesture_result.effective_throttle,
                          self._gesture_result.count, self._gesture_result.active,
                          self._gesture_result.toggle_to, self._gesture_result.reason,
                          self._pending_auto_at is not None,
                          mode == "AUTO" and self.trigger_direction == "reverse"
                          and self._gesture_result.active)

    def _cancel_gesture(self, reason):
        self._gesture.reset(suppress_until_neutral=True)
        self._pending_auto_at = None
        self._gesture_result = TriggerGestureResult(0.0, reason=reason)

    def _qualify_auto_gesture(self, snapshot, now):
        """Give the completed pull's strict receiver-neutral gate up to 150 ms.

        The recognizer's small neutral band may settle before three exact
        calibrated-neutral PWM frames. Only this settling interval is retained;
        perception readiness is never queued here.
        """
        result = self._gesture_result
        if result.toggle_to == "AUTO":
            self._pending_auto_at = now
        if self._pending_auto_at is None:
            return
        age = now - self._pending_auto_at
        if (not math.isfinite(age) or age < 0 or age > AUTO_NEUTRAL_QUALIFICATION_S + 1e-9
                or self._raw_steering != 0 or self._raw_throttle * self._gesture.direction_sign < 0
                or self._raw_throttle * self._gesture.direction_sign > NEUTRAL_THRESHOLD):
            self._pending_auto_at = None
            self._gesture_result = replace(result, toggle_to=None,
                reason="AUTO_NEUTRAL_TIMEOUT" if age > AUTO_NEUTRAL_QUALIFICATION_S + 1e-9
                else "AUTO_NEUTRAL_CANCELLED")
        elif snapshot.neutral:
            self._pending_auto_at = None
            self._gesture_result = replace(result, effective_throttle=0.0,
                toggle_to="AUTO", active=False, reason=self._gesture.completion_reason)
        else:
            self._gesture_result = replace(result, effective_throttle=0.0,
                toggle_to=None, active=True, reason="AUTO_WAITING_FOR_NEUTRAL")

    def evaluate(self, snapshot, auto_requested=False, stop_requested=False, now=None,
                 manual_requested=False):
        auto_edge = bool(auto_requested) and not self._previous_auto
        manual_edge = bool(manual_requested) and not self._previous_manual
        self._previous_auto, self._previous_manual = bool(auto_requested), bool(manual_requested)
        now = snapshot.captured_at if now is None else now
        self._raw_steering = snapshot.steering.normalized
        self._raw_throttle = snapshot.throttle.normalized
        prior_gesture_result = self._gesture_result
        prior_gesture_active = prior_gesture_result.active
        completed_auto_gesture = auto_edge and snapshot.neutral and prior_gesture_result.toggle_to == "AUTO"
        self._gesture_result = prior_gesture_result if completed_auto_gesture else TriggerGestureResult(self._raw_throttle)
        if stop_requested:
            self._cancel_gesture("STOP_REQUESTED")
            self._fault_latched = True
            self._armed = False
            self._mode = "NEUTRAL"
            self._reset_manual_recovery(now)
            return self._decision("STOP", "stop_requested")
        if not self.config.calibrated:
            self._cancel_gesture("CALIBRATION_UNCONFIRMED")
            self._armed = False
            self._mode = "NEUTRAL"
            self._reset_manual_recovery(now)
            return self._decision("NEUTRAL", "calibration_unconfirmed")
        # Recheck age at the decision, so caching a ready snapshot cannot keep
        # an output alive after edge processing stops.
        fresh = all(channel.last_valid_at is not None and
                    0 <= now - channel.last_valid_at <= FRESHNESS_S
                    for channel in (snapshot.steering, snapshot.throttle))
        if not snapshot.ready or not fresh:
            self._cancel_gesture("SIGNAL_INVALID_OR_STALE")
            self._fault_latched = self._fault_latched or self._ever_armed
            self._armed = False
            self._mode = "NEUTRAL"
            self._reset_manual_recovery(now)
            return self._decision("NEUTRAL", "signal_invalid_or_stale")
        if (self._mode == "AUTO" and self._raw_throttle < 0
                and (self.explicit_mode_only or self.gesture_only_auto_takeover
                     and self.trigger_direction == "forward")):
            # Braking remains an immediate safety stop, never an implicit mode
            # handover that could turn a held brake into reverse motion.
            self._cancel_gesture("BRAKE_STOP")
            self._fault_latched = True
            self._armed = False
            self._mode = "NEUTRAL"
            self._reset_manual_recovery(now)
            return self._decision("STOP", "rc_brake_stop")
        gesture_rearmed = False
        if not self._armed and self._fault_latched and self.enable_trigger_toggle and not (auto_edge or manual_edge):
            # A new completed gesture is an explicit rearm request. Keep its
            # detector alive through displaced throttle samples, while the
            # fault continues to force neutral output. Recovered pulses or a
            # single pull alone never restore permission.
            if self._raw_steering != 0 or self._raw_throttle * self._gesture.direction_sign < 0:
                self._cancel_gesture("WAITING_FOR_NEUTRAL")
            else:
                self._gesture_result = self._gesture.update(
                    self._raw_steering, self._raw_throttle, "RC", now=now)
                self._qualify_auto_gesture(snapshot, now)
            if self._gesture_result.toggle_to == "AUTO" and snapshot.neutral:
                self._armed = self._ever_armed = True
                self._fault_latched = False
                self._takeover_latched = False
                self._mode = "RC"
                gesture_rearmed = True
            else:
                return self._decision("NEUTRAL", "rearm_required" if snapshot.neutral else "waiting_for_neutral")
        if not self._armed:
            if not snapshot.neutral:
                self._cancel_gesture("WAITING_FOR_NEUTRAL")
                self._reset_manual_recovery(now)
                return self._decision("NEUTRAL", "waiting_for_neutral")
            if self._fault_latched and not (auto_edge or manual_edge):
                if not (self.recover_manual_on_neutral and self._manual_neutral_qualified(snapshot, now)):
                    self._cancel_gesture("WAITING_FOR_NEUTRAL" if self.recover_manual_on_neutral else "REARM_REQUIRED")
                    return self._decision("NEUTRAL", "waiting_for_neutral" if self.recover_manual_on_neutral else "rearm_required")
                self._takeover_latched = False
            self._armed = self._ever_armed = True
            self._fault_latched = False
            self._mode = "RC"
            self._manual_recovery_at = None
            self._manual_recovery_sources = [None, None]
            self._manual_recovery_counts = [0, 0]
        if self.enable_trigger_toggle and not gesture_rearmed and not completed_auto_gesture:
            ignore_auto_axes = self.gesture_only_auto_takeover and self._mode == "AUTO"
            # Reverse candidates reserve throttle even while manually steering;
            # strict centered steering is still required to commit AUTO.
            ignore_gesture_steering = ignore_auto_axes or self.trigger_direction == "reverse"
            # Configuration already applies the measured steering deadband.
            # Any remaining steering/braking intent bypasses the recognizer.
            if ((self._raw_steering != 0 and not ignore_gesture_steering)
                    or self._raw_throttle * self._gesture.direction_sign < 0):
                self._gesture.reset(suppress_until_neutral=False)
                self._gesture_result = TriggerGestureResult(
                    self._raw_throttle, reason="STEERING" if self._raw_steering else "REVERSE_OR_BRAKE")
            elif auto_edge or manual_edge:
                # Explicit requests cancel incomplete sequences; no stale
                # final release may change mode after a separate command.
                self._cancel_gesture("EXPLICIT_MODE_REQUEST")
                self._gesture_result = self._gesture.update(
                    0.0 if ignore_gesture_steering else self._raw_steering, self._raw_throttle, self._mode, now=now)
            else:
                self._gesture_result = self._gesture.update(
                    0.0 if ignore_gesture_steering else self._raw_steering, self._raw_throttle, self._mode, now=now)
            self._qualify_auto_gesture(snapshot, now)
            if (ignore_auto_axes and self.trigger_direction == "reverse"
                    and self._gesture_result.toggle_to != "RC"
                    and (not self._gesture_result.active or self._gesture_result.reason == "GAP_TIMEOUT")
                    and (self._raw_throttle < -NEUTRAL_THRESHOLD or prior_gesture_active)):
                # A short reverse candidate only reserves neutral. An invalid,
                # incomplete or held candidate must never release AUTO or emit
                # reverse: require a new neutral-qualified gesture to rearm.
                self._cancel_gesture("BRAKE_STOP")
                self._fault_latched = True
                self._armed = False
                self._mode = "NEUTRAL"
                self._reset_manual_recovery(now)
                return self._decision("STOP", "rc_brake_stop")
            if ignore_auto_axes and self._gesture_result.toggle_to != "RC":
                self._gesture_result = replace(self._gesture_result, effective_throttle=0.0)
        intent = self._raw_steering != 0 or self._gesture_result.effective_throttle != 0
        gesture_manual = self._gesture_result.toggle_to == "RC"
        axis_takeover = intent and not (self.gesture_only_auto_takeover or self.explicit_mode_only)
        if self._mode == "AUTO" and (axis_takeover or manual_edge or gesture_manual):
            self._mode = "RC"
            self._takeover_latched = True
        elif manual_edge or gesture_manual:
            self._mode = "RC"
        elif auto_edge and snapshot.neutral:
            self._mode = "AUTO"
            self._takeover_latched = False
        if self._mode == "AUTO":
            return self._decision("AUTO", "auto_requested")
        return self._decision("RC", "rc_takeover" if self._takeover_latched else "manual", snapshot)
