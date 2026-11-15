"""Configurable receiver-side short-pull recognizer; never writes an actuator.

Run ``update`` on every receiver/control sample, including unchanged samples.
All times are monotonic seconds. A candidate pull is discarded rather than
replayed to the ESC. Ordinary long throttle holds and steering/braking retain
manual priority.
"""

from dataclasses import dataclass, replace
import math
from numbers import Real
from typing import Optional


FULL_THRESHOLD = .85
NEUTRAL_THRESHOLD = .08
RISE_GRACE_S = .100
MIN_FULL_S = .040
MAX_FULL_S = .250
NEUTRAL_STABLE_S = .040
MAX_GAP_S = .300
TOTAL_WINDOW_S = 1.200
REQUIRED_PULLS = 2
_EPSILON = 1e-9


@dataclass(frozen=True)
class TriggerGestureResult:
    effective_throttle: float
    count: int = 0
    active: bool = False
    toggle_to: Optional[str] = None
    reason: str = ""


def _finite(value):
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


class TriggerToggleGesture:
    def __init__(self, required_pulls=REQUIRED_PULLS, direction="forward"):
        if type(required_pulls) is not int or required_pulls not in (2, 3):
            raise ValueError("required_pulls must be 2 or 3")
        self.required_pulls = required_pulls
        if direction not in ("forward", "reverse"):
            raise ValueError("direction must be forward or reverse")
        self.direction = direction
        self.direction_sign = 1 if direction == "forward" else -1
        self.completion_reason = "TRIPLE_PULL" if required_pulls == 3 else "DOUBLE_PULL"
        self._last_at = None
        self._first_at = self._rise_at = self._full_at = self._completed_at = 0.0
        self._origin = "RC"
        self._clear(False)

    def reset(self, suppress_until_neutral=True):
        self._clear(suppress_until_neutral)
        self._last_at = None

    def _clear(self, suppress):
        self._phase = "BYPASS"
        self._count = 0
        self._neutral_at = None
        self._suppress = suppress

    def _result(self, throttle, reason="", toggle_to=None):
        return TriggerGestureResult(throttle, self._count,
                                    self._phase not in ("BYPASS", "ARMED"),
                                    toggle_to, reason)

    def _begin_pull(self, at, throttle):
        self._rise_at = at
        self._neutral_at = None
        self._phase = "FULL" if throttle >= FULL_THRESHOLD else "RISING"
        if self._phase == "FULL":
            self._full_at = at

    def _cancel_at_neutral(self, at, reason):
        self._clear(True)
        self._neutral_at = at
        return self._result(0.0, reason)

    def update(self, steering, throttle, mode, ready=True, *, now):
        # Recognize one positive-oriented waveform internally, while returned
        # throttle retains its original calibrated sign and full range.
        directional_throttle = throttle * self.direction_sign if _finite(throttle) else throttle
        result = self._update(steering, directional_throttle, mode, ready, now=now)
        value = result.effective_throttle * self.direction_sign
        return replace(result, effective_throttle=0.0 if value == 0 else value)

    def _update(self, steering, throttle, mode, ready=True, *, now):
        if not _finite(now) or (self._last_at is not None and now < self._last_at):
            self._clear(True)
            self._last_at = now if _finite(now) else None
            return self._result(0.0, "CLOCK_RESET")
        self._last_at = now
        if (not _finite(steering) or not _finite(throttle) or abs(steering) > 1
                or abs(throttle) > 1 or mode not in ("RC", "AUTO", "MANUAL", "STEER_AUTO")):
            self._clear(True)
            return self._result(0.0, "INVALID_INPUT")
        if not ready:
            self._clear(True)
            return self._result(0.0, "NOT_READY")

        neutral = abs(throttle) <= NEUTRAL_THRESHOLD
        output = 0.0 if neutral else throttle
        if abs(steering) > NEUTRAL_THRESHOLD:
            self._clear(False)
            return self._result(output, "STEERING")
        if throttle < -NEUTRAL_THRESHOLD:
            self._clear(False)
            return self._result(throttle, "REVERSE_OR_BRAKE")
        active = self._phase not in ("BYPASS", "ARMED")
        if active and now - self._first_at > TOTAL_WINDOW_S + _EPSILON:
            self._clear(True)
            return self._result(0.0, "TOTAL_TIMEOUT")
        if self._phase == "GAP" and now - self._completed_at > MAX_GAP_S + _EPSILON:
            self._clear(False)
            self._phase = "ARMED"
            if not neutral:
                self._origin = mode
                self._first_at = now
                self._begin_pull(now, throttle)
            return self._result(0.0, "GAP_TIMEOUT")

        if self._phase == "BYPASS":
            if neutral:
                if self._neutral_at is None:
                    self._neutral_at = now
                if now - self._neutral_at >= NEUTRAL_STABLE_S - _EPSILON:
                    self._phase = "ARMED"
                    self._suppress = False
            else:
                self._neutral_at = None
            return self._result(0.0 if self._suppress else output)
        if self._phase == "ARMED":
            if neutral:
                return self._result(0.0)
            self._origin = mode
            self._first_at = now
            self._begin_pull(now, throttle)
            return self._result(0.0)
        if self._phase == "GAP":
            if not neutral:
                self._begin_pull(now, throttle)
            return self._result(0.0)
        if self._phase == "RISING":
            if neutral:
                return self._cancel_at_neutral(now, "PARTIAL_PULSE")
            if now - self._rise_at > RISE_GRACE_S + _EPSILON:
                self._clear(False)
                return self._result(throttle, "NORMAL_THROTTLE")
            if throttle >= FULL_THRESHOLD:
                self._phase = "FULL"
                self._full_at = now
            return self._result(0.0)
        if self._phase == "FULL":
            held = now - self._full_at
            if held > MAX_FULL_S + _EPSILON:
                self._clear(False)
                return self._result(output, "NORMAL_THROTTLE")
            if throttle >= FULL_THRESHOLD:
                return self._result(0.0)
            if held < MIN_FULL_S - _EPSILON:
                if neutral:
                    return self._cancel_at_neutral(now, "SHORT_PULSE")
                self._clear(True)
                return self._result(0.0, "SHORT_PULSE")
            self._phase = "NEUTRAL" if neutral else "RETURNING"
            if neutral:
                self._neutral_at = now
            return self._result(0.0)
        if self._phase == "RETURNING":
            if now - self._full_at > MAX_FULL_S + _EPSILON:
                self._clear(False)
                return self._result(output, "NORMAL_THROTTLE")
            if neutral:
                self._phase = "NEUTRAL"
                self._neutral_at = now
            elif throttle >= FULL_THRESHOLD:
                self._clear(True)
                return self._result(0.0, "BOUNCED_PULL")
            return self._result(0.0)
        if self._phase == "NEUTRAL":
            if not neutral:
                self._clear(True)
                return self._result(0.0, "NEUTRAL_BOUNCE")
            if now - self._neutral_at < NEUTRAL_STABLE_S - _EPSILON:
                return self._result(0.0)
            self._count += 1
            self._completed_at = self._neutral_at + NEUTRAL_STABLE_S
            self._phase = "GAP"
            if self._count < self.required_pulls:
                return self._result(0.0)
            toggle_to = "AUTO" if self._origin in ("RC", "MANUAL") else "RC"
            self._clear(False)
            self._phase = "ARMED"
            return self._result(0.0, self.completion_reason, toggle_to)
        raise RuntimeError("Unknown trigger gesture phase")
