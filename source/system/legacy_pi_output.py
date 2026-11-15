"""Run the original follower's duty values on the calibrated direct Pi outputs.

The caller owns the pigpio connection. This adapter has no receiver, mode
arbitration, background worker, automatic start, or reconnection behavior.
"""

import math
import numbers
import threading


STEERING_GPIO = 19  # Pi physical pin 35, hardware PWM channel 1.
THROTTLE_GPIO = 12  # Pi physical pin 32, hardware PWM channel 0.
OUTPUT_HZ = 50
LOGICAL_HZ = 70
STEERING_NEUTRAL_US = 1530
THROTTLE_NEUTRAL_US = 1531
MIN_PULSE_US = 750
MAX_PULSE_US = 2250


def _duty(value, name):
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise ValueError(name + " must be a finite duty percentage")
    value = float(value)
    if not math.isfinite(value) or not 0 <= value <= 100:
        raise ValueError(name + " must be a finite duty percentage in 0..100")
    return value


def convert_legacy_pair(steering_duty, throttle_duty):
    """Convert old percentages into the two calibrated physical pulse widths.

    Steering retains the original +/-3 percentage point span, shifted to the
    measured 1530 us center. Throttle uses the current asymmetric endpoints:
    7.413 / 10.48 / 13.86 percent correspond to 1059 / 1531 / 1980 us.
    """
    steering = _duty(steering_duty, "steering")
    throttle = _duty(throttle_duty, "throttle")
    if not 7.0 <= steering <= 13.0:
        raise ValueError("steering must stay within the original 7..13 percent span")
    steering_us = round(STEERING_NEUTRAL_US + (steering - 10.0) * 10000 / LOGICAL_HZ)
    endpoint_duty, endpoint_us = ((7.413, 1059) if throttle <= 10.48 else (13.86, 1980))
    throttle_us = round(THROTTLE_NEUTRAL_US + (throttle - 10.48)
                        * (endpoint_us - THROTTLE_NEUTRAL_US) / (endpoint_duty - 10.48))
    if not all(MIN_PULSE_US <= pulse <= MAX_PULSE_US
               for pulse in (steering_us, throttle_us)):
        raise ValueError("converted output is outside the 750..2250 us pulse range")
    return steering_us, throttle_us


class LegacyPiOutput:
    """One direct Pi output owner; failures attempt both calibrated neutrals.

    ``write`` takes percentages used by the original 70 Hz program, while the
    actual GPIO pulses are generated at 50 Hz. Unchanged accepted pulse widths
    continue in hardware without another RPC. Explicit neutral and close calls
    always attempt both outputs, allowing a failed shutdown write to be retried.
    ``pi.stop()`` remains the caller's responsibility.
    """

    def __init__(self, pi):
        if not callable(getattr(pi, "hardware_PWM", None)):
            raise ValueError("a connected pigpio hardware_PWM provider is required")
        if getattr(pi, "connected", True) is False:
            raise ValueError("pigpio is not connected")
        self.pi = pi
        self._lock = threading.RLock()
        self._closed = False
        self._commanded_us = {STEERING_GPIO: None, THROTTLE_GPIO: None}
        try:
            # GPIO18 shares PWM0 with GPIO12; GPIO13 shares PWM1 with GPIO19.
            # Disconnect their pads without stopping either shared PWM channel.
            set_mode = getattr(pi, "set_mode", None)
            set_pull = getattr(pi, "set_pull_up_down", None)
            for gpio in (18, 13):
                if callable(set_mode):
                    self._check_result(set_mode(gpio, 0), "set_mode")  # INPUT
                if callable(set_pull):
                    self._check_result(set_pull(gpio, 0), "set_pull_up_down")  # PUD_OFF
        except BaseException as error:
            self._try_neutral_after(error)
            raise
        self._neutral_independently()

    @staticmethod
    def _check_result(result, operation):
        if result is not None and (type(result) is not int or result < 0):
            raise OSError(operation + " failed: " + repr(result))

    def _write_us(self, gpio, pulse_us):
        try:
            self._check_result(self.pi.hardware_PWM(gpio, OUTPUT_HZ, pulse_us * OUTPUT_HZ),
                               "hardware_PWM")
        except BaseException:
            # A failed RPC cannot establish what the daemon accepted. Never
            # suppress a later write using an acknowledgement from before it.
            self._commanded_us[gpio] = None
            raise
        self._commanded_us[gpio] = pulse_us

    def _neutral_independently(self):
        first_error = None
        for gpio, pulse_us in ((THROTTLE_GPIO, THROTTLE_NEUTRAL_US),
                               (STEERING_GPIO, STEERING_NEUTRAL_US)):
            try:
                self._write_us(gpio, pulse_us)
            except BaseException as error:
                if first_error is None:
                    first_error = error
        if first_error is not None:
            raise first_error

    def _try_neutral_after(self, original_error):
        try:
            self._neutral_independently()
        except BaseException as neutral_error:
            add_note = getattr(original_error, "add_note", None)
            if callable(add_note):
                add_note("Neutral output also failed: " + repr(neutral_error))

    def write(self, steering_duty, throttle_duty):
        with self._lock:
            if self._closed:
                raise RuntimeError("legacy Pi output is closed")
            try:
                steering_us, throttle_us = convert_legacy_pair(steering_duty, throttle_duty)
                # Validate both duties before touching either output. Compare
                # physical microseconds: different floats may quantize to the
                # same pulse, and each independent PWM channel keeps running.
                if self._commanded_us[STEERING_GPIO] != steering_us:
                    self._write_us(STEERING_GPIO, steering_us)
                if self._commanded_us[THROTTLE_GPIO] != throttle_us:
                    self._write_us(THROTTLE_GPIO, throttle_us)
            except BaseException as error:
                self._try_neutral_after(error)
                raise
            return True

    def neutral(self):
        with self._lock:
            self._neutral_independently()

    def status(self):
        """Describe accepted pigpio commands, not measured electrical signals."""
        with self._lock:
            def channel(gpio):
                pulse = self._commanded_us[gpio]
                return {"gpio": gpio, "frequency_hz": OUTPUT_HZ,
                        "commanded_pulse_us": pulse,
                        "duty_per_million": None if pulse is None else pulse * OUTPUT_HZ,
                        "measured_physical": False}

            return {"transport": "pi_direct_pwm", "logical_frequency_hz": LOGICAL_HZ,
                    "frequency_hz": OUTPUT_HZ, "closed": self._closed,
                    "measured_physical": False,
                    "pi_steering": channel(STEERING_GPIO),
                    "pi_throttle": channel(THROTTLE_GPIO)}

    def close(self):
        with self._lock:
            self._closed = True
            self._neutral_independently()
