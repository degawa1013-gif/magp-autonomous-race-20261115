"""Apply current driver controls without waiting for camera processing.

This coordinator is the only writer after ``start``. The camera publishes
autonomous plans and can request an immediate serialized refresh; driver
arbitration and the physical write share ControlState's RLock so
that an older AUTO decision cannot overwrite an accepted STOP or manual mode.
Hardware PWM continues between writes; unchanged commands need no refresh.
"""

import copy
import math
import threading
import time

from dualsense_control import SPD_FORWARD, SPD_NEUTRAL, SPD_REVERSE, STR_NEUTRAL


class FastControlOutput:
    def __init__(self, control, output, motor_test_lock=False, interval=0.01,
                 clock=time.monotonic):
        interval = float(interval)
        if not math.isfinite(interval) or interval <= 0:
            raise ValueError("output interval must be finite and positive")
        self.control = control
        self.output = output
        self.motor_test_lock = bool(motor_test_lock)
        self.interval = interval
        self._clock = clock
        self._condition = threading.Condition()
        self._wakeup = threading.Event()
        self._cycle_lock = threading.Lock()
        self._thread = None
        self._stopping = False
        self._closed = False
        self._error = None
        self._plan = (STR_NEUTRAL, SPD_NEUTRAL, None, self._clock(), 0,
                      control.snapshot().get("mode_generation", 0))
        self._decision = None
        self._write_count = 0
        self._input_change_to_output_ms = None
        self._last_pair = None
        self._last_write_at = None

    def start(self):
        # Match the state -> status lock order used by the sole output writer.
        with self.control._lock:
            with self._condition:
                if self._closed or self._stopping:
                    raise RuntimeError("control output is closed")
                if self._thread is None:
                    self.control.register_output_wakeup(self._wakeup)
                    try:
                        self._thread = threading.Thread(target=self._run,
                                                        name="driver-output", daemon=True)
                        self._thread.start()
                    except BaseException:
                        self._thread = None
                        self.control.unregister_output_wakeup(self._wakeup)
                        raise
        self.check_error()
        return self

    def publish_auto(self, steering, throttle, forward_limit=None, mode_generation=None):
        steering, throttle = float(steering), float(throttle)
        if (not math.isfinite(steering) or not math.isfinite(throttle)
                or not SPD_FORWARD <= throttle <= SPD_REVERSE):
            raise ValueError("autonomous output must be finite and within throttle limits")
        if forward_limit is not None:
            forward_limit = float(forward_limit)
            if (not math.isfinite(forward_limit)
                    or not SPD_FORWARD <= forward_limit <= SPD_NEUTRAL):
                raise ValueError("forward limit must be a finite forward duty")
        # Read state before taking the plan condition; the output cycle takes
        # the state lock before publishing its status under that condition.
        if mode_generation is None:
            mode_generation = self.control.snapshot().get("mode_generation", 0)
        if type(mode_generation) is not int or mode_generation < 0:
            raise ValueError("mode generation must be a nonnegative integer")
        with self._condition:
            if self._closed or self._stopping or self._error is not None:
                return False
            self._plan = (steering, throttle, forward_limit, self._clock(),
                          self._plan[4] + 1, mode_generation)
            self._wakeup.set()
            self._condition.notify_all()
        return True

    def _sample(self, plan, previous_pair, last_write_at):
        steering, throttle, forward_limit, plan_at, revision, plan_generation = plan
        # ControlState deliberately exposes this reentrant lock to its output
        # coordinator. Taking only apply_and_snapshot's internal lock would
        # leave a STOP-versus-write race after that method returns.
        with self.control._lock:
            current = self.control.snapshot()
            auto_plan_ready = (current["mode"] == "AUTO"
                               and current.get("mode_generation", 0) == plan_generation)
            waiting_for_auto_plan = current["mode"] == "AUTO" and not auto_plan_ready
            # Use neutral inside arbitration as well, so a withheld stale plan
            # cannot mark forward motion in the ESC brake history.
            steering, throttle, control = self.control.apply_and_snapshot(
                steering, SPD_NEUTRAL if waiting_for_auto_plan else throttle)
            if (forward_limit is not None and control["mode"] == "AUTO"
                    and control["gear"] == "D" and not control["emergency"]
                    and throttle < SPD_NEUTRAL):
                throttle = max(throttle, forward_limit)
            if self.motor_test_lock or waiting_for_auto_plan:
                throttle = SPD_NEUTRAL
            pair = (steering, throttle)
            write_started = self._clock()
            changed = pair != previous_pair
            if changed:
                self.output.write(*pair)
                last_write_at = self._clock()
                input_at = control.get("input_timing", {}).get("input_change_monotonic")
                if isinstance(input_at, (int, float)) and math.isfinite(input_at):
                    self._input_change_to_output_ms = round(max(0., last_write_at - input_at) * 1000, 3)
            sampled_at = self._clock()
            actuator = self.output.status()
        decision = {
            "steering": steering, "throttle": throttle,
            "vehicle_control": control, "actuator_output": actuator,
            "decision_monotonic": sampled_at,
            "output_monotonic": last_write_at,
            "plan_monotonic": plan_at, "plan_revision": revision,
            "plan_mode_generation": plan_generation,
            "auto_plan_ready": auto_plan_ready,
            "plan_to_decision_ms": round(max(0., sampled_at - plan_at) * 1000, 3),
            "output_write_ms": round((sampled_at - write_started) * 1000, 3) if changed else 0.,
            "input_change_to_output_ms": self._input_change_to_output_ms,
            "applied_input_revision": control.get("input_timing", {}).get("input_revision"),
            "shutdown": False,
        }
        with self._condition:
            self._write_count += int(changed)
            self._decision = decision
        return pair, last_write_at

    def _record_failure(self, error):
        with self._condition:
            if self._error is None:
                self._error = error
            self._stopping = True
            self._wakeup.set()
            self._condition.notify_all()

    def _perform_cycle(self):
        with self._cycle_lock:
            with self._condition:
                if self._stopping or self._closed:
                    return None
                plan = self._plan
            try:
                self._last_pair, self._last_write_at = self._sample(
                    plan, self._last_pair, self._last_write_at)
            except Exception as error:
                self._record_failure(error)
                raise
            return plan[4]

    def refresh(self):
        """Apply the newest plan and driver state without a worker-tick delay."""
        try:
            self._perform_cycle()
        except Exception:
            self.check_error()
            raise
        self.check_error()
        return self.status()

    def _run(self):
        try:
            while True:
                # Clear before sampling. A change during or immediately after
                # this cycle stays set until wait(), so no input wake is lost.
                self._wakeup.clear()
                self._perform_cycle()
                with self._condition:
                    if self._stopping:
                        break
                self._wakeup.wait(self.interval)
        except BaseException as error:
            self._record_failure(error)
        finally:
            try:
                with self._cycle_lock:
                    with self.control._lock:
                        self.output.neutral()
                        closed_at = self._clock()
                        control = self.control.snapshot()
                        actuator = self.output.status()
                with self._condition:
                    self._write_count += 1
                    self._decision = {
                        "steering": STR_NEUTRAL, "throttle": SPD_NEUTRAL,
                        "vehicle_control": control, "actuator_output": actuator,
                        "decision_monotonic": closed_at, "output_monotonic": closed_at,
                        "plan_monotonic": self._plan[3], "plan_revision": self._plan[4],
                        "plan_mode_generation": self._plan[5], "auto_plan_ready": False,
                        "plan_to_decision_ms": None, "output_write_ms": None,
                        "input_change_to_output_ms": None,
                        "applied_input_revision": control.get("input_timing", {}).get("input_revision"),
                        "shutdown": True,
                    }
            except BaseException as error:
                with self._condition:
                    if self._error is None:
                        self._error = error
            finally:
                self.control.unregister_output_wakeup(self._wakeup)
                with self._condition:
                    self._closed = True
                    self._condition.notify_all()

    def status(self):
        with self._condition:
            result = copy.deepcopy(self._decision or {})
            result.setdefault("vehicle_control", {})
            result.setdefault("actuator_output", {})
            result.setdefault("auto_plan_ready", False)
            now = self._clock()
            for stamp, age in (("decision_monotonic", "decision_age_ms"),
                               ("output_monotonic", "output_age_ms")):
                value = result.get(stamp)
                result[age] = None if value is None else round(max(0., now - value) * 1000, 3)
            result.update(interval_ms=self.interval * 1000,
                          wakeup_mode="input_or_plan_event_with_periodic_fallback",
                          running=bool(self._thread is not None and self._thread.is_alive()
                                       and not self._closed),
                          closed=self._closed, write_count=self._write_count,
                          error=None if self._error is None else
                          type(self._error).__name__ + ": " + str(self._error))
            return result

    def check_error(self):
        with self._condition:
            error = self._error
        if error is not None:
            raise RuntimeError("driver output worker failed") from error

    def close(self, timeout=2.0):
        with self._condition:
            self._stopping = True
            self._wakeup.set()
            self._condition.notify_all()
            thread = self._thread
        if thread is None:
            # No worker was started; still satisfy the neutral-on-close contract.
            with self.control._lock:
                self.control.unregister_output_wakeup(self._wakeup)
                self.output.neutral()
            with self._condition:
                self._closed = True
            return True
        if thread is threading.current_thread():
            raise RuntimeError("output worker cannot join itself")
        thread.join(timeout)
        if thread.is_alive():
            raise TimeoutError("driver output worker did not stop")
        self.check_error()
        return True
