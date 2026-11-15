"""Pico USB receiver input with explicitly enabled, paired actuator commands.

Each request uses a fresh nonce. Source age and bounded round-trip time prevent
queued USB replies from extending signal freshness. No serial reconnection is
attempted after a disconnect.
"""

import json
from dataclasses import replace
import math
import re
import secrets
import sys
import threading
import time

from rc_receiver import (ChannelSnapshot, FRESHNESS_S, MAX_PERIOD_US,
                         MAX_PULSE_US, MIN_PERIOD_US, MIN_PULSE_US, RCConfig,
                         RCSnapshot, VALID_FRAMES)


POLL_S = 0.02
RESPONSE_TIMEOUT_S = 0.05
TRANSACTION_TIMEOUT_S = 0.1
ENCODER_PREPARATION_TIMEOUT_S = 0.2  # Passive allocation only; never an actuator challenge.
MAX_LINE_BYTES = 1024
MAX_ENCODER_LINE_BYTES = 2048
MAX_SKIPPED_LINES = 8
SEQUENCE_MODULUS = 1 << 29
SEQUENCE_MASK = SEQUENCE_MODULUS - 1
ACTUATOR_LEASE_S = 0.15
# Reserve the entire bounded transaction for a nonneutral producer pair. This
# proof does not assume when MCU challenge issuance occurs relative to receipt.
ACTUATOR_PRODUCER_QUEUE_S = ACTUATOR_LEASE_S - TRANSACTION_TIMEOUT_S
ACTUATOR_NEUTRAL = (1530, 1531)


class _ProtocolError(ValueError):
    pass


def _integer(value, minimum, maximum, name):
    if type(value) is not int or not minimum <= value <= maximum:
        raise _ProtocolError(f"Invalid {name}")
    return value


def _object(value, keys, name):
    if not isinstance(value, dict) or set(value) != set(keys):
        raise _ProtocolError(f"Invalid {name} schema")
    return value


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise _ProtocolError("Duplicate JSON key")
        result[key] = value
    return result


class _Channel:
    def __init__(self, config):
        self.config = config
        self.source_seq = self.source_run = None
        self.pulse = self.period = self.last_valid_at = None
        self.consecutive = self.neutral_frames = 0
        self.error = "missing"

    def invalidate(self, reason):
        self.consecutive = self.neutral_frames = 0
        self.error = reason

    def update(self, sample, requested_at):
        seq, run = sample["seq"], sample["valid_run"]
        if self.source_seq is not None:
            delta = (seq - self.source_seq) & SEQUENCE_MASK
            if delta >= SEQUENCE_MODULUS // 2:
                raise _ProtocolError("Source sequence regressed")
        else:
            delta = None
        if run == 0:
            # Missing inputs, invalid pulses and source expiration are ordinary
            # capture states. Keep the other channel's diagnostics intact and
            # remember this sequence/reset without inventing a fresh frame.
            self.source_seq, self.source_run = seq, 0
            self.pulse, self.period = sample["pulse_us"], sample["period_us"]
            self.last_valid_at = None
            self.invalidate("missing" if sample["age_us"] is None else "source_unqualified")
            return
        if delta is not None:
            if delta == 0:
                if (run != self.source_run or sample["pulse_us"] != self.pulse
                        or sample["period_us"] != self.period):
                    raise _ProtocolError("Source changed without a new frame")
                return  # Duplicate frames cannot renew freshness or qualification.
            expected_run = min(SEQUENCE_MASK, self.source_run + delta)
            if run < expected_run:
                self.invalidate("source_qualification_reset")
            elif run > expected_run:
                raise _ProtocolError("Source valid_run exceeds observed frame progression")
        self.source_seq, self.source_run = seq, run
        self.pulse, self.period = sample["pulse_us"], sample["period_us"]
        self.last_valid_at = requested_at - sample["age_us"] / 1_000_000
        self.consecutive += 1
        self.neutral_frames = self.neutral_frames + 1 if self.config.normalize(self.pulse) == 0 else 0
        self.error = "" if self.consecutive >= VALID_FRAMES else "qualifying"

    def snapshot(self, now):
        age = None if self.last_valid_at is None else now - self.last_valid_at
        fresh = age is not None and 0 <= age <= FRESHNESS_S
        valid = self.consecutive >= VALID_FRAMES
        return ChannelSnapshot(self.config.gpio, self.pulse, self.period, self.consecutive,
                               self.neutral_frames, self.last_valid_at, age, valid, fresh,
                               0.0 if self.pulse is None else self.config.normalize(self.pulse),
                               self.neutral_frames >= VALID_FRAMES,
                               "stale" if valid and not fresh else self.error)


class PicoReceiverInput:
    """Nonblocking snapshots backed by a 50 Hz USB request thread.

    Firmware input assignments are fixed: steering GP4 and throttle GP5.
    ``serial_factory`` and ``autostart=False`` permit hardware-free tests;
    ``poll_once`` performs one bounded transaction when no worker is running.
    """

    def __init__(self, port, config, now=time.monotonic, serial_factory=None, autostart=True,
                 encoder_source=None):
        if not isinstance(config, RCConfig) or (config.steering.gpio, config.throttle.gpio) != (4, 5):
            raise ValueError("Pico firmware requires steering gpio=4 (GP4), throttle gpio=5 (GP5)")
        if not isinstance(port, str) or not port.strip():
            raise ValueError("Pico serial port must be a nonempty path")
        self.config, self.now = config, now
        self.encoder_source = encoder_source
        self._encoder_session = secrets.token_hex(8) if encoder_source is not None else None
        self._encoder_prepared = encoder_source is None
        self._encoder_preparing = self._encoder_preparation_failed = False
        self._encoder_prepared_identity = None
        # RC2 delivery acknowledgement is independent of usable measurements.
        # A latched source fault must not replay an old packed page forever,
        # but discarding delivered edges must never revive that source.
        self._encoder_transport_cursor = self._encoder_transport_total = 0
        self._encoder_transport_identity = None
        self._encoder_transport_mode = "rc1" if encoder_source is None else "rc2"
        self._encoder_manual_transition_neutral = False
        self._lock = threading.Lock()
        self._actuator_changed = threading.Condition(self._lock)
        self._actuator_inflight = None
        self._poll_lock = threading.Lock()
        self._close_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._closed = self._disconnected = False
        self._closing = False
        self._boot = self._request_seq = None
        self._actuator_enabled = self._actuator_bootstrapped = self._actuator_retired = False
        self._actuator_session = None
        self._actuator_next_seq = 0
        self._actuator_last_sent_seq = None
        self._actuator_pending = self._actuator_ack = None
        self._actuator_fault = ""
        self._fault_detail = None
        self._channels = {name: _Channel(getattr(config, name)) for name in ("steering", "throttle")}
        if serial_factory is None:
            import serial

            serial_factory = serial.Serial
        kwargs = dict(port=port, baudrate=115200, timeout=0.005, write_timeout=RESPONSE_TIMEOUT_S)
        if sys.platform.startswith("linux"):
            kwargs["exclusive"] = True
        try:
            try:
                self._serial = serial_factory(**kwargs)
            except TypeError as exc:
                if "exclusive" not in kwargs or "exclusive" not in str(exc):
                    raise
                del kwargs["exclusive"]
                self._serial = serial_factory(**kwargs)
            if not getattr(self._serial, "is_open", True):
                self._serial.close()
                raise OSError("Serial port is not open")
        except Exception as exc:
            raise RuntimeError(f"Cannot open Pico serial port {port}: {exc}") from exc
        if autostart and not self.start():
            raise RuntimeError(f"Cannot start Pico USB worker: {self._fault_detail or 'receiver unavailable'}")

    def start(self):
        """Start the existing connection's worker once, without rearming faults.

        Deferred clients may synchronously poll neutral around camera startup
        before starting continuous USB transactions. Starting never reconnects,
        resets an output session, or clears its retired state.
        """
        with self._close_lock:
            with self._lock:
                if (self._closed or self._closing or self._disconnected
                        or self._stop.is_set() or self._actuator_retired
                        or self._encoder_preparation_failed):
                    return False
            if self._thread is not None:
                return True
            try:
                self._thread = threading.Thread(target=self._run, name="pico-receiver", daemon=True)
                self._thread.start()
            except Exception as exc:
                self._thread = None
                self._stop.set()
                self._disconnected = self._closed = True
                self._fault_detail = f"{type(exc).__name__}: {exc}"
                self._retire_actuator("serial_worker_start_failed")
                self._invalidate("serial_worker_start_failed")
                try:
                    self._serial.close()
                except Exception:
                    pass
                return False
            return True

    def _invalidate(self, reason):
        with self._lock:
            for channel in self._channels.values():
                channel.invalidate(reason)

    def enable_actuator_output(self):
        """Opt in to AX1 on this connection; the first command is always neutral.

        A failed output session is retired until this receiver is recreated.
        Repeated enable calls cannot clear an output fault or rearm driving.
        """
        with self._lock:
            if self._closed or self._closing or self._disconnected or self._encoder_preparation_failed:
                return False
            if not self._actuator_enabled:
                self._actuator_enabled = True
                self._actuator_session = secrets.token_hex(8)
                self._actuator_pending = (*ACTUATOR_NEUTRAL, self.now())
            return not self._actuator_retired

    def set_output_pulses(self, steering_us, throttle_us):
        """Publish one atomic, bounded latest pair; never perform serial I/O."""
        for value in (steering_us, throttle_us):
            if type(value) is not int or not MIN_PULSE_US <= value <= MAX_PULSE_US:
                raise ValueError("Actuator pulse widths must be integers in 750..2250 us")
        with self._lock:
            if (not self._actuator_enabled or self._closed or self._closing
                    or self._disconnected or self._actuator_retired):
                return False
            at = self.now()
            if not math.isfinite(at):
                raise ValueError("Actuator producer clock must be finite")
            self._actuator_pending = (steering_us, throttle_us, at)
            return True

    def _retire_actuator(self, reason, exc=None, *, preserve_fault=False):
        with self._lock:
            keep_fault = preserve_fault and self._actuator_retired
            if exc is not None and not keep_fault:
                self._fault_detail = f"{type(exc).__name__}: {exc}"
            if self._actuator_enabled:
                self._actuator_retired = True
                if not keep_fault:
                    self._actuator_fault = reason
                self._actuator_pending = (*ACTUATOR_NEUTRAL, self.now())
            self._actuator_changed.notify_all()

    def _actuator_status_locked(self, now):
        ack = self._actuator_ack
        age = None if ack is None else now - ack["acknowledged_at"]
        fresh = age is not None and math.isfinite(age) and 0 <= age <= ACTUATOR_LEASE_S
        pending = self._actuator_pending
        return {"enabled": self._actuator_enabled,
                "ready": bool(self._actuator_enabled and self._actuator_bootstrapped
                              and not self._actuator_retired and not self._closed
                              and not self._closing and not self._disconnected and fresh),
                "retired": self._actuator_retired, "fault": self._actuator_fault,
                "fault_detail": self._fault_detail,
                "session": self._actuator_session, "last_sent_seq": self._actuator_last_sent_seq,
                "ack_age_s": age, "applied": None if ack is None else dict(ack),
                "pending": (None if pending is None else
                            {"steering_us": pending[0], "throttle_us": pending[1],
                             "producer_age_s": now - pending[2]}),
                "encoder_delivery_cursor": (None if self.encoder_source is None
                                             else self._encoder_transport_cursor),
                "encoder_delivery_total": (None if self.encoder_source is None
                                            else self._encoder_transport_total),
                "encoder_transport_mode": self._encoder_transport_mode,
                "measured_physical": False}

    def _await_actuator_status_locked(self, *, decision_margin_s=0.):
        """Await only the already-running transaction's ACK, never old evidence.

        Condition.wait releases the receiver lock so the USB owner can commit.
        Both the original source-time and wall-time 100 ms deadline bound this
        wait. No request, queued command, timestamp or lease is renewed here.
        """
        inflight = self._actuator_inflight
        waiting = False
        while True:
            now = self.now()
            status = self._actuator_status_locked(now)
            age = status["ack_age_s"]
            applied = status["applied"]
            handoff_pending = (inflight is not None and self._actuator_inflight is inflight
                and applied is not None and applied["acknowledged_at"] == inflight[0])
            needs_fresh = handoff_pending or (age is not None and age > ACTUATOR_LEASE_S - decision_margin_s)
            if ((status["ready"] and not waiting and not needs_fresh)
                    or inflight is None or self._actuator_inflight is not inflight
                    or not self._actuator_enabled or not self._actuator_bootstrapped
                    or self._actuator_retired or self._closed or self._closing
                    or self._disconnected or self._stop.is_set()
                    or age is None or not math.isfinite(age) or age < 0
                    or (not waiting and not needs_fresh)
                    or inflight[2] == threading.get_ident()):
                return status
            elapsed = now - inflight[0]
            if not math.isfinite(elapsed) or not 0 <= elapsed < TRANSACTION_TIMEOUT_S:
                return {**status, "ready": False} if handoff_pending else status
            remaining = min(TRANSACTION_TIMEOUT_S - elapsed, inflight[1] - time.monotonic())
            if remaining <= 0:
                return {**status, "ready": False} if handoff_pending else status
            waiting = True
            self._actuator_changed.wait(timeout=remaining)

    def actuator_status(self, *, wait_for_fresh=False):
        if type(wait_for_fresh) is not bool:
            raise ValueError("wait_for_fresh must be a boolean")
        with self._lock:
            return (self._await_actuator_status_locked() if wait_for_fresh else
                    self._actuator_status_locked(self.now()))

    def _validate_reply(self, reply, nonce, *, encoder_expected=None):
        """Check framing and channel values without publishing any freshness."""
        keys = ("v", "nonce", "boot", "seq", "steering", "throttle")
        encoder_enabled = (self._encoder_transport_mode == "rc2"
                           if encoder_expected is None else encoder_expected)
        _object(reply, keys + (("encoder",) if encoder_enabled else ()), "reply")
        version = 2 if encoder_enabled else 1
        _integer(reply["v"], version, version, "protocol version")
        if reply["nonce"] != nonce:
            raise _ProtocolError("Reply nonce mismatch")
        boot = reply["boot"]
        if not isinstance(boot, str) or re.fullmatch(r"[0-9a-f]{16}", boot) is None:
            raise _ProtocolError("Invalid boot identifier")
        request_seq = _integer(reply["seq"], 0, SEQUENCE_MASK, "request sequence")
        for name in self._channels:
            sample = _object(reply[name], ("seq", "pulse_us", "period_us", "age_us", "valid_run"), name)
            _integer(sample["seq"], 0, SEQUENCE_MASK, "source sequence")
            run = _integer(sample["valid_run"], 0, SEQUENCE_MASK, "source valid run")
            if run == 0:
                for field in ("pulse_us", "period_us", "age_us"):
                    if sample[field] is not None:
                        _integer(sample[field], -SEQUENCE_MODULUS, SEQUENCE_MASK, field)
            else:
                _integer(sample["pulse_us"], MIN_PULSE_US, MAX_PULSE_US, "pulse width")
                _integer(sample["period_us"], MIN_PERIOD_US, MAX_PERIOD_US, "pulse period")
                _integer(sample["age_us"], 0, 150000, "source age")
        return boot, request_seq

    def _accept(self, reply, nonce, requested_at):
        boot, request_seq = self._validate_reply(reply, nonce)
        with self._lock:
            if boot != self._boot:
                if self._boot is not None and self._actuator_enabled:
                    self._actuator_retired = True
                    self._actuator_fault = "pico_rebooted"
                    self._actuator_pending = (*ACTUATOR_NEUTRAL, self.now())
                    self._actuator_session = secrets.token_hex(8)
                    self._actuator_next_seq = 0
                    self._actuator_bootstrapped = False
                    self._actuator_ack = None
                self._channels = {name: _Channel(getattr(self.config, name)) for name in self._channels}
                self._boot, self._request_seq = boot, None
            if self._request_seq is not None:
                delta = (request_seq - self._request_seq) & SEQUENCE_MASK
                if not 0 < delta < SEQUENCE_MODULUS // 2:
                    raise _ProtocolError("Request sequence did not advance")
            for name, channel in self._channels.items():
                channel.update(reply[name], requested_at)
            self._request_seq = request_seq
            self._actuator_changed.notify_all()

    def _check_reply_deadline(self, requested_at, transaction_started_at, deadline, *, arrived=False):
        now = self.now()
        if transaction_started_at is not None:
            total_age = now - transaction_started_at
            if not math.isfinite(total_age) or not 0 <= total_age <= TRANSACTION_TIMEOUT_S:
                raise TimeoutError("Pico transaction exceeded 100 ms")
        age = now - requested_at
        if (not math.isfinite(age) or not 0 <= age <= RESPONSE_TIMEOUT_S
                or (deadline is not None and time.monotonic() >= deadline)):
            raise TimeoutError("Pico reply arrived too late" if arrived else "Pico reply timeout")

    def _read_reply(self, nonce, requested_at, *, allow_stopped=False, deadline=None,
                    transaction_started_at=None):
        def check_deadline(*, arrived=False):
            self._check_reply_deadline(requested_at, transaction_started_at, deadline, arrived=arrived)

        return self._read_json_reply(nonce, check_deadline, allow_stopped=allow_stopped)

    def _read_encoder_preparation_reply(self, nonce, requested_at, *, deadline=None):
        # This longer budget is unreachable from the normal RC2/AX1 reader.
        # No channel, encoder or acknowledgement is accepted during this leg.
        with self._lock:
            if (not self._encoder_preparing or self._encoder_prepared
                    or self._encoder_preparation_failed or self._actuator_bootstrapped
                    or self._boot is not None or self._closing or self._closed
                    or self.encoder_source is None or self.encoder_source.cursor != 0):
                raise _ProtocolError("Passive encoder preparation is not permitted")
        wall_deadline = time.monotonic() + ENCODER_PREPARATION_TIMEOUT_S
        if deadline is not None:
            wall_deadline = min(wall_deadline, deadline)

        def check_deadline(*, arrived=False):
            self._check_encoder_preparation_deadline(requested_at, wall_deadline)

        return self._read_json_reply(nonce, check_deadline)

    def _check_encoder_preparation_deadline(self, requested_at, wall_deadline):
        age = self.now() - requested_at
        if (not math.isfinite(age) or not 0 <= age <= ENCODER_PREPARATION_TIMEOUT_S
                or time.monotonic() >= wall_deadline):
            raise TimeoutError("Pico passive encoder preparation exceeded 200 ms or its caller deadline")

    def _read_json_reply(self, nonce, check_deadline, *, allow_stopped=False):
        """One common strict JSON parser; callers supply distinct, bounded clocks."""
        buffered = bytearray()
        skipped = 0
        while allow_stopped or not self._stop.is_set():
            check_deadline()
            if not getattr(self._serial, "is_open", True):
                raise OSError("Pico serial disconnected")
            available = getattr(self._serial, "in_waiting", 0)
            if type(available) is not int or available < 0:
                raise OSError("Invalid serial input count")
            # pyserial read(n) waits for n bytes or its timeout. Drain only
            # available data so a short ACK/tail incurs no extra 5 ms wait;
            # an empty queue still makes one bounded blocking read, not a spin.
            read_size = min(256, available) if available else 1
            chunk = self._serial.read(read_size)
            if not isinstance(chunk, bytes) or len(chunk) > read_size:
                raise OSError("Invalid serial read result")
            buffered.extend(chunk)
            while b"\n" in buffered:
                line, _, remainder = buffered.partition(b"\n")
                buffered = bytearray(remainder)
                limit = MAX_ENCODER_LINE_BYTES if self._encoder_transport_mode == "rc2" else MAX_LINE_BYTES
                if len(line) > limit:
                    raise _ProtocolError("Pico reply exceeds line limit")
                try:
                    reply = json.loads(line.decode("ascii"), object_pairs_hook=_unique_object)
                except (ValueError, UnicodeError):
                    reply = None
                if isinstance(reply, dict) and reply.get("nonce") == nonce:
                    check_deadline(arrived=True)
                    return reply
                skipped += 1
                if skipped > MAX_SKIPPED_LINES:
                    raise _ProtocolError("Too many unmatched Pico replies")
            limit = MAX_ENCODER_LINE_BYTES if self._encoder_transport_mode == "rc2" else MAX_LINE_BYTES
            if len(buffered) > limit:
                raise _ProtocolError("Pico reply exceeds line limit")
        return None

    def _write_request(self, request, deadline=None):
        if deadline is not None and time.monotonic() >= deadline:
            raise TimeoutError("Pico request deadline expired")
        if self._serial.write(request) != len(request):
            raise OSError("Incomplete serial write")

    def _actuator_transaction(self, nonce, requested_at, received_at, *, force_neutral=False,
                              allow_stopped=False, deadline=None, challenge_identity=None):
        with self._lock:
            if not self._actuator_enabled:
                return False
            now = self.now()
            pending = self._actuator_pending
            age = None if pending is None else now - pending[2]
            queue_limit = (ACTUATOR_PRODUCER_QUEUE_S if pending is not None
                           and pending[:2] != ACTUATOR_NEUTRAL else ACTUATOR_LEASE_S)
            stale = age is None or not math.isfinite(age) or not 0 <= age <= queue_limit
            if stale:
                # Retire the stale drive pair permanently, even if the clock
                # later catches up or repeated input polls keep succeeding.
                self._actuator_pending = pending = (*ACTUATOR_NEUTRAL, now)
            pair = (ACTUATOR_NEUTRAL if force_neutral or stale or self._actuator_retired
                    or self._encoder_manual_transition_neutral
                    or not self._actuator_bootstrapped else pending[:2])
            producer_at = None if pair == ACTUATOR_NEUTRAL else pending[2]
            session, boot, rc_seq = self._actuator_session, self._boot, self._request_seq
            if challenge_identity is not None:
                if not force_neutral or not self._actuator_retired:
                    raise _ProtocolError("Cleanup challenge requires retired neutral output")
                boot, rc_seq = challenge_identity
            seq = self._actuator_next_seq
            self._actuator_next_seq = (seq + 1) & SEQUENCE_MASK
            self._actuator_last_sent_seq = seq
        # The Pico issues its challenge after emitting the RC reply, separately
        # from the earlier source sample. Bound our send from that reply's
        # receipt; do not charge RC2 page transfer against the new challenge.
        # RC reply and AX1 reply each retain 50 ms, total retains 100 ms, and
        # ACK/source freshness still uses the ORIGINAL RC request timestamp.
        actuator_requested_at = self.now()
        receipt_age = received_at - requested_at
        if (not math.isfinite(receipt_age) or not 0 <= receipt_age <= RESPONSE_TIMEOUT_S
                or not math.isfinite(received_at) or actuator_requested_at < received_at):
            raise TimeoutError("Invalid Pico RC reply receipt time")
        challenge_age = actuator_requested_at - received_at
        if not math.isfinite(challenge_age) or not 0 <= challenge_age <= RESPONSE_TIMEOUT_S:
            raise TimeoutError("Pico actuator challenge expired before send")
        if producer_at is not None:
            producer_age = actuator_requested_at - producer_at
            if not math.isfinite(producer_age) or not 0 <= producer_age <= ACTUATOR_PRODUCER_QUEUE_S:
                pair, producer_at = ACTUATOR_NEUTRAL, None
                with self._lock:
                    if self._actuator_pending is pending:
                        self._actuator_pending = (*ACTUATOR_NEUTRAL, actuator_requested_at)
        request = f"AX1 {nonce} {session} {seq} {rc_seq} {pair[0]} {pair[1]}\n".encode("ascii")
        self._write_request(request, deadline)
        written_at = self.now()
        if producer_at is not None:
            producer_age = written_at - producer_at
            if not math.isfinite(producer_age) or not 0 <= producer_age <= ACTUATOR_LEASE_S:
                raise TimeoutError("Pico actuator producer expired during send")
        challenge_age = written_at - received_at
        if not math.isfinite(challenge_age) or not 0 <= challenge_age <= RESPONSE_TIMEOUT_S:
            raise TimeoutError("Pico actuator challenge expired during send")
        reply = self._read_reply(nonce, actuator_requested_at, allow_stopped=allow_stopped,
                                 deadline=deadline, transaction_started_at=requested_at)
        if reply is None:
            raise TimeoutError("Pico actuator acknowledgement missing")
        _object(reply, ("v", "nonce", "boot", "actuator"), "actuator reply")
        _integer(reply["v"], 1, 1, "actuator protocol version")
        if reply["nonce"] != nonce or reply["boot"] != boot:
            raise _ProtocolError("Actuator acknowledgement boot/nonce mismatch")
        output = _object(reply["actuator"],
                         ("session", "seq", "accepted", "steering_us", "throttle_us", "fault"),
                         "actuator acknowledgement")
        _integer(output["seq"], 0, SEQUENCE_MASK, "actuator sequence")
        if (output["session"] != session or output["seq"] != seq
                or type(output["accepted"]) is not bool
                or not isinstance(output["fault"], str) or len(output["fault"]) > 128):
            raise _ProtocolError("Actuator acknowledgement identity/type mismatch")
        for key in ("steering_us", "throttle_us"):
            _integer(output[key], MIN_PULSE_US, MAX_PULSE_US, key)
        if (not output["accepted"] or output["fault"]
                or (output["steering_us"], output["throttle_us"]) != pair):
            raise _ProtocolError("Actuator command rejected or applied pair mismatch")
        with self._lock:
            self._check_reply_deadline(actuator_requested_at, requested_at, deadline, arrived=True)
            if producer_at is not None:
                producer_age = self.now() - producer_at
                if not math.isfinite(producer_age) or not 0 <= producer_age <= ACTUATOR_LEASE_S:
                    raise TimeoutError("Pico actuator producer expired before acknowledgement")
            self._actuator_bootstrapped = True
            self._actuator_ack = {"session": session, "seq": seq, "boot": boot,
                                  "steering_us": pair[0], "throttle_us": pair[1],
                                  "acknowledged_at": requested_at,
                                  "measured_physical": False}
            if self._encoder_manual_transition_neutral:
                # Discard even pairs published while the neutral handoff was
                # in flight. Only a producer write after this ACK may drive.
                self._actuator_pending = (*ACTUATOR_NEUTRAL, self.now())
            self._encoder_manual_transition_neutral = False
        return True

    def _transaction(self, *, force_neutral=False, allow_stopped=False, deadline=None,
                     neutral_cleanup=False):
        if self._encoder_preparation_failed:
            return False
        if not self._encoder_prepared:
            self._prepare_encoder(deadline=deadline)
        nonce = secrets.token_hex(16)
        requested_at = self.now()
        wall_deadline = time.monotonic() + TRANSACTION_TIMEOUT_S
        if deadline is not None:
            wall_deadline = min(wall_deadline, deadline)
        with self._lock:
            inflight = (requested_at, wall_deadline, threading.get_ident())
            self._actuator_inflight = inflight
        try:
            return self._exchange(nonce, requested_at, force_neutral=force_neutral,
                                  allow_stopped=allow_stopped, deadline=deadline,
                                  neutral_cleanup=neutral_cleanup)
        finally:
            with self._lock:
                if self._actuator_inflight is inflight:
                    self._actuator_inflight = None
                self._actuator_changed.notify_all()

    def _exchange(self, nonce, requested_at, *, force_neutral=False, allow_stopped=False,
                  deadline=None, neutral_cleanup=False):
        if neutral_cleanup and (not force_neutral or not self._actuator_retired):
            raise _ProtocolError("RC1 cleanup requires retired neutral output")
        source = (self.encoder_source if self._encoder_transport_mode == "rc2"
                  and not neutral_cleanup else None)
        cursor = self._encoder_transport_cursor
        request = (f"RC1 {nonce}\n" if source is None
                   else f"RC2 {nonce} {cursor} {self._encoder_session}\n")
        self._write_request(request.encode("ascii"), deadline)
        reply = self._read_reply(nonce, requested_at, allow_stopped=allow_stopped, deadline=deadline)
        if reply is None:
            return False
        received_at = self.now()
        boot, request_seq = self._validate_reply(reply, nonce,
            **({"encoder_expected": False} if neutral_cleanup else {}))
        if neutral_cleanup:
            expected_boot, sequence = self._boot, self._request_seq
            if expected_boot is None and self._encoder_prepared_identity is not None:
                expected_boot, sequence = self._encoder_prepared_identity[:2]
            if (expected_boot is not None and boot != expected_boot
                    or sequence is not None
                    and not 0 < ((request_seq - sequence) & SEQUENCE_MASK) < SEQUENCE_MODULUS // 2):
                raise _ProtocolError("RC1 cleanup boot/sequence mismatch")
            # A shutdown proof must not republish RC channels or encoder state.
            self._actuator_transaction(nonce, requested_at, received_at, force_neutral=True,
                allow_stopped=allow_stopped, deadline=deadline,
                challenge_identity=(boot, request_seq))
            return True
        decoded = None
        if source is not None:
            try:
                decoded = source.decode_page(reply["encoder"], cursor)
            except ValueError as exc:
                raise _ProtocolError("Invalid Pico encoder page") from exc
            if self._boot is None:
                boot, sequence, epoch = self._encoder_prepared_identity
                delta = (_integer(reply.get("seq"), 0, SEQUENCE_MASK, "request sequence")
                         - sequence) & SEQUENCE_MASK
                if (reply.get("boot") != boot or decoded.epoch != epoch
                        or not 0 < delta < SEQUENCE_MODULUS // 2):
                    self._encoder_preparation_failed = True
                    raise _ProtocolError("Pico identity changed after passive encoder preparation")
        self._accept(reply, nonce, requested_at)
        actuator_accepted = self._actuator_transaction(nonce, requested_at, received_at, force_neutral=force_neutral,
                                                      allow_stopped=allow_stopped, deadline=deadline)
        # History/clock bookkeeping follows the time-critical AX1 leg. Source
        # correlation uses RC2 receipt, never the later actuator ACK arrival.
        if source is not None:
            source.feed(decoded, requested_at=requested_at, received_at=received_at,
                        boot=reply["boot"])
            self._acknowledge_encoder_page(decoded, reply["boot"])
            self._select_faulted_measurement_transport(source, actuator_accepted)
        return True

    def _select_faulted_measurement_transport(self, source, actuator_accepted):
        """After one completed RC2/AX1, stop requesting unusable encoder pages.

        Only permanent measurement faults qualify. Serial/protocol errors and
        passive preparation cannot enter this mode. The source remains latched
        and AUTO-ineligible; this does not rearm the RC controller or output.
        Reading the just-fed source's scalar latch avoids copying its history.
        """
        fault = source._fault
        with self._lock:
            if (not actuator_accepted or not fault.startswith("ENCODER_")
                    or self._encoder_transport_mode != "rc2" or self._actuator_retired
                    or not self._actuator_bootstrapped or self._closed or self._closing):
                return
            self._encoder_transport_mode = "rc1_faulted_measurement"
            # Do not carry a pre-fault AUTO producer pair across this handoff.
            # First complete a fresh neutral RC1/AX1, retaining the same session.
            self._encoder_manual_transition_neutral = True
            self._actuator_pending = (*ACTUATOR_NEUTRAL, self.now())

    def _acknowledge_encoder_page(self, page, boot):
        """Discard only an actually delivered, contiguous page after AX1.

        This cursor is transport accounting, not a count or freshness source.
        It can progress while PicoEncoderSource remains permanently faulted.
        Never jump to total (which may include undelivered pages), across lost
        history, or into a different boot/epoch. No retry/reset is performed.
        """
        with self._lock:
            cursor = self._encoder_transport_cursor
            identity = (boot, page.epoch)
            if (page.fault or page.requested_cursor != cursor or page.first != cursor + 1
                    or page.oldest > cursor + 1 or page.total < self._encoder_transport_total
                    or (self._encoder_transport_identity is not None
                        and identity != self._encoder_transport_identity)):
                return
            self._encoder_transport_cursor = cursor + page.count
            self._encoder_transport_total = page.total
            self._encoder_transport_identity = identity

    def _prepare_encoder(self, *, deadline=None):
        """Allocate a new reader's encoder once, without accepting its old sample.

        The first regular RC2 then replays cursor zero with a new nonce under
        the original 50 ms budget. Only that fresh reply can authorize AX1 or
        publish channel/source timestamps. Failure never retries preparation.
        """
        try:
            with self._lock:
                if (self._encoder_prepared or self._encoder_preparation_failed
                        or self._encoder_preparing or self._actuator_bootstrapped
                        or self._boot is not None or self._closing or self._closed
                        or self.encoder_source is None or self.encoder_source.cursor != 0):
                    raise _ProtocolError("Passive encoder preparation is not permitted")
                self._encoder_preparing = True
            nonce = secrets.token_hex(16)
            requested_at = self.now()
            preparation_deadline = time.monotonic() + ENCODER_PREPARATION_TIMEOUT_S
            if deadline is not None:
                preparation_deadline = min(preparation_deadline, deadline)
            self._write_request(f"RC2 {nonce} 0 {self._encoder_session}\n".encode("ascii"), preparation_deadline)
            reply = self._read_encoder_preparation_reply(nonce, requested_at, deadline=preparation_deadline)
            if reply is None:
                raise TimeoutError("Pico passive encoder preparation acknowledgement missing")
            boot, sequence = self._validate_reply(reply, nonce)
            try:
                page = self.encoder_source.decode_page(reply["encoder"], 0)
            except ValueError as exc:
                raise _ProtocolError("Invalid Pico encoder preparation page") from exc
            if page.fault or page.more or page.first != 1 or page.oldest > 1:
                raise _ProtocolError("Pico encoder preparation unavailable: "
                                     + (page.fault or "history incomplete"))
            self._check_encoder_preparation_deadline(requested_at, preparation_deadline)
            with self._lock:
                self._encoder_prepared_identity = (boot, sequence, page.epoch)
                self._encoder_prepared = True
        except BaseException:
            self._encoder_preparation_failed = True
            raise
        finally:
            self._encoder_preparing = False

    def poll_once(self):
        if (self._closed or self._disconnected or self._stop.is_set()
                or self._encoder_preparation_failed):
            return False
        with self._poll_lock:
            try:
                return self._transaction()
            except TimeoutError as exc:
                if self.encoder_source is not None:
                    self.encoder_source.invalidate("serial_timeout")
                self._retire_actuator("serial_timeout", exc)
                self._invalidate("serial_timeout")
            except _ProtocolError as exc:
                if self.encoder_source is not None:
                    self.encoder_source.invalidate("invalid_pico_reply")
                self._retire_actuator("invalid_pico_reply", exc)
                self._invalidate("invalid_pico_reply")
            except Exception as exc:
                if self.encoder_source is not None:
                    self.encoder_source.invalidate("serial_disconnected")
                self._disconnected = True
                self._retire_actuator("serial_disconnected", exc)
                self._invalidate("serial_disconnected")
        return False

    def _run(self):
        while not self._stop.is_set() and not self._disconnected and not self._actuator_retired:
            started = self.now()
            self.poll_once()
            self._stop.wait(max(0.0, POLL_S - (self.now() - started)))

    def snapshot(self, now=None):
        with self._lock:
            prepared_status = None
            if now is None:
                # Reserve one normal control tick BEFORE arbitration. The
                # output gate itself never waits after startup/proposal checks.
                prepared_status = self._await_actuator_status_locked(decision_margin_s=POLL_S)
                now = self.now()
            channels = [self._channels[name].snapshot(now) for name in ("steering", "throttle")]
            if self._actuator_enabled and (not self._actuator_status_locked(now)["ready"]
                    or prepared_status is not None and not prepared_status["ready"]):
                channels = [replace(channel, valid=False, fresh=False, neutral=False,
                                    error=self._actuator_fault or "actuator_not_ready")
                            for channel in channels]
            return RCSnapshot(*channels, now)

    def flush_actuator_neutral(self, timeout=.15):
        """One bounded fresh RC challenge + neutral AX1 on the existing port.

        The worker and this caller share poll_lock; neither can interleave frames.
        A retired session uses RC1 only, without accepting channel/source data.
        A healthy session retains its configured RC1/RC2 path. Neither can clear
        a fault, reconnect, reset the encoder reader or rearm driving.
        """
        if not math.isfinite(timeout) or not 0 < timeout <= .5:
            raise ValueError("Neutral flush timeout must be finite and in (0, .5]")
        deadline = time.monotonic() + timeout
        with self._lock:
            if not self._actuator_enabled:
                return True
            if self._closed or self._disconnected:
                return False
            self._actuator_pending = (*ACTUATOR_NEUTRAL, self.now())
        if not self._poll_lock.acquire(timeout=max(0, deadline - time.monotonic())):
            self._retire_actuator("neutral_flush_busy", preserve_fault=True)
            self._invalidate("neutral_flush_busy")
            return False
        try:
            return self._transaction(force_neutral=True, allow_stopped=True, deadline=deadline,
                                     neutral_cleanup=self._actuator_retired)
        except TimeoutError as exc:
            self._retire_actuator("neutral_flush_timeout", exc, preserve_fault=True)
            self._invalidate("neutral_flush_timeout")
        except _ProtocolError as exc:
            self._retire_actuator("neutral_flush_invalid_reply", exc, preserve_fault=True)
            self._invalidate("neutral_flush_invalid_reply")
        except Exception as exc:
            self._disconnected = True
            self._retire_actuator("serial_disconnected", exc, preserve_fault=True)
            self._invalidate("serial_disconnected")
        finally:
            self._poll_lock.release()
        return False

    def close(self):
        with self._close_lock:
            if self._closed:
                return
            with self._lock:
                self._closing = True
                self._stop.set()
                self._actuator_changed.notify_all()
            if self._thread is not None and self._thread is not threading.current_thread():
                self._thread.join(timeout=0.2)
            try:
                self.flush_actuator_neutral()
            finally:
                with self._lock:
                    self._closed = True
                    self._actuator_changed.notify_all()
                self._serial.close()
                self._invalidate("closed")
