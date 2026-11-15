"""One worker for visual processing that must not block the control loop.

The caller copies frames and other mutable state before ``submit`` and transfers
ownership of the payload. Neither this worker nor the callback copies payloads.
At most one callback runs at a time, with one latest payload waiting behind it.
Replacing that waiting payload avoids a growing queue of obsolete images.
"""
import threading


_EMPTY = object()


class LatestWorker:
    """Run ``callback(payload)`` serially, discarding older waiting payloads.

    ``submit`` never waits for a callback. ``processed`` counts completed
    callbacks, including callbacks that raised; ``dropped`` counts accepted
    payloads replaced before processing. ``error`` keeps the latest exception
    description even when later callbacks succeed. ``pending`` describes the
    waiting slot, rather than a callback already running.

    ``close`` stops accepting new payloads and flushes the latest waiting one.
    A false result means a callback is still running: resources used by the
    callback must remain alive until a later ``close`` returns true. The daemon
    thread does not hold the process open if the caller has to exit.
    """

    def __init__(self, callback, name="latest-visuals"):
        if not callable(callback):
            raise TypeError("callback must be callable")
        self._callback = callback
        self._condition = threading.Condition()
        self._pending = _EMPTY
        self._closing = False
        self._submitted = 0
        self._processed = 0
        self._dropped = 0
        self._error = None
        self._thread = threading.Thread(target=self._run, name=name, daemon=True)
        self._thread.start()

    def submit(self, payload):
        """Transfer an owned payload to the worker, or return false if closed."""
        with self._condition:
            if self._closing:
                return False
            if self._pending is not _EMPTY:
                self._dropped += 1
            self._pending = payload
            self._submitted += 1
            self._condition.notify()
            return True

    def status(self):
        """Return a thread-safe snapshot without waiting for visual work."""
        with self._condition:
            return {
                "submitted": self._submitted,
                "processed": self._processed,
                "dropped": self._dropped,
                "error": self._error,
                "pending": self._pending is not _EMPTY,
            }

    def close(self, timeout=5):
        """Flush latest pending work; return true only after the worker exits."""
        with self._condition:
            self._closing = True
            self._condition.notify()
        # A callback can request shutdown, but cannot join its own thread.
        if threading.current_thread() is self._thread:
            return False
        self._thread.join(timeout)
        return not self._thread.is_alive()

    def _run(self):
        while True:
            with self._condition:
                self._condition.wait_for(
                    lambda: self._closing or self._pending is not _EMPTY
                )
                if self._pending is _EMPTY:
                    return
                payload = self._pending
                self._pending = _EMPTY
            error = None
            try:
                self._callback(payload)
            except Exception as exc:
                try:
                    error = "{}: {}".format(type(exc).__name__, exc)[:500]
                except Exception:
                    error = type(exc).__name__
            finally:
                # Large images should be released on this worker, outside the
                # small state lock used by the control-loop-facing methods.
                del payload
            with self._condition:
                self._processed += 1
                if error is not None:
                    self._error = error
