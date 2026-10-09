"""Single-flight scheduler that coalesces planner input updates."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable


class LatestInputScheduler:
    """Run a callback once for the newest available input notification."""

    def __init__(self, callback: Callable[[float], None],
                 on_error: Callable[[BaseException], None]) -> None:
        self._callback = callback
        self._on_error = on_error
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._notifications = 0
        self._cycles = 0
        self._last_notify_monotonic = -1.0
        self._last_start_monotonic = -1.0
        self._last_finish_monotonic = -1.0

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("LatestInputScheduler already started")
        self._thread = threading.Thread(
            target=self._loop, name="r1-neupan-latest-input", daemon=True
        )
        self._thread.start()

    def notify(self) -> None:
        """Coalesce a newly available scan or controller-context update."""
        with self._lock:
            self._notifications += 1
            self._last_notify_monotonic = time.monotonic()
        self._wake.set()

    def snapshot(self) -> dict[str, float | int]:
        with self._lock:
            return {
                "notifications": self._notifications,
                "cycles": self._cycles,
                "last_notify_monotonic": self._last_notify_monotonic,
                "last_start_monotonic": self._last_start_monotonic,
                "last_finish_monotonic": self._last_finish_monotonic,
                "pending": int(self._wake.is_set()),
            }

    def stop(self, timeout_sec: float) -> bool:
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout=max(0.0, float(timeout_sec)))
        return self._thread is None or not self._thread.is_alive()

    def _loop(self) -> None:
        while True:
            self._wake.wait()
            # Clear before running. A notification received during callback
            # execution stays set and schedules one fresh follow-up cycle.
            self._wake.clear()
            if self._stop.is_set():
                return
            started = time.monotonic()
            with self._lock:
                self._cycles += 1
                self._last_start_monotonic = started
            try:
                self._callback(started)
            except BaseException as exc:
                self._on_error(exc)
            finally:
                with self._lock:
                    self._last_finish_monotonic = time.monotonic()
