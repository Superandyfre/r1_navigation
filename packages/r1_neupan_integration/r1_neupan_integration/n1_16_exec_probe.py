"""N2.4C6 Gate A executor probe for the N1.16 planner I/O node.

Benchmark-only diagnostics, activated exclusively by R1_N116_EXEC_PROBE=1.
It observes the executor topology without altering planner math: heartbeat
GIL-probe thread, GC pause recorder, scan/run callback duration wrappers,
state_lock wait measurement, and thread-frame snapshots on stalls.  Every
recorded quantity is observational; no planner input, state, output, or
timing behaviour is modified.
"""

from __future__ import annotations

import gc
import json
import os
import sys
import threading
import time
import traceback
from pathlib import Path


HEARTBEAT_PERIOD_SEC = 0.005
RECORD_GAP_SEC = 0.05
SNAPSHOT_GAP_SEC = 0.10
RECORD_CALLBACK_SEC = 0.02
RECORD_LOCK_WAIT_SEC = 0.02


def _page_faults() -> tuple[int, int]:
    """(minflt, majflt) delta source from /proc/self/stat."""
    try:
        fields = Path("/proc/self/stat").read_bytes().split()
        return int(fields[9]), int(fields[11])
    except (OSError, ValueError, IndexError):
        return -1, -1


class _ProfiledLock:
    """Wraps a lock, recording acquire waits above RECORD_LOCK_WAIT_SEC."""

    def __init__(self, inner: threading.Lock, probe: "ExecProbe") -> None:
        self._inner = inner
        self._probe = probe

    def acquire(self, blocking: bool = True, timeout: float = -1.0) -> bool:
        started = time.monotonic()
        acquired = self._inner.acquire(blocking, timeout)
        waited = time.monotonic() - started
        if acquired and waited >= RECORD_LOCK_WAIT_SEC:
            self._probe.record_lock_wait(waited)
        return acquired

    def release(self) -> None:
        self._inner.release()

    def __enter__(self):
        self.acquire()
        return self

    def __exit__(self, *exc):
        self.release()
        return False


class ExecProbe:
    def __init__(self, output_file: str) -> None:
        self._path = Path(output_file).expanduser().resolve()
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._events: list[dict] = []
        self._stop = threading.Event()
        self._gc_started = 0.0
        self._gc_started_stats: list[dict] | None = None
        self._gc_durations: list[float] = []
        self._lock_waits: list[float] = []
        self._callback_durations: dict[str, list[float]] = {}
        self._start_monotonic = time.monotonic()

        self._gc_thread_id = threading.get_ident()
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop, name="r1-exec-probe-heartbeat", daemon=True
        )
        self._heartbeat_thread.start()
        gc.callbacks.append(self._gc_callback)

    # -- relative timestamp helper -------------------------------------------------
    def _t(self) -> float:
        return time.monotonic() - self._start_monotonic

    def _append(self, event: dict) -> None:
        event["t_rel"] = self._t()
        with self._lock:
            self._events.append(event)
            path = self._path
        # JSONL line append; benchmark-only I/O on this process's own file.
        with open(path, "a", encoding="utf-8") as stream:
            stream.write(json.dumps(event, sort_keys=True) + "\n")

    # -- gc hook -------------------------------------------------------------------
    def _gc_callback(self, phase: str, info: dict) -> None:
        if phase == "start":
            self._gc_started = time.perf_counter_ns()
            return
        duration_ms = (time.perf_counter_ns() - self._gc_started) / 1.0e6
        self._gc_durations.append(duration_ms)
        if duration_ms >= RECORD_CALLBACK_SEC * 1000.0:
            self._append({
                "kind": "gc_pause",
                "generation": info.get("generation"),
                "duration_ms": duration_ms,
                "collected": info.get("collected"),
                "uncollectable": info.get("uncollectable"),
            })

    # -- heartbeat / GIL probe ------------------------------------------------------
    def _heartbeat_loop(self) -> None:
        last = time.monotonic()
        last_faults = _page_faults()
        while not self._stop.is_set():
            time.sleep(HEARTBEAT_PERIOD_SEC)
            now = time.monotonic()
            gap = now - last
            faults = _page_faults()
            fault_delta = (
                faults[0] - last_faults[0] if faults[0] >= 0 and last_faults[0] >= 0 else -1
            )
            majflt_delta = (
                faults[1] - last_faults[1] if faults[1] >= 0 and last_faults[1] >= 0 else -1
            )
            last, last_faults = now, faults
            if gap >= SNAPSHOT_GAP_SEC:
                self._append({
                    "kind": "heartbeat_gap",
                    "gap_sec": gap,
                    "threads": self._thread_frames(),
                    "gc_count": gc.get_count(),
                    "gc_stats": [
                        {"collections": gen["collections"]}
                        for gen in gc.get_stats()
                    ],
                    "minflt_delta": fault_delta,
                    "majflt_delta": majflt_delta,
                })
            elif gap >= RECORD_GAP_SEC:
                self._append({
                    "kind": "heartbeat_gap",
                    "gap_sec": gap,
                    "minflt_delta": fault_delta,
                    "majflt_delta": majflt_delta,
                })

    def _thread_frames(self) -> list[dict]:
        frames = sys._current_frames()
        ident_to_name = {th.ident: th.name for th in threading.enumerate()}
        snapshot = []
        for ident, frame in frames.items():
            if ident == threading.get_ident():
                continue
            try:
                stack = traceback.extract_stack(frame)[-6:]
            except Exception:
                stack = []
            snapshot.append({
                "thread": ident_to_name.get(ident, str(ident)),
                "frames": [
                    f"{item.filename.split('/')[-1]}:{item.lineno}:{item.name}"
                    for item in stack
                ],
            })
        return snapshot

    # -- lock / callback instrumentation --------------------------------------------
    def record_lock_wait(self, waited_sec: float) -> None:
        self._lock_waits.append(waited_sec)
        if waited_sec >= SNAPSHOT_GAP_SEC:
            self._append({
                "kind": "state_lock_wait",
                "waited_sec": waited_sec,
                "threads": self._thread_frames(),
            })

    def wrap_callback(self, name: str, function):
        probe = self

        def wrapper(*args, **kwargs):
            started = time.monotonic()
            try:
                return function(*args, **kwargs)
            finally:
                duration = time.monotonic() - started
                with probe._lock:
                    probe._callback_durations.setdefault(name, []).append(duration)
                if duration >= RECORD_CALLBACK_SEC:
                    probe._append({
                        "kind": "callback_duration",
                        "name": name,
                        "duration_sec": duration,
                        "threads": (
                            probe._thread_frames() if duration >= SNAPSHOT_GAP_SEC else None
                        ),
                    })

        return wrapper

    # -- finalize --------------------------------------------------------------------
    def finalize(self) -> None:
        if self._stop.is_set():
            return
        self._stop.set()
        try:
            gc.callbacks.remove(self._gc_callback)
        except ValueError:
            pass
        self._heartbeat_thread.join(timeout=1.0)
        stats = {
            "kind": "summary",
            "wall_sec": self._t(),
            "heartbeat_gaps_recorded": sum(
                1 for event in self._events if event["kind"] == "heartbeat_gap"
            ),
            "gc_collections": len(self._gc_durations),
            "gc_total_ms": sum(self._gc_durations),
            "gc_max_ms": max(self._gc_durations) if self._gc_durations else 0.0,
            "lock_waits_recorded": len(self._lock_waits),
            "lock_wait_max_sec": max(self._lock_waits) if self._lock_waits else 0.0,
            "callback_stats": {
                name: {
                    "count": len(values),
                    "max_sec": max(values),
                    "mean_sec": sum(values) / len(values),
                }
                for name, values in self._callback_durations.items()
            },
        }
        self._append(stats)

    def install_node_hooks(self, node) -> None:
        """Wrap the shared state lock on a live node (observation only).

        Callback wrapping happens at class level before instantiation (see
        patch_class) because subscriptions and timers bind the callable at
        creation time.
        """
        inner = getattr(node, "state_lock", None)
        if isinstance(inner, type(threading.Lock())):
            node.state_lock = _ProfiledLock(inner, self)


def install(node_class) -> ExecProbe:
    """Patch the node class and return the probe.

    Entry point used by n1_16_node.main() when R1_N116_EXEC_PROBE=1.  Must be
    called BEFORE the node is instantiated so wrapped callbacks are the ones
    subscriptions and timers bind to.
    """
    output = os.environ.get(
        "R1_N116_EXEC_PROBE_FILE", "n2_4c6_results/exec_probe/n1_16_exec_probe.jsonl"
    )
    probe = ExecProbe(output)
    if hasattr(node_class, "scan_callback"):
        node_class.scan_callback = probe.wrap_callback(
            "scan_callback", node_class.scan_callback
        )
    if hasattr(node_class, "run"):
        node_class.run = probe.wrap_callback("run_timer", node_class.run)
    import atexit

    atexit.register(probe.finalize)
    return probe
