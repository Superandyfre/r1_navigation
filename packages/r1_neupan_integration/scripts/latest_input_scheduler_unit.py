#!/usr/bin/env python3
"""Deterministic unit checks for the bounded NeuPAN input scheduler."""

from __future__ import annotations

import threading
import time

from r1_neupan_integration.latest_input_scheduler import LatestInputScheduler


def wait_until(predicate, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return predicate()


def main() -> None:
    started = threading.Event()
    release = threading.Event()
    completed = []
    errors = []

    def callback(_started_at):
        completed.append(time.monotonic())
        if len(completed) == 1:
            started.set()
            release.wait(1.0)

    scheduler = LatestInputScheduler(callback, errors.append)
    scheduler.start()
    scheduler.notify()
    assert started.wait(1.0), "first callback did not start"
    for _ in range(20):
        scheduler.notify()
    assert scheduler.snapshot()["pending"] == 1
    release.set()
    assert wait_until(lambda: len(completed) == 2), "missing coalesced follow-up"
    time.sleep(0.05)
    assert len(completed) == 2, "more than one stale follow-up was scheduled"
    snapshot = scheduler.snapshot()
    assert snapshot["cycles"] == 2
    assert snapshot["pending"] == 0
    assert not errors
    assert scheduler.stop(1.0)
    print("latest_input_scheduler_unit.py PASS")


if __name__ == "__main__":
    main()
