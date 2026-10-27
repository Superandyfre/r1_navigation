"""Thread-safe supervisor health, independent of ROS, logging and worker IPC.

This is observation only: RUNNING never authorizes chassis motion. The first
fatal record is immutable until this entire runtime instance is discarded.
"""
from __future__ import annotations

from dataclasses import dataclass
import threading
import time
import uuid

HEALTH_SCHEMA = 'r1_neupan_worker_health_v1'


@dataclass(frozen=True)
class FatalFault:
    fault_id: str
    code: str
    worker_generation: int
    path_version: int
    goal_version: int
    failed_request_id: int
    occurrence_monotonic: float
    navigation_attempt_id: str | None


class WorkerHealth:
    def __init__(self) -> None:
        self.boot_id = str(uuid.uuid4())
        self._lock = threading.Lock()
        self._fault: FatalFault | None = None
        self._sequence = 0
        self._state = 'STARTING'
        self._generation = -1
        self._path_version = -1
        self._goal_version = -1
        self._last_success = -1
        self._attempt = None

    @property
    def fault(self) -> FatalFault | None:
        with self._lock:
            return self._fault

    def update_context(self, generation: int, path_version: int, goal_version: int) -> None:
        with self._lock:
            if self._fault is None:
                self._generation = int(generation)
                self._path_version = int(path_version)
                self._goal_version = int(goal_version)

    def bind_execution(self, attempt: str | None, generation: int, path: int, goal: int) -> None:
        with self._lock:
            if self._fault is None:
                self._attempt = attempt
                self._generation = generation
                self._path_version = path
                self._goal_version = goal

    def ready(self, generation: int) -> None:
        with self._lock:
            if self._fault is None and self._state != 'SHUTTING_DOWN':
                self._generation = int(generation)
                self._state = 'RUNNING'

    def success(self, request_id: int) -> None:
        with self._lock:
            if self._fault is None and self._state != 'SHUTTING_DOWN':
                self._last_success = int(request_id)

    def latch(self, code: str, generation: int, request_id: int = -1) -> FatalFault:
        with self._lock:
            if self._fault is None:
                self._fault = FatalFault(
                    str(uuid.uuid4()), str(code), int(generation),
                    self._path_version, self._goal_version, int(request_id), time.monotonic(),
                    self._attempt,
                )
                self._state = 'FAULT_LATCHED'
            return self._fault

    def shutting_down(self) -> None:
        with self._lock:
            if self._fault is None:
                self._state = 'SHUTTING_DOWN'

    def snapshot(self) -> dict:
        # No external locks, callbacks, IO or ROS calls while holding this lock.
        with self._lock:
            self._sequence += 1
            fault = self._fault
            return {
                'schema': HEALTH_SCHEMA,
                'boot_id': self.boot_id,
                'sequence': self._sequence,
                'state': self._state,
                'navigation_attempt_id': fault.navigation_attempt_id if fault else self._attempt,
                'worker_generation': fault.worker_generation if fault else self._generation,
                'path_version': fault.path_version if fault else self._path_version,
                'goal_version': fault.goal_version if fault else self._goal_version,
                'last_successful_request_id': self._last_success,
                'fault_id': fault.fault_id if fault else None,
                'fault_code': fault.code if fault else None,
                'failed_request_id': fault.failed_request_id if fault else None,
                'occurrence_monotonic': fault.occurrence_monotonic if fault else None,
                'monotonic': time.monotonic(),
            }
