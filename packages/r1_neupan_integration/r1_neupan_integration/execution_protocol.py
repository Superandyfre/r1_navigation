"""Strict same-host execution context and source-v2 contracts (no ROS or IO).

The caller serializes Store operations and installation under its state lock.
A context authorizes computation only; the final Guard owns motion permission.
"""
from dataclasses import dataclass
import math

CONTEXT_SCHEMA = 'r1_neupan_execution_context_v1'
SOURCE_SCHEMA = 'r1_neupan_planner_source_v2'
MAX_INT = 2**63 - 1


class ContextError(ValueError):
    def __init__(self, code, *, fatal=False):
        super().__init__(code)
        self.fatal = fatal


def integer(value, minimum=0):
    if type(value) is not int or not minimum <= value <= MAX_INT:
        raise ContextError('invalid_integer')
    return value


def number(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ContextError('invalid_number')
    return float(value)


def text(value):
    if not isinstance(value, str) or not value:
        raise ContextError('invalid_string')
    return value


@dataclass(frozen=True)
class ExecutionContext:
    authority: str
    controller: str
    attempt: str
    path: int
    goal: int
    frame: str
    coordinates: tuple

    @property
    def identity(self):
        return self.authority, self.controller, self.attempt

    def fields(self):
        return dict(authority_boot_id=self.authority, controller_boot_id=self.controller,
                    navigation_attempt_id=self.attempt, path_version=self.path,
                    goal_version=self.goal)


class ExecutionContextStore:
    def __init__(self):
        self.active = None
        self.bound_boots = None
        self.retired = set()
        self.sequence = 0
        self.last_payload = None

    def accept(self, payload, now):
        """Return True only on an atomic install/end; duplicates never reset state."""
        if not isinstance(payload, dict) or payload.get('schema') != CONTEXT_SCHEMA:
            raise ContextError('invalid_context_schema')
        authority = text(payload.get('authority_boot_id'))
        controller = text(payload.get('controller_boot_id'))
        attempt = text(payload.get('navigation_attempt_id'))
        sequence = integer(payload.get('sequence'), 1)
        path = integer(payload.get('path_version'))
        goal = integer(payload.get('goal_version'))
        stamp = number(payload.get('monotonic'))
        if stamp <= 0 or stamp > now:
            raise ContextError('invalid_context_time')
        state = payload.get('state')
        if state not in ('ACTIVE', 'END'):
            raise ContextError('invalid_context_state')
        boots = authority, controller
        identity = authority, controller, attempt
        if self.bound_boots is not None and boots != self.bound_boots:
            raise ContextError('execution_authority_changed', fatal=True)
        # Safety terminal notifications apply even if delivered after a newer
        # active message. Never let replay rejection resurrect a retired UUID.
        if state == 'END':
            self.retired.add(identity)
            ended = self.active is not None and self.active.identity == identity
            if ended:
                self.active = None
            if self.bound_boots is not None and sequence > self.sequence:
                self.sequence, self.last_payload = sequence, dict(payload)
            return ended
        if identity in self.retired or now - stamp > 0.5:
            return False
        if sequence <= self.sequence:
            if sequence == self.sequence and payload != self.last_payload:
                raise ContextError('execution_sequence_conflict', fatal=self.active is not None)
            return False
        frame = text(payload.get('frame_id'))
        rows = payload.get('path')
        if not isinstance(rows, list) or not rows:
            raise ContextError('invalid_execution_path')
        coordinates = []
        for row in rows:
            if not isinstance(row, list) or len(row) != 3:
                raise ContextError('invalid_execution_pose')
            coordinates.append(tuple(number(v) for v in row))
        endpoint = payload.get('goal')
        if not isinstance(endpoint, list) or len(endpoint) != 3:
            raise ContextError('invalid_execution_goal')
        if tuple(number(v) for v in endpoint) != coordinates[-1]:
            raise ContextError('execution_goal_not_endpoint')
        candidate = ExecutionContext(authority, controller, attempt, path, goal, frame,
                                     tuple(coordinates))
        if self.active is not None:
            if candidate.identity == self.active.identity:
                if candidate.path < self.active.path or candidate.goal < self.active.goal:
                    return False
                if candidate.path == self.active.path and candidate.goal == self.active.goal:
                    if candidate != self.active:
                        raise ContextError('execution_identity_conflict', fatal=True)
                    self.sequence, self.last_payload = sequence, dict(payload)
                    return False
                if candidate.path <= self.active.path or candidate.goal <= self.active.goal:
                    raise ContextError('execution_revision_conflict', fatal=True)
            else:
                self.retired.add(self.active.identity)
        self.bound_boots = boots
        self.active = candidate
        self.sequence, self.last_payload = sequence, dict(payload)
        return True


def validate_source(payload, now):
    """Python side of the shared source-v2 parser contract."""
    if not isinstance(payload, dict) or payload.get('schema') != SOURCE_SCHEMA:
        raise ContextError('invalid_source_schema')
    for key in ('authority_boot_id', 'controller_boot_id', 'navigation_attempt_id', 'worker_boot_id'):
        text(payload.get(key))
    for key in ('worker_generation', 'request_id', 'cycle_id', 'scan_counter'):
        integer(payload.get(key), 1)
    for key in ('path_version', 'goal_version'):
        integer(payload.get(key))
    if payload.get('finite') is not True:
        raise ContextError('invalid_finite_flag')
    stamps = [number(payload.get(key)) for key in (
        'scan_receive_monotonic', 'request_dispatched_monotonic',
        'worker_solve_end_monotonic', 'planner_result_monotonic')]
    if not 0 < stamps[0] <= stamps[1] <= stamps[2] <= stamps[3] <= now:
        raise ContextError('source_timestamp_order', fatal=True)
    command = payload.get('command')
    if not isinstance(command, dict):
        raise ContextError('invalid_source_command')
    number(command.get('linear_x'))
    number(command.get('angular_z'))
    return payload


def make_source(context, *, worker_boot, generation, request, cycle, scan,
                scan_received, dispatched, solved, published, linear, angular):
    """Only captured input/transport timestamps; publication cannot launder age."""
    payload = context.fields()
    payload.update(schema=SOURCE_SCHEMA, worker_boot_id=text(worker_boot),
                   worker_generation=integer(generation, 1), request_id=integer(request, 1),
                   cycle_id=integer(cycle, 1), scan_counter=integer(scan, 1), finite=True,
                   scan_receive_monotonic=number(scan_received),
                   request_dispatched_monotonic=number(dispatched),
                   worker_solve_end_monotonic=number(solved),
                   planner_result_monotonic=number(published),
                   command=dict(linear_x=number(linear), angular_z=number(angular)))
    return validate_source(payload, published)
