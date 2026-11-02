"""Numerical failure evidence and bounded retry state; no motion authorization."""
from __future__ import annotations

import math
import time

RECOVERY_PROTOCOL = 'r1_solver_recovery_v1'


class RecoverableSolveError(RuntimeError):
    def __init__(self, detail, request_id, exhausted=False):
        super().__init__(detail)
        self.request_id = request_id
        self.exhausted = exhausted


class EcosOutcome:
    """Observe this process' last ECOS call, never change arguments or results."""
    def __init__(self):
        self.info = None

    def install(self):
        import ecos
        original = ecos.solve
        def solve(*args, **kwargs):
            self.info = None
            result = original(*args, **kwargs)
            self.info = dict(result.get('info', {}))
            return result
        ecos.solve = solve

    def begin(self):
        self.info = None

    def numerical_failure(self, error):
        from diffcp.cone_program import SolverError
        # ECOS -1 specifically means iteration limit, not corrupted data,
        # infeasibility, arbitrary SolverError, or an unrelated Python exception.
        return (isinstance(error, SolverError) and str(error) == 'Solver ecos errored.'
                and self.info is not None and self.info.get('exitFlag') == -1
                and all(math.isfinite(float(self.info.get(k, float('nan'))))
                        for k in ('pres', 'dres', 'gap')))


class SolveBudget:
    """One cold retry per execution, bounded even with intermittent success.

    Paths do not identify executions. Owner supplies an immutable attempt key.
    BT bounds FollowPath replacement separately at the parent task boundary.
    """
    def __init__(self, retries=1, recovery_seconds=1.0):
        self.limit = retries
        self.seconds = recovery_seconds
        self.key = None
        self.used = 0
        self.started = None
        self.exhausted = False
        self.spent = 0.0

    def bind(self, key):
        if key != self.key:
            self.key, self.used, self.started, self.exhausted = key, 0, None, False
            self.spent = 0.0

    def failure(self, now=None):
        now = time.monotonic() if now is None else now
        if self.started is None:
            self.started = now
        self.used += 1
        self.exhausted = self.used > self.limit or self.spent + now - self.started >= self.seconds
        return self.exhausted

    def expired(self, now=None):
        now = time.monotonic() if now is None else now
        if self.started is not None and self.spent + now - self.started >= self.seconds:
            self.exhausted = True
        return self.exhausted

    def success(self):
        # Successful solves stop the recovery clock, never replenish its budget.
        if self.started is not None:
            self.spent += time.monotonic() - self.started
            self.started = None
