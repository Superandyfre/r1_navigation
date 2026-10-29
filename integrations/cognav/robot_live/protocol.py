"""ROS-free execution lifecycle. All timestamps are same-host monotonic seconds."""
import json
import math
import os
import threading
import time
from pathlib import Path

SCHEMA = 'cognav_execution_v2'
IDENTITY = ('bridge_session', 'driver_session', 'task_id', 'map_session',
            'generation', 'seq', 'version', 'uid')
LIVE = ('pending', 'executing', 'cancelling')


def read_json(path):
    try:
        return json.loads(Path(path).read_text('utf-8'))
    except (OSError, ValueError):
        return None


def write_json(path, payload):
    path = Path(path)
    tmp = path.with_name('.' + path.name + '.tmp')
    tmp.write_text(json.dumps(payload, allow_nan=False), encoding='utf-8')
    os.replace(str(tmp), str(path))


def identity(row):
    if not isinstance(row, dict) or row.get('schema') != SCHEMA:
        raise ValueError('Execution schema missing/mismatched')
    if any(not isinstance(row.get(k), str) or not row[k]
           for k in IDENTITY[:4]):
        raise ValueError('Execution session identity missing')
    if any(type(row.get(k)) is not int or row[k] < 1
           for k in ('generation', 'seq', 'version')):
        raise ValueError('Execution version missing')
    if not isinstance(row.get('uid'), str):
        raise ValueError('Invalid candidate UID')
    return tuple(row[k] for k in IDENTITY)


def lease_reason(plan, lease, bridge_session, now, max_age=.6):
    try:
        if plan['bridge_session'] != bridge_session:
            return 'bridge_session_changed'
        if identity(plan) != identity(lease):
            return 'lease_identity_mismatch'
        stamp = lease['at']
        # Explicit same-path renewal avoids cancel/resend on each RGB-D frame.
        expires = lease.get('valid_until', plan['expires_at'])
        if not all(type(v) in (int, float) and math.isfinite(v)
                   for v in (stamp, expires)):
            return 'invalid_deadline'
        if not -.1 <= now - stamp <= max_age:
            return 'driver_heartbeat_stale'
        if now >= expires:
            return 'plan_expired'
        if expires - now > 30:
            return 'unbounded_plan_lease'
        if lease.get('permitted') is not True:
            return 'driver_veto:' + str(lease.get('reason', 'unknown'))
        return ''
    except (KeyError, TypeError, ValueError):
        return 'invalid_execution_contract'


class GoalLifecycle:
    """One unresolved action at a time, including a pending acceptance/cancel.

    A cancel request is not a terminal result or measured physical stop. Until
    its result arrives, replacement goals wait. Late callbacks close over the
    original context, never a mutable global sequence number.
    """
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.RLock()
        self.current = None
        self.sequence = 0
        self.events = []

    def _emit(self, ctx, state, reason=''):
        self.sequence += 1
        ctx.update(state=state, reason=str(reason), at=self.clock(),
                   feedback_seq=self.sequence)
        self.events.append((self.sequence, ctx['identity'], state, str(reason)))
        self.events = self.events[-128:]

    def busy(self):
        with self.lock:
            return self.current is not None and self.current['state'] in LIVE

    def begin(self, plan):
        with self.lock:
            if self.busy():
                raise RuntimeError('Previous goal unresolved; cannot replace')
            ctx = dict(plan)
            ctx.update(identity=identity(plan), handle=None, cancel_requested=False,
                       cancel_sent=False, accepted=False, ready=False)
            self.current = ctx
            self._emit(ctx, 'pending')
            return ctx

    def fail_send(self, ctx, error):
        with self.lock:
            if ctx is self.current:
                self._emit(ctx, 'failed', str(error))

    def _cancel_handle(self, ctx):
        if ctx['handle'] is None or ctx['cancel_sent']:
            return
        try:
            ctx['handle'].cancel_goal_async()
            ctx['cancel_sent'] = True
        except Exception as error:
            # Keep the lifecycle unresolved; never claim physical cancellation.
            ctx['reason'] = 'cancel_transport_error:' + str(error)

    def cancel(self, reason):
        with self.lock:
            ctx = self.current
            if ctx is None or ctx['state'] not in LIVE:
                return
            if not ctx['cancel_requested']:
                ctx['cancel_requested'] = True
                ctx['ready'] = False
                self._emit(ctx, 'cancelling', reason)
            self._cancel_handle(ctx)

    def accepted(self, ctx, future):
        with self.lock:
            try:
                handle = future.result()
            except Exception as error:
                if ctx is self.current:
                    ctx['cancel_requested'] = True
                    self._emit(ctx, 'cancelling', 'acceptance_transport_error:' + str(error))
                return
            if not handle.accepted:
                if ctx is self.current:
                    self._emit(ctx, 'rejected', 'action_rejected')
                return
            ctx['handle'] = handle
            ctx['accepted'] = True
            goal_id = getattr(getattr(handle, 'goal_id', None), 'uuid', None)
            if goal_id is not None:
                ctx['navigation_attempt_id'] = bytes(goal_id).hex()
            if ctx is not self.current or ctx['cancel_requested']:
                self._cancel_handle(ctx)
            else:
                self._emit(ctx, 'executing', 'action_accepted')
            # Even cancelled/pending-old actions need their eventual result drained.
            handle.get_result_async().add_done_callback(
                lambda result, bound=ctx: self.finished(bound, result))

    def finished(self, ctx, future):
        with self.lock:
            if ctx is not self.current:
                return
            try:
                response = future.result()
                status = response.status
                result = getattr(response, 'result', None)
                details = {}
                if result is not None:
                    for name in ('error_code', 'error_msg'):
                        value = getattr(result, name, None)
                        if isinstance(value, (int, str)):
                            details[name] = value if isinstance(value, int) else value[:1024]
                ctx['action_result'] = details
                state = {4: 'succeeded', 5: 'cancelled', 6: 'aborted'}.get(status, 'failed')
                reason = ctx['reason'] if ctx['cancel_requested'] else 'action_status:%s' % status
                if not ctx['cancel_requested'] and details.get('error_msg'):
                    reason += ':' + details['error_msg']
                # A success received after revocation never grants a ready result.
                if ctx['cancel_requested']:
                    state = 'cancelled' if status in (4, 5) else state
                self._emit(ctx, state, reason)
                ctx['handle'] = None
            except Exception as error:
                # Unknown action state is unresolved; do not overlap a new goal.
                ctx['cancel_requested'] = True
                self._emit(ctx, 'cancelling', 'result_transport_error:' + str(error))
                self._cancel_handle(ctx)

    def action_feedback(self, ctx, message):
        with self.lock:
            if ctx is not self.current or ctx['cancel_requested']:
                return
            report = {}
            for name in ('distance_to_goal', 'speed'):
                value = getattr(message, name, None)
                if value is not None and math.isfinite(float(value)):
                    report[name] = float(value)
            ctx['action_feedback'] = report
            ctx['action_feedback_at'] = self.clock()

    def result(self):
        with self.lock:
            if self.current is None:
                return None
            ctx = self.current
            row = {k: ctx[k] for k in IDENTITY}
            row.update(schema=SCHEMA, state=ctx['state'], reason=ctx['reason'],
                       feedback_seq=ctx['feedback_seq'], at=self.clock(),
                       accepted=ctx['accepted'], cancel_requested=ctx['cancel_requested'],
                       kind=ctx['kind'], ready=False)
            row['navigation_attempt_id'] = ctx.get('navigation_attempt_id')
            row['action_feedback'] = dict(ctx.get('action_feedback', {}))
            row['action_feedback_at'] = ctx.get('action_feedback_at')
            row['action_result'] = dict(ctx.get('action_result', {}))
            return row


class SettledObservation:
    """Measured alignment + continuous low speed, scoped to one exact plan."""
    def __init__(self, dwell=.3):
        self.dwell = dwell
        self.key = None
        self.since = None

    def update(self, key, now, fresh, aligned, linear, angular):
        if key != self.key:
            self.key, self.since = key, None
        valid = (fresh and aligned and linear is not None and angular is not None
                 and math.isfinite(linear) and math.isfinite(angular)
                 and abs(linear) < .02 and abs(angular) < .03)
        if not valid:
            self.since = None
            return False, None
        if self.since is None:
            self.since = now
        ready = now - self.since >= self.dwell
        return ready, self.since + self.dwell if ready else None
