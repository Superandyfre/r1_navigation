"""Stop ROS actions before shutting their context down. No ROS import here."""
import signal
import time

from .protocol import SCHEMA, write_json


class _SignalFlag:
    def __init__(self):
        self.value = False

    def set(self):
        self.value = True

    def is_set(self):
        return self.value


class StopSignals:
    """Signal handlers only set a flag; the main loop performs ROS operations."""
    def __init__(self):
        # Avoid locks inside Python signal handlers, including repeated Ctrl-C.
        self.requested = _SignalFlag()
        self.signum = None
        self.previous = {}

    def request(self, signum=None, frame=None):
        if self.signum is None:
            self.signum = signum
        self.requested.set()

    def __enter__(self):
        for number in (signal.SIGINT, signal.SIGTERM):
            self.previous[number] = signal.getsignal(number)
            signal.signal(number, self.request)
        return self

    def __exit__(self, *unused):
        for number, handler in self.previous.items():
            signal.signal(number, handler)


def finish_bridge(node, executor, ros, signum=None, timeout=1., clock=time.monotonic):
    """Best-effort bounded drain. Cancel transport != terminal result != stop.

    If an external component has already destroyed the ROS context, record the
    lost capability and still attempt all remaining cleanup steps. Exceptions
    in one cleanup action must not prevent node/context cleanup.
    """
    errors = []

    def attempt(name, function):
        try:
            return function()
        except Exception as error:
            errors.append({'step': name, 'error': str(error)})
            return None

    def context_ok():
        return attempt('context_ok', ros.ok) is True

    context_before = context_ok()
    report = dict(schema=SCHEMA, signal=signum, at=clock(),
                  context_available_before_cancel=context_before,
                  cancel_requested=False, cancel_dispatched=False,
                  action_terminal_received=False, measured_stopped=False,
                  cleanup_errors=errors)
    if node is not None:
        node.stopping = True  # spin_once during drain must not send another goal
        report['bridge_session'] = node.session
        before = node.lifecycle.result()
        report['action_before'] = None if before is None else before['state']
        attempt('request_cancel', lambda: node.lifecycle.cancel('bridge_shutdown'))
        deadline = clock() + timeout
        while executor is not None and node.lifecycle.busy() and context_ok() and clock() < deadline:
            if attempt('drain_callback', lambda: executor.spin_once(timeout_sec=.05)) is False:
                break
            if errors and errors[-1]['step'] == 'drain_callback':
                break
        current = node.lifecycle.current
        after = node.lifecycle.result()
        report['action_after'] = None if after is None else after['state']
        report['cancel_requested'] = bool(current and current['cancel_requested'])
        report['cancel_dispatched'] = bool(current and current['cancel_sent'])
        report['action_terminal_received'] = bool(after and after['state'] in
            ('succeeded', 'cancelled', 'aborted'))
        # Do not infer physical stop from action status or node destruction.
        motion = getattr(node, 'motion_payload', {})
        now = clock()
        v, w = motion.get('linear_speed_m_s'), motion.get('angular_speed_rad_s')
        report['measured_stopped'] = bool(
            motion.get('inputs_valid') is True and v is not None and w is not None
            and 0 <= now-motion.get('received_at', 0.) <= node.a.input_max_age
            and abs(v) < .02 and abs(w) < .03)
        attempt('shutdown_record', lambda: write_json(node.root/'live/shutdown.json', report))
    if executor is not None:
        if node is not None:
            attempt('remove_node', lambda: executor.remove_node(node))
        attempt('executor_shutdown', lambda: executor.shutdown(timeout_sec=.5))
    if node is not None:
        attempt('destroy_node', node.destroy_node)
    if context_ok():
        attempt('context_shutdown', ros.shutdown)
    if node is not None:
        # Include later destroy/shutdown errors in the final local record.
        attempt('final_shutdown_record', lambda: write_json(node.root/'live/shutdown.json', report))
    return report
