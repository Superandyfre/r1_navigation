#!/usr/bin/python3
"""Run a launch command in its own session and stop it when the parent exits."""

import ctypes
import errno
import os
import signal
import subprocess
import sys
import time


PR_SET_PDEATHSIG = 1
GRACE_SECONDS = 5.0


def set_parent_death_signal() -> None:
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(PR_SET_PDEATHSIG, signal.SIGTERM) != 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))
    # The parent may have exited between process creation and prctl().
    if os.getppid() == 1:
        os.kill(os.getpid(), signal.SIGTERM)


def reset_child_signal_handlers() -> None:
    for signum in (signal.SIGHUP, signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, signal.SIG_DFL)


def main() -> int:
    if len(sys.argv) < 2:
        raise SystemExit("usage: launch_with_parent_death.py COMMAND [ARG ...]")

    parent_pid = os.getppid()
    set_parent_death_signal()
    os.setsid()
    child = subprocess.Popen(
        sys.argv[1:], close_fds=True, preexec_fn=reset_child_signal_handlers)
    stopping = False

    def terminate(signum: int, _frame: object) -> None:
        nonlocal stopping
        if stopping:
            return
        stopping = True
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)

        try:
            os.killpg(os.getpgrp(), signal.SIGINT)
        except ProcessLookupError:
            pass

        deadline = time.monotonic() + GRACE_SECONDS
        while child.poll() is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if child.poll() is None:
            try:
                os.killpg(os.getpgrp(), signal.SIGTERM)
            except ProcessLookupError:
                pass
            deadline = time.monotonic() + GRACE_SECONDS
            while child.poll() is None and time.monotonic() < deadline:
                time.sleep(0.05)
        if child.poll() is None:
            os.killpg(os.getpgrp(), signal.SIGKILL)
        raise SystemExit(128 + signum)

    signal.signal(signal.SIGHUP, terminate)
    signal.signal(signal.SIGINT, terminate)
    signal.signal(signal.SIGTERM, terminate)
    while child.poll() is None:
        # PR_SET_PDEATHSIG is the fast path. Polling parent identity is the
        # fallback for launch environments where that signal is not delivered.
        if os.getppid() != parent_pid:
            terminate(signal.SIGTERM, None)
        time.sleep(0.05)
    return child.returncode


if __name__ == "__main__":
    raise SystemExit(main())
