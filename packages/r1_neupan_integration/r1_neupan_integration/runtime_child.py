"""Process-group leader: stop all owned children if the supervisor disappears."""
import ctypes
import os
import signal
import subprocess
import sys
import time


def main():
    if os.getpgrp()!=os.getpid():raise RuntimeError("owned_session_leader_required")
    parent=os.getppid()
    libc=ctypes.CDLL(None,use_errno=True)
    if libc.prctl(1,signal.SIGTERM)!=0:raise OSError(ctypes.get_errno(),'prctl_failed')
    def stop(signum,frame):
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,signal.SIG_IGN)
        os.killpg(os.getpgrp(),signal.SIGTERM)
        time.sleep(.3)
        os.killpg(os.getpgrp(),signal.SIGKILL)
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,stop)
    if os.getppid()!=parent or parent==1:stop(signal.SIGTERM,None)
    child=subprocess.Popen(sys.argv[1:],close_fds=True)
    while child.poll() is None:
        if os.getppid()!=parent:stop(signal.SIGTERM,None)
        time.sleep(.05)
    code=child.returncode
    # Even an unexpected direct launch exit cannot leave its grandchildren.
    stop(signal.SIGTERM,None)
    return code


if __name__=='__main__':raise SystemExit(main())
