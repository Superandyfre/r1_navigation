"""Bounded recovery transaction. Backends own process and ROS operations.

No retries, no old-goal replay, no fault clearing in existing processes.
"""
import threading
import time


class RecoveryTransaction:
    def __init__(self, backend, timeout=45.):
        self.backend = backend
        self.timeout = timeout
        self.lock = threading.Lock()
        self.cancelled = threading.Event()
        self.phase = 'idle'

    def run(self):
        if not self.lock.acquire(blocking=False):
            return False, 'recovery_already_running'
        self.cancelled.clear()
        deadline=time.monotonic()+self.timeout
        try:
            def step(name, operation):
                self.phase=name
                if self.cancelled.is_set() or time.monotonic() >= deadline:
                    raise RuntimeError('recovery_cancelled_or_expired')
                operation(deadline, self.cancelled)
            step('capture_owned_runtime',self.backend.capture)
            step('revoke',self.backend.revoke)
            step('verify_stopped',self.backend.verify_stopped)
            step('save_evidence',self.backend.save_evidence)
            step('stop_old',self.backend.stop_old)
            step('start_new',self.backend.start_new)
            step('verify_new_idle',self.backend.verify_new_idle)
            step('enable_idle_runtime',self.backend.enable_idle)
            self.phase='waiting_new_goal'
            return True,self.phase
        except Exception as error:
            # Failure cleanup cannot turn a failed transaction into success.
            self.phase='failed:'+self.phase
            try:self.backend.abort()
            except Exception:pass
            return False,self.phase+':'+str(error)
        finally:
            self.lock.release()


class OwnedProcessGroup:
    """Own a fresh runtime group from birth; never discover/kill by process name."""
    def __init__(self, command, log, environment=None):
        self.command=list(command)
        self.log=log
        self.environment=environment
        self.process=None
        self.group=None

    def start(self):
        import subprocess
        if self.members():raise RuntimeError('old_runtime_still_alive')
        import sys
        command=[sys.executable, "-m", "r1_neupan_integration.runtime_child"]+self.command
        self.process=subprocess.Popen(command,env=self.environment,start_new_session=True,
                                      stdout=self.log,stderr=subprocess.STDOUT)
        self.group=self.process.pid

    def members(self):
        from pathlib import Path
        if self.group is None:return []
        found=[]
        for path in Path('/proc').glob('[0-9]*/stat'):
            try:
                raw=path.read_text();fields=raw[raw.rfind(')')+2:].split()
                if int(fields[2])==self.group and fields[0] not in ('Z','X'):
                    found.append(int(path.parent.name))
            except (OSError,ValueError):continue
        return found

    def stop(self,deadline,cancel):
        import os
        import signal
        # Cancellation cannot leave an already revoked group half alive.
        for sig,grace in ((signal.SIGINT,2.),(signal.SIGTERM,1.),(signal.SIGKILL,.5)):
            if not self.members():break
            os.killpg(self.group,sig)
            end=min(deadline,time.monotonic()+grace)
            while self.members() and time.monotonic()<end:time.sleep(.02)
        if self.process is not None:self.process.poll()
        if self.members():raise RuntimeError('old_processes_did_not_exit')
