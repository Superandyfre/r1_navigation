"""One latest-only background transformer for auxiliary arm obstacle sensors."""
from __future__ import annotations

import threading
import time
import numpy as np
from .obstacle_cloud import decode_xyz_timestamps, project_cloud_to_world, project_scan_to_world


class LatestArmObstacleScenes:
    """Read-only sensor processing. Never publishes or commands motion."""
    def __init__(self, lookup, bin_ns=5_000_000, max_span_ns=200_000_000):
        self.lookup=lookup
        self.bin_ns=int(bin_ns)
        self.max_span_ns=int(max_span_ns)
        self._lock=threading.Lock()
        self._wake=threading.Event()
        self._stopped=False
        self._pending={}
        self._results={}
        self._errors={}
        self._sequence=0
        self._thread=threading.Thread(target=self._run,name="arm-obstacle-scenes",daemon=True)
        self._thread.start()

    def submit(self, kind, message, received_monotonic=None):
        if kind not in ("front","rear"):
            raise ValueError("unknown obstacle sensor")
        received=time.monotonic() if received_monotonic is None else float(received_monotonic)
        with self._lock:
            if self._stopped:
                return False
            self._sequence+=1
            self._pending[kind]=(self._sequence,message,received)
        self._wake.set()
        return True

    def snapshot(self):
        with self._lock:
            return {kind:{**item,"points":item["points"].copy()}
                    for kind,item in self._results.items()}

    def close(self):
        with self._lock:self._stopped=True;self._pending.clear()
        self._wake.set()
        self._thread.join(timeout=1.)
        return not self._thread.is_alive()

    def _run(self):
        while True:
            self._wake.wait()
            self._wake.clear()
            with self._lock:
                if self._stopped:return
                pending=self._pending
                self._pending={}
            for kind,(sequence,message,received) in pending.items():
                try:
                    frame=str(message.header.frame_id)
                    if not frame:
                        raise ValueError("missing sensor frame")
                    lookup=lambda stamp:self.lookup(frame,stamp)
                    header=int(message.header.stamp.sec)*1_000_000_000+int(message.header.stamp.nanosec)
                    if kind=="front":
                        xyz,stamps,_=decode_xyz_timestamps(message)
                        world=project_cloud_to_world(xyz,stamps,header,self.bin_ns,self.max_span_ns,lookup)
                    else:
                        world=project_scan_to_world(message,self.bin_ns,self.max_span_ns,lookup)
                    item=dict(sequence=sequence,stamp_ns=header,received_monotonic=received,
                              processed_monotonic=time.monotonic(),points=world)
                    with self._lock:
                        old=self._results.get(kind)
                        if not self._stopped and (old is None or header>old["stamp_ns"]):
                            self._results[kind]=item
                except Exception as exc:
                    # Retain original timestamp of last successful observation;
                    # a later arrival cannot make it fresh again.
                    with self._lock:
                        self._errors[kind]=type(exc).__name__+":"+str(exc)
                    continue
