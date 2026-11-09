"""Isolated latest-scene NeuPAN trajectory revalidator. It never publishes ROS.

The caller must separately check execution authority at publication time.
"""
from dataclasses import asdict
import copy
import multiprocessing as mp
import threading
import time
import numpy as np


def _result_is_relevant(message, epoch, latest_sequence, completed_sequence):
    """Never drop an observed obstacle just because a newer scan is queued."""
    sequence=message.get("sequence",0)
    if message.get("epoch")!=epoch or sequence<=completed_sequence:
        return False
    return (sequence==latest_sequence or
            (message.get("type")=="result" and message.get("result") is None and
             message.get("outcome")=="obstacle_conflict"))


def _run(conn, config_path, checkpoint_path):
    try:
        import torch
        torch.set_num_threads(1)
        torch.set_num_interop_threads(1)
        from .n1_15_whole_core import _build_planner
        from .trajectory_continuation import TrajectoryContinuation
        planner = _build_planner(config_path, checkpoint_path, {}, None)
        engine = TrajectoryContinuation(planner)
        installed = None
        conn.send({"type": "ready"})
        while True:
            job = conn.recv()
            if job is None:
                return
            try:
                plan = job["plan"]
                key = (job["epoch"], plan["request_id"])
                if key != installed:
                    engine.install(**plan)
                    installed = key
                scene = dict(job["scene"])
                scene["now"] = time.monotonic()
                result, outcome = engine.validate_detailed(**scene)
                finished = time.monotonic()
                conn.send({"type": "result", "epoch": job["epoch"], "sequence": job["sequence"],
                           "result": None if result is None else asdict(result), "outcome": outcome,
                           "scan_counter": scene["scan_counter"],
                           "scan_receive_monotonic": scene["scan_received_monotonic"],
                           "pose_receive_monotonic": scene["pose_received_monotonic"],
                           "validation_started_monotonic": scene["now"],
                           "validated_monotonic": finished})
            except Exception as exc:
                conn.send({"type": "error", "epoch": job["epoch"], "sequence": job["sequence"],
                           "error": type(exc).__name__ + ": " + str(exc)})
    except BaseException as exc:
        try: conn.send({"type": "fatal", "error": type(exc).__name__ + ": " + str(exc)})
        except Exception: pass
    finally:
        conn.close()


class ContinuationWorker:
    """One validator process and one replaceable pending scan, no cmd publisher."""
    def __init__(self, config_path, checkpoint_path):
        ctx=mp.get_context("spawn")
        self._parent, child=ctx.Pipe()
        self._process=ctx.Process(target=_run,args=(child,str(config_path),str(checkpoint_path)),daemon=True)
        self._process.start();child.close()
        self._lock=threading.Lock()
        self._epoch=0;self._sequence=0;self._completed_sequence=0;self._plan=None;self._pending=None;self._result=None;self._event=None
        self._closed=False;self._ready=False;self._failure=None
        self._thread=threading.Thread(target=self._io,daemon=True);self._thread.start()

    def install(self, *, context, request_id, scan_counter, started_monotonic, controls):
        plan=dict(context=tuple(context),request_id=request_id,scan_counter=scan_counter,
                  started_monotonic=started_monotonic,controls=np.asarray(controls,dtype=float).copy())
        with self._lock:
            if self._closed: raise RuntimeError("continuation_worker_closed")
            self._epoch+=1;self._plan=plan;self._pending=None;self._result=None;self._event=None
            return self._epoch

    def clear(self):
        with self._lock:
            self._epoch+=1;self._plan=None;self._pending=None;self._result=None;self._event=None

    def submit(self, *, context, scan_counter, scan_received_monotonic, pose_received_monotonic,
               input_timeout, state, points):
        with self._lock:
            if self._closed or self._failure or self._plan is None or tuple(context)!=self._plan["context"]:
                return False
            self._sequence+=1
            scene=dict(context=tuple(context),scan_counter=scan_counter,
                scan_received_monotonic=scan_received_monotonic,pose_received_monotonic=pose_received_monotonic,
                input_timeout=input_timeout,state=np.asarray(state,dtype=float).copy(),
                points=None if points is None else np.asarray(points,dtype=float).copy())
            self._pending=dict(epoch=self._epoch,sequence=self._sequence,plan=self._plan,scene=scene)
            # Keep an unconsumed obstacle denial until the consumer sees it;
            # submitting a newer scan must not erase that evidence.
            return True

    def snapshot(self):
        with self._lock:
            return dict(ready=self._ready, failure=self._failure, epoch=self._epoch,
                        sequence=self._sequence, completed_sequence=self._completed_sequence, result=copy.deepcopy(self._result),
                        event=copy.deepcopy(self._event))

    def _io(self):
        busy=False
        try:
            while not self._closed:
                if self._parent.poll(.01):
                    message=self._parent.recv()
                    with self._lock:
                        kind=message.get("type")
                        if kind=="ready":self._ready=True
                        elif kind=="fatal":self._failure=message.get("error","worker_failed")
                        else:
                            busy=False
                            if _result_is_relevant(message,self._epoch,self._sequence,self._completed_sequence):
                                if kind=="error":self._failure=message.get("error","validation_failed")
                                else:
                                    self._result=message.get("result");self._event=dict(message)
                                    self._completed_sequence=message["sequence"]
                if not self._process.is_alive():
                    with self._lock:self._failure=self._failure or "continuation_worker_exited"
                    return
                # One request in flight; only the newest pending scan is sent.
                with self._lock:
                    job=self._pending if self._ready and not busy else None
                    if job is not None:self._pending=None;busy=True
                if job is not None:self._parent.send(job)
        except (EOFError,OSError,BrokenPipeError) as exc:
            with self._lock:self._failure=type(exc).__name__

    def close(self):
        with self._lock:self._closed=True;self._epoch+=1;self._pending=None;self._result=None;self._event=None
        self._thread.join(timeout=.5)
        if self._thread.is_alive():
            self._process.terminate();self._thread.join(timeout=1.)
        elif self._process.is_alive():
            try:self._parent.send(None)
            except (OSError,BrokenPipeError):pass
        self._process.join(timeout=1.)
        if self._process.is_alive():self._process.terminate();self._process.join(timeout=1.)
        self._parent.close()
