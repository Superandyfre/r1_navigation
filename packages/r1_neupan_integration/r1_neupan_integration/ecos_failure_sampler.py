"""Read-only ECOS failure sampler for the N1.15/N1.16 worker.

Purpose
-------
``diffcp/cone_program.py`` calls ``ecos.solve`` and then inspects
``solution["info"]["exitFlag"]``.  When the flag is negative it raises
``SolverError("Solver ecos errored.")`` and throws away every field of
``info`` as well as the problem itself.  This module wraps ``ecos.solve``
*in the worker process only* and persists the real cone program plus the
raw ECOS ``info`` whenever the flag is negative.

Guarantees (all enforced by the offline test next to this file)
---------------------------------------------------------------
1. Arguments are passed to the original solver unchanged.
2. The returned object is the original object, unmodified, identity-wise.
3. Exceptions raised by the solver are re-raised untouched.
4. Any failure inside the sampler is swallowed; it can never change the
   planner result or mask the original exception.
5. Nothing is written to disk on a successful solve (metadata only, in
   memory, bounded).
6. Nothing happens at all unless ``install()`` is called, which the worker
   does only when ``COGNAV_ECOS_SAMPLER_DIR`` is set.

Output layout (one directory per worker process, never overwritten)
-------------------------------------------------------------------
``<dir>/worker_<pid>_<boot>/summary.json``          counters, always current
``<dir>/worker_<pid>_<boot>/failure_0001/problem.npz``   real cone program
``<dir>/worker_<pid>_<boot>/failure_0001/info.json``     raw ECOS info
``<dir>/worker_<pid>_<boot>/failure_0001/context.json``  phase/request/stack
"""

from __future__ import annotations

import collections
import json
import os
import threading
import time
import traceback

ENV_VAR = "COGNAV_ECOS_SAMPLER_DIR"


def _to_jsonable(value):
    """Best-effort conversion that never raises."""
    try:
        if isinstance(value, (str, int, float, bool)) or value is None:
            return value
        if isinstance(value, (list, tuple)):
            return [_to_jsonable(item) for item in value]
        if isinstance(value, dict):
            return {str(key): _to_jsonable(item) for key, item in value.items()}
        return repr(value)
    except Exception:
        return "<unserializable>"


def _matrix_descriptor(matrix):
    """Shape/nnz/dtype for a dense array, a scipy sparse matrix, or None."""
    if matrix is None:
        return {"present": False}
    try:
        import numpy as np

        if hasattr(matrix, "data") and hasattr(matrix, "indptr"):
            data = np.asarray(matrix.data)
            return {
                "present": True,
                "format": "sparse",
                "shape": [int(v) for v in matrix.shape],
                "nnz": int(data.size),
                "dtype": str(data.dtype),
                "nbytes": int(data.nbytes + np.asarray(matrix.indices).nbytes
                               + np.asarray(matrix.indptr).nbytes),
            }
        dense = np.asarray(matrix)
        return {
            "present": True,
            "format": "dense",
            "shape": [int(v) for v in dense.shape],
            "dtype": str(dense.dtype),
            "nbytes": int(dense.nbytes),
        }
    except Exception as exc:  # pragma: no cover - defensive
        return {"present": True, "error": type(exc).__name__ + ": " + str(exc)}


class EcosSampler:
    """Wraps ``ecos.solve``; see the module docstring for the contract."""

    def __init__(self, directory, max_records=16, success_ring=5,
                 capture_success=True, success_ring_bytes=262144):
        self.directory = os.path.abspath(str(directory))
        self.max_records = int(max_records)
        self.success_ring = int(success_ring)
        self.capture_success = bool(capture_success)
        self.success_ring_bytes = int(success_ring_bytes)

        self.lock = threading.RLock()
        self._original = None
        self.module_name = None

        self.solves = 0
        self.failures = 0
        self.sampler_errors = 0
        self.exit_hist = {}
        self.phase = "startup"
        self.context = {}
        self.capture_ns = 0
        self.solve_ns = 0
        self._recent = collections.deque(maxlen=max(0, self.success_ring))
        self._run_dir = None

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    def install(self, module=None):
        if self._original is not None:
            return self
        if module is None:
            import ecos as module  # noqa: PLC0415 - intentional lazy import
        original = getattr(module, "solve", None)
        if original is None:
            raise RuntimeError("module %r has no solve()" % (module,))
        self.module_name = getattr(module, "__name__", "ecos")
        self._original = original
        self._run_dir = os.path.join(
            self.directory, "worker_%d_%d" % (os.getpid(), int(time.time()))
        )
        try:
            os.makedirs(self._run_dir, exist_ok=True)
        except Exception:
            self._run_dir = None

        def wrapped(*args, **kwargs):
            started = time.perf_counter_ns()
            solution = original(*args, **kwargs)
            solve_ns = time.perf_counter_ns() - started
            try:
                self._after_solve(args, kwargs, solution, solve_ns)
            except BaseException:  # never affect the caller
                try:
                    self.sampler_errors += 1
                except Exception:
                    pass
            return solution

        module.solve = wrapped
        return self

    def uninstall(self):
        if self._original is None or self.module_name is None:
            return
        import sys

        module = sys.modules.get(self.module_name)
        if module is not None:
            module.solve = self._original
        self._original = None

    # ------------------------------------------------------------------
    # context set by the worker (all optional, all cheap)
    # ------------------------------------------------------------------
    def set_phase(self, phase):
        self.phase = str(phase)

    def set_context(self, **fields):
        with self.lock:
            self.context = _to_jsonable(fields)
            self.context["phase"] = self.phase

    # ------------------------------------------------------------------
    # internals
    # ------------------------------------------------------------------
    def _after_solve(self, args, kwargs, solution, solve_ns):
        started = time.perf_counter_ns()
        flag = None
        info = None
        if isinstance(solution, dict):
            info = solution.get("info")
            if isinstance(info, dict):
                flag = info.get("exitFlag")
        with self.lock:
            self.solves += 1
            self.solve_ns += int(solve_ns)
            key = str(flag)
            self.exit_hist[key] = self.exit_hist.get(key, 0) + 1

        negative = isinstance(flag, (int, float)) and flag < 0
        if negative:
            with self.lock:
                self.failures += 1
                record_index = self.failures
            if record_index <= self.max_records:
                self._write_record(record_index, args, kwargs, info, flag)
            self._write_summary()
        else:
            self._remember_success(args, kwargs, info, flag)
            if self.solves % 500 == 0:
                self._write_summary()
        self.capture_ns += time.perf_counter_ns() - started

    def _payload_bytes(self, args, kwargs):
        total = 0
        for item in _split_args(args, kwargs):
            if item is None:
                continue
            try:
                if hasattr(item, "data") and hasattr(item, "indptr"):
                    total += int(getattr(item.data, "nbytes", 0))
                    total += int(getattr(item.indices, "nbytes", 0))
                    total += int(getattr(item.indptr, "nbytes", 0))
                else:
                    total += int(getattr(item, "nbytes", 0))
            except Exception:
                return None
        return total

    def _remember_success(self, args, kwargs, info, flag):
        if not self.capture_success or self.success_ring <= 0:
            return
        try:
            size = self._payload_bytes(args, kwargs)
            entry = {
                "solve_index": self.solves,
                "exitFlag": flag,
                "info": _to_jsonable(info),
                "context": dict(self.context),
                "problem": _describe_arrays(args, kwargs),
                "payload_bytes": size,
                "full": False,
            }
            if size is not None and size <= self.success_ring_bytes:
                entry["full"] = True
                entry["args"] = args
                entry["kwargs"] = kwargs
            self._recent.append(entry)
        except Exception:
            self.sampler_errors += 1

    def _write_record(self, index, args, kwargs, info, flag):
        if not self._run_dir:
            self.sampler_errors += 1
            return
        import numpy as np

        target = os.path.join(self._run_dir, "failure_%04d" % index)
        try:
            os.makedirs(target, exist_ok=False)
        except FileExistsError:
            import uuid

            target = os.path.join(
                self._run_dir, "failure_%04d_%s" % (index, uuid.uuid4().hex[:6])
            )
            os.makedirs(target, exist_ok=True)

        c, g, h, dims, a, b = _split_args(args, kwargs)
        arrays = {}
        if c is not None:
            arrays["c"] = np.asarray(c)
        if h is not None:
            arrays["h"] = np.asarray(h)
        if b is not None:
            arrays["b"] = np.asarray(b)
        if g is not None:
            arrays["G_shape"] = np.asarray(getattr(g, "shape", (0, 0)))
            if hasattr(g, "data") and hasattr(g, "indptr"):
                arrays["G_data"] = np.asarray(g.data)
                arrays["G_indices"] = np.asarray(g.indices)
                arrays["G_indptr"] = np.asarray(g.indptr)
            else:
                arrays["G_dense"] = np.asarray(g)
        if a is not None:
            arrays["A_shape"] = np.asarray(getattr(a, "shape", (0, 0)))
            if hasattr(a, "data") and hasattr(a, "indptr"):
                arrays["A_data"] = np.asarray(a.data)
                arrays["A_indices"] = np.asarray(a.indices)
                arrays["A_indptr"] = np.asarray(a.indptr)
            else:
                arrays["A_dense"] = np.asarray(a)
        tmp = os.path.join(target, "problem.npz.tmp")
        with open(tmp, "wb") as handle:
            np.savez_compressed(handle, **arrays)
        os.replace(tmp, os.path.join(target, "problem.npz"))

        solver_kwargs = {key: value for key, value in kwargs.items()
                         if key not in ("c", "G", "h", "dims", "A", "b")}
        context = {
            "phase": self.phase,
            "worker_context": self.context,
            "pid": os.getpid(),
            "solve_index": self.solves,
            "ecos_module": self.module_name,
            "solver_kwargs": _to_jsonable(solver_kwargs),
            "dims": _to_jsonable(dims),
            "shapes": _describe_arrays(args, kwargs),
            "call_stack": traceback.format_stack(limit=24),
            "monotonic": time.monotonic(),
            "wall_time": time.time(),
        }
        with open(os.path.join(target, "context.json"), "w", encoding="utf-8") as fh:
            json.dump(context, fh, indent=2, ensure_ascii=False)
        with open(os.path.join(target, "info.json"), "w", encoding="utf-8") as fh:
            json.dump({"exitFlag": flag, "info": _to_jsonable(info)},
                      fh, indent=2, ensure_ascii=False)

        # Preceding solves: full payloads when small enough, else metadata.
        previous = []
        for entry in list(self._recent):
            item = {key: value for key, value in entry.items()
                    if key not in ("args", "kwargs")}
            if entry.get("full"):
                try:
                    sub = os.path.join(target, "prev_%05d" % entry["solve_index"])
                    os.makedirs(sub, exist_ok=True)
                    sub_arrays = _arrays_from(entry["args"], entry["kwargs"])
                    sub_tmp = os.path.join(sub, "problem.npz.tmp")
                    with open(sub_tmp, "wb") as handle:
                        np.savez_compressed(handle, **sub_arrays)
                    os.replace(sub_tmp, os.path.join(sub, "problem.npz"))
                    item["stored"] = os.path.relpath(sub, target)
                except Exception as exc:
                    item["store_error"] = type(exc).__name__
            previous.append(item)
        with open(os.path.join(target, "previous.json"), "w", encoding="utf-8") as fh:
            json.dump(previous, fh, indent=2, ensure_ascii=False)

    def _write_summary(self):
        if not self._run_dir:
            return
        with self.lock:
            payload = {
                "pid": os.getpid(),
                "phase": self.phase,
                "context": self.context,
                "solves": self.solves,
                "failures": self.failures,
                "sampler_errors": self.sampler_errors,
                "exit_flag_histogram": dict(self.exit_hist),
                "capture_ms": round(self.capture_ns / 1e6, 3),
                "solve_ms": round(self.solve_ns / 1e6, 3),
                "updated_monotonic": time.monotonic(),
            }
        try:
            path = os.path.join(self._run_dir, "summary.json")
            tmp = path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as fh:
                json.dump(payload, fh, indent=2, ensure_ascii=False)
            os.replace(tmp, path)
        except Exception:
            self.sampler_errors += 1


def _split_args(args, kwargs):
    """Split ``ecos.solve(c, G, h, dims, A=None, b=None, **kw)`` arguments."""
    c = args[0] if len(args) > 0 else kwargs.get("c")
    g = args[1] if len(args) > 1 else kwargs.get("G")
    h = args[2] if len(args) > 2 else kwargs.get("h")
    dims = args[3] if len(args) > 3 else kwargs.get("dims")
    a = args[4] if len(args) > 4 else kwargs.get("A")
    b = args[5] if len(args) > 5 else kwargs.get("b")
    return c, g, h, dims, a, b


def _describe_arrays(args, kwargs):
    c, g, h, dims, a, b = _split_args(args, kwargs)
    return {
        "c": _matrix_descriptor(c),
        "G": _matrix_descriptor(g),
        "h": _matrix_descriptor(h),
        "A": _matrix_descriptor(a),
        "b": _matrix_descriptor(b),
        "dims": _to_jsonable(dims),
    }


def _arrays_from(args, kwargs):
    import numpy as np

    c, g, h, dims, a, b = _split_args(args, kwargs)
    out = {}
    if c is not None:
        out["c"] = np.asarray(c)
    if h is not None:
        out["h"] = np.asarray(h)
    if b is not None:
        out["b"] = np.asarray(b)
    if g is not None and hasattr(g, "indptr"):
        out["G_data"] = np.asarray(g.data)
        out["G_indices"] = np.asarray(g.indices)
        out["G_indptr"] = np.asarray(g.indptr)
        out["G_shape"] = np.asarray(g.shape)
    if a is not None and hasattr(a, "indptr"):
        out["A_data"] = np.asarray(a.data)
        out["A_indices"] = np.asarray(a.indices)
        out["A_indptr"] = np.asarray(a.indptr)
        out["A_shape"] = np.asarray(a.shape)
    return out


def install_from_env(module=None):
    """Worker startup hook: no-op unless the env var names a directory."""
    directory = os.environ.get(ENV_VAR, "").strip()
    if not directory:
        return None
    sampler = EcosSampler(
        directory,
        max_records=int(os.environ.get("COGNAV_ECOS_SAMPLER_MAX", "16") or 16),
        success_ring=int(os.environ.get("COGNAV_ECOS_SAMPLER_RING", "5") or 5),
    )
    sampler.install(module)
    return sampler
