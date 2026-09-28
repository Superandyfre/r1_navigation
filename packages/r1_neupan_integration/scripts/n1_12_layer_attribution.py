#!/usr/bin/env python3
"""N1.12 offline attribution harness for the installed CVXPYLayers path.

This script replays the parameter tensors captured by N1.11 directly through a
fresh NeuPAN NRMP CvxpyLayer.  Instrumentation is process-local monkey patching;
the installed packages and the NeuPAN problem are never modified.
"""

from __future__ import annotations

import argparse
import cProfile
import gc
import hashlib
import json
import os
from pathlib import Path
import pstats
import resource
import sys
import time
import tracemalloc
from typing import Any, Callable

import numpy as np


ROOT = Path(__file__).resolve().parents[3]
NEUPAN_ROOT = Path("/home/pinggu/neupan_n0/NeuPAN")
sys.path.insert(0, str(NEUPAN_ROOT))
sys.path.insert(0, str(ROOT / "src" / "r1_neupan_integration" / "scripts"))


def now() -> tuple[int, int]:
    return time.perf_counter_ns(), time.process_time_ns()


def ms(delta: int) -> float:
    return delta / 1_000_000.0


def arr_meta(value: Any) -> dict[str, Any]:
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    a = np.asarray(value)
    return {
        "shape": list(a.shape),
        "numel": int(a.size),
        "dtype": str(a.dtype),
        "strides": list(a.strides),
        "c_contiguous": bool(a.flags.c_contiguous),
        "f_contiguous": bool(a.flags.f_contiguous),
        "writeable": bool(a.flags.writeable),
    }


def sha_array(value: Any) -> str:
    a = np.ascontiguousarray(np.asarray(value))
    return hashlib.sha256(str(a.dtype).encode() + a.shape.__repr__().encode() + a.tobytes()).hexdigest()


def jsonable(value: Any) -> Any:
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    return repr(value)


class Recorder:
    def __init__(self) -> None:
        self.events: list[dict[str, Any]] = []
        self.current: dict[str, Any] | None = None

    def begin(self, sequence: int) -> None:
        self.current = {
            "sequence": sequence,
            "started_perf_ns": now()[0],
            "started_cpu_ns": now()[1],
            "stages": [],
            "ops": [],
            "parameters": [],
        }

    def stage(self, name: str, started: tuple[int, int], ended: tuple[int, int], **extra: Any) -> None:
        if self.current is None:
            return
        self.current["stages"].append({
            "name": name,
            "wall_ms": ms(ended[0] - started[0]),
            "cpu_ms": ms(ended[1] - started[1]),
            **jsonable(extra),
        })

    def op(self, name: str, started: tuple[int, int], ended: tuple[int, int], **extra: Any) -> None:
        if self.current is None:
            return
        self.current["ops"].append({
            "name": name,
            "wall_ms": ms(ended[0] - started[0]),
            "cpu_ms": ms(ended[1] - started[1]),
            **jsonable(extra),
        })

    def end(self, output: Any = None, error: str | None = None) -> dict[str, Any]:
        assert self.current is not None
        ended = now()
        self.current["ended_perf_ns"] = ended[0]
        self.current["ended_cpu_ns"] = ended[1]
        self.current["wall_ms"] = ms(ended[0] - self.current["started_perf_ns"])
        self.current["cpu_ms"] = ms(ended[1] - self.current["started_cpu_ns"])
        if output is not None:
            self.current["outputs"] = [arr_meta(x) for x in output]
            for meta, value in zip(self.current["outputs"], output):
                meta["requires_grad"] = bool(getattr(value, "requires_grad", False))
                meta["grad_fn"] = type(value.grad_fn).__name__ if getattr(value, "grad_fn", None) is not None else None
        if error:
            self.current["error"] = error
        event = self.current
        self.events.append(event)
        self.current = None
        return event


class Hooks:
    def __init__(self, rec: Recorder, detailed: bool = True, solve_only: bool = False) -> None:
        self.rec = rec
        self.detailed = detailed
        self.solve_only = solve_only
        self.originals: dict[tuple[Any, str], Any] = {}

    def replace(self, module: Any, name: str, fn: Any) -> None:
        key = (module, name)
        if key not in self.originals:
            self.originals[key] = getattr(module, name)
        setattr(module, name, fn)

    def install(self) -> None:
        import cvxpylayers.interfaces.diffcp_if as dif
        import cvxpylayers.torch.cvxpylayer as cvx
        import diffcp
        import ecos

        original_flatten = cvx._flatten_and_batch_params
        original_recover = cvx._recover_results
        original_validate = cvx.pa.LayersContext.validate_params
        original_build = dif._build_diffcp_matrices
        original_ecos = ecos.solve
        original_batch_deriv = diffcp.solve_and_derivative_batch
        original_batch_only = diffcp.solve_only_batch

        def validate(ctx: Any, params: list[Any]) -> Any:
            rec = self.rec
            if rec.current is None:
                return original_validate(ctx, params)
            s = now()
            result = original_validate(ctx, params)
            e = now()
            rec.stage("parameter_validation", s, e, batch=list(result), param_count=len(params))
            for i, p in enumerate(params):
                a = p.detach().cpu().numpy() if hasattr(p, "detach") else np.asarray(p)
                rec.current["parameters"].append({
                    "index": i,
                    "name": ctx.parameters[i].name() if i < len(ctx.parameters) else f"parameter_{i}",
                    "device": str(getattr(p, "device", "numpy")),
                    "requires_grad": bool(getattr(p, "requires_grad", False)),
                    **arr_meta(a),
                    "input_hash": sha_array(a),
                })
            return result

        def flatten(params: tuple[Any, ...], ctx: Any, batch: tuple) -> Any:
            if self.rec.current is None or not self.detailed:
                return original_flatten(params, ctx, batch)
            flatten_started = now()
            effective_batch = batch if batch else (1,)
            flattened: list[Any | None] = [None] * (len(params) + 1)
            for i, param in enumerate(params):
                if ctx.batch_sizes[i] == 0:
                    s = now()
                    expanded = param.unsqueeze(0).expand(effective_batch + param.shape)
                    e = now()
                    self.rec.op("parameter_expand", s, e, index=i, shape=arr_meta(expanded.detach().cpu().numpy()))
                else:
                    expanded = param
                triu = ctx.param_triu_indices[i] if ctx.param_triu_indices else None
                if triu is not None:
                    s = now()
                    rows, cols = triu
                    value = expanded[..., rows, cols]
                    e = now()
                    self.rec.op("parameter_triu_index", s, e, index=i, shape=arr_meta(value.detach().cpu().numpy()))
                else:
                    s = now()
                    value = cvx._reshape_fortran(expanded, effective_batch + (-1,))
                    e = now()
                    self.rec.op("parameter_reshape_fortran", s, e, index=i, shape=arr_meta(value.detach().cpu().numpy()))
                # This is the per-parameter timing requested by N1.12.  The
                # operation above includes the actual tensor mapping, while
                # this record also preserves the parameter identity.
                meta = self.rec.current["parameters"][i]
                meta["flattened_shape"] = list(value.shape)
                meta["flattened_numel"] = int(value.numel())
                meta["flattened_hash"] = sha_array(value.detach().cpu().numpy())
                flattened[ctx.user_order_to_col_order[i]] = value
            s = now()
            flattened[-1] = cvx.torch.ones(effective_batch + (1,), dtype=params[0].dtype, device=params[0].device)
            e = now()
            self.rec.op("parameter_constant_one", s, e, shape=list(flattened[-1].shape))
            s = now()
            p_stack = cvx.torch.cat(flattened, -1)
            e = now()
            self.rec.op("torch.cat_flattened_parameters", s, e, input_count=len(flattened), output_shape=list(p_stack.shape), output_numel=int(p_stack.numel()))
            s = now()
            p_stack = p_stack.T
            e = now()
            self.rec.op("parameter_transpose", s, e, output_shape=list(p_stack.shape), strides=list(p_stack.stride()))
            output_shape = (p_stack.shape[0],) + batch
            s = now()
            out = p_stack.reshape(output_shape)
            e = now()
            self.rec.op("parameter_final_reshape", s, e, output_shape=list(out.shape), contiguous=bool(out.is_contiguous()))
            self.rec.stage("parameter_flatten_detailed", flatten_started, now(), param_count=len(params), output_shape=list(out.shape))
            return out

        def build(con_values: Any, lin_obj_values: Any, structure: Any, shape: Any, b_idx: Any, batch_size: int) -> Any:
            if self.rec.current is None:
                return original_build(con_values, lin_obj_values, structure, shape, b_idx, batch_size)
            marshal_started = now()
            out_as, out_bs, out_cs, out_idxs = [], [], [], []
            for i in range(batch_size):
                s = now()
                con_i = np.array(con_values[:, i])
                e = now(); self.rec.op("diffcp_con_values_numpy", s, e, batch_index=i, shape=arr_meta(con_i))
                s = now()
                lin_i = np.array(lin_obj_values[:-1, i])
                e = now(); self.rec.op("diffcp_objective_numpy", s, e, batch_index=i, shape=arr_meta(lin_i))
                s = now()
                aug = dif.sp.csc_matrix((con_i, *structure), shape=shape)
                e = now(); self.rec.op("scipy_csc_assembly", s, e, batch_index=i, shape=list(aug.shape), nnz=int(aug.nnz), dtype=str(aug.dtype))
                s = now(); a = -aug[:, :-1]; e = now(); self.rec.op("scipy_csc_slice_negate", s, e, batch_index=i, shape=list(a.shape), nnz=int(a.nnz))
                s = now(); b = aug[:, -1].toarray().flatten(); e = now(); self.rec.op("scipy_sparse_to_dense_b", s, e, batch_index=i, shape=arr_meta(b))
                out_as.append(a); out_bs.append(b); out_cs.append(lin_i); out_idxs.append(b_idx)
            self.rec.stage("diffcp_input_marshaling", marshal_started, now(), batch_size=batch_size, sparse_shape=list(shape), parameter_vector_length=int(con_values.shape[0]))
            return out_as, out_bs, out_cs, out_idxs

        def ecos_wrapper(*args: Any, **kwargs: Any) -> Any:
            if self.rec.current is None:
                return original_ecos(*args, **kwargs)
            s = now(); result = original_ecos(*args, **kwargs); e = now()
            c, G, h = args[:3]
            self.rec.stage("native_ecos", s, e, c_shape=list(np.asarray(c).shape), G_shape=list(G.shape), G_nnz=int(G.nnz), h_shape=list(np.asarray(h).shape), status=jsonable(result.get("info", {}).get("exitFlag") if isinstance(result, dict) else None))
            return result

        def diffcp_deriv(*args: Any, **kwargs: Any) -> Any:
            if self.rec.current is None:
                return original_batch_deriv(*args, **kwargs)
            s = now(); result = original_batch_deriv(*args, **kwargs); e = now()
            self.rec.stage("diffcp_solve_and_derivative_batch", s, e, batch_size=len(args[0]), mode="derivative")
            return result

        def diffcp_only(*args: Any, **kwargs: Any) -> Any:
            if self.rec.current is None:
                return original_batch_only(*args, **kwargs)
            s = now(); result = original_batch_only(*args, **kwargs); e = now()
            self.rec.stage("diffcp_solve_only_batch", s, e, batch_size=len(args[0]), mode="solve_only")
            return result

        def recover(primal: Any, dual: Any, ctx: Any, batch: tuple) -> Any:
            if self.rec.current is None or not self.detailed:
                return original_recover(primal, dual, ctx, batch)
            recovery_started = now()
            results = []
            internal_batch = tuple(primal.shape[:-1])
            for index, var in enumerate(ctx.var_recover):
                s = now()
                data = primal[..., var.primal] if var.source == "primal" else dual[..., var.dual]
                e = now(); self.rec.op("result_variable_slice", s, e, index=index, source=var.source, shape=list(data.shape))
                s = now()
                if var.unpack_fn == "svec_primal":
                    value = cvx._unpack_primal_svec(data, var.shape[0], internal_batch)
                elif var.unpack_fn == "svec_dual":
                    value = cvx._unpack_svec(data, var.shape[0], internal_batch)
                else:
                    value = cvx._reshape_fortran(data, internal_batch + var.shape)
                e = now(); self.rec.op("result_unpack_or_reshape", s, e, index=index, unpack_fn=var.unpack_fn, shape=list(value.shape))
                s = now(); value = value.reshape(batch + var.shape); e = now(); self.rec.op("result_final_reshape", s, e, index=index, shape=list(value.shape))
                results.append(value)
            if ctx.gp:
                results = [cvx.torch.exp(r) if var.source == "primal" else r for r, var in zip(results, ctx.var_recover)]
            self.rec.stage("result_recovery", recovery_started, now(), result_count=len(results))
            return tuple(results)

        def sparse_apply(cls: Any, *args: Any, **kwargs: Any) -> Any:
            # This hook is installed on the static forward method below.
            return original_sparse_forward(*args, **kwargs)

        def sparse_forward(scipy_csr: Any, x: Any) -> Any:
            if self.rec.current is None:
                return original_sparse_forward(scipy_csr, x)
            name = f"scipy_sparse_matmul_{len([x for x in self.rec.current['ops'] if x['name'].startswith('scipy_sparse_matmul')])}"
            s = now(); x_detached = x.detach(); e = now(); self.rec.op(name + ".detach", s, e, input_shape=list(x.shape), input_dtype=str(x.dtype), input_requires_grad=bool(x.requires_grad))
            s = now(); x_np = x_detached.numpy(); e = now(); self.rec.op(name + ".numpy", s, e, input_meta=arr_meta(x_np))
            s = now(); result = scipy_csr @ x_np; e = now(); self.rec.op(name + ".scipy_csr_matmul", s, e, sparse_shape=list(scipy_csr.shape), sparse_nnz=int(scipy_csr.nnz), output_shape=list(np.asarray(result).shape))
            s = now(); result_np = np.asarray(result); e = now(); self.rec.op(name + ".asarray", s, e, output_meta=arr_meta(result_np))
            s = now(); out = cvx.torch.from_numpy(result_np); e = now(); self.rec.op(name + ".torch_from_numpy", s, e, output_shape=list(out.shape), output_dtype=str(out.dtype))
            return out

        original_sparse_forward = cvx._ScipySparseMatmul.forward
        self.replace(cvx._ScipySparseMatmul, "forward", staticmethod(sparse_forward))
        self.replace(cvx, "_flatten_and_batch_params", flatten)
        self.replace(cvx, "_recover_results", recover)
        self.replace(cvx.pa.LayersContext, "validate_params", validate)
        self.replace(dif, "_build_diffcp_matrices", build)
        self.replace(ecos, "solve", ecos_wrapper)
        self.replace(diffcp, "solve_and_derivative_batch", diffcp_deriv)
        self.replace(diffcp, "solve_only_batch", diffcp_only)


def runtime_inventory() -> dict[str, Any]:
    import importlib
    import torch
    import threadpoolctl
    names = ["cvxpy", "cvxpylayers", "diffcp", "scipy", "numpy", "torch", "threadpoolctl"]
    out: dict[str, Any] = {"python": sys.executable, "env": {k: os.environ.get(k) for k in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS")}}
    for name in names:
        mod = importlib.import_module(name)
        out[name] = {"version": getattr(mod, "__version__", None), "file": getattr(mod, "__file__", None)}
    out["torch_threads"] = {"intra": torch.get_num_threads(), "inter": torch.get_num_interop_threads()}
    out["threadpools"] = threadpoolctl.threadpool_info()
    return out


def build_layer(config: str, checkpoint: str) -> Any:
    from neupan import neupan
    return neupan.init_from_yaml(config, pan={"iter_num": 2, "dune_max_num": 200, "nrmp_max_num": 10, "iter_threshold": 0.1, "dune_checkpoint": checkpoint}, device="cpu").pan.nrmp_layer.nrmp_layer


def load_params(layer: Any, cycle_dir: Path, requires_grad: bool = True, contiguous: str = "original") -> tuple[Any, ...]:
    import torch
    values = []
    files = sorted(cycle_dir.glob("layer_000_parameter_*.npy"))
    if len(files) != len(layer.ctx.parameters):
        raise RuntimeError(f"expected {len(layer.ctx.parameters)} parameter files, found {len(files)} in {cycle_dir}")
    for i, path in enumerate(files):
        value = torch.from_numpy(np.load(path, allow_pickle=False))
        if contiguous == "contiguous":
            value = value.contiguous()
        elif contiguous == "clone":
            value = value.clone().contiguous()
        # Online NeuPAN has gradient-carrying gamma parameters (1, 2) and
        # the five scalar adjustment parameters (33..37).  Preserve this
        # execution boundary without changing any values or solver settings.
        if requires_grad and (i in (1, 2) or i >= 33):
            value.requires_grad_(True)
        values.append(value)
    return tuple(values)


def run_forward(layer: Any, params: tuple[Any, ...], rec: Recorder, sequence: int, solve_only: bool = False, gc_enabled: bool = True) -> dict[str, Any]:
    if gc_enabled:
        gc.enable()
    else:
        gc.disable()
    rec.begin(sequence)
    try:
        with __import__("torch").set_grad_enabled(not solve_only):
            output = layer(*params, solver_args={"solve_method": "ECOS"})
        return rec.end(output=output)
    except Exception as exc:
        return rec.end(error=f"{type(exc).__name__}: {exc}")
    finally:
        if not gc_enabled:
            gc.enable()


def save_events(out: Path, events: list[dict[str, Any]], inventory: dict[str, Any], args: argparse.Namespace) -> None:
    out.mkdir(parents=True, exist_ok=True)
    (out / "runtime.json").write_text(json.dumps(jsonable({"runtime": inventory, "args": vars(args)}), indent=2, sort_keys=True), encoding="utf-8")
    with (out / "events.jsonl").open("w", encoding="utf-8") as f:
        for event in events:
            f.write(json.dumps(jsonable(event), sort_keys=True) + "\n")


def capture(args: argparse.Namespace) -> None:
    import torch
    if args.torch_threads:
        torch.set_num_threads(args.torch_threads)
    if args.interop_threads:
        try:
            torch.set_num_interop_threads(args.interop_threads)
        except RuntimeError:
            pass
    snapshot = Path(args.snapshot)
    cycle_dir = snapshot if snapshot.is_dir() else snapshot.parent
    layer = build_layer(args.config, args.checkpoint)
    params = load_params(layer, cycle_dir, requires_grad=not args.solve_only, contiguous=args.layout)
    rec = Recorder()
    if not args.no_hooks:
        Hooks(rec, detailed=not args.minimal, solve_only=args.solve_only).install()
    for i in range(args.warmup):
        run_forward(layer, params, rec, -args.warmup + i, solve_only=args.solve_only)
    rec.events.clear()
    for i in range(args.measured):
        run_forward(layer, params, rec, i, solve_only=args.solve_only, gc_enabled=not args.gc_disable)
    save_events(Path(args.output), rec.events, runtime_inventory(), args)


def microbench(args: argparse.Namespace) -> None:
    import torch
    import cvxpylayers.torch.cvxpylayer as cvx
    if args.torch_threads:
        torch.set_num_threads(args.torch_threads)
    cycle_dir = Path(args.snapshot)
    layer = build_layer(args.config, args.checkpoint)
    params = load_params(layer, cycle_dir, requires_grad=False, contiguous="original")
    layer.ctx.validate_params(list(params))
    rows = []
    for i, p in enumerate(params):
        for op_name, op in [
            ("reshape", lambda x: x.reshape(-1)),
            ("flatten", lambda x: x.flatten()),
            ("contiguous", lambda x: x.contiguous()),
            ("detach", lambda x: x.detach()),
            ("cpu", lambda x: x.cpu()),
            ("clone", lambda x: x.clone()),
            ("numpy", lambda x: x.detach().numpy()),
            ("indexing", lambda x: x[...]),
            ("gather", lambda x: x.reshape(-1)[torch.arange(x.numel())]),
            ("stack_same", lambda x: torch.stack((x, x))),
        ]:
            for _ in range(args.warmup): op(p)
            vals = []
            for _ in range(args.measured):
                s = time.perf_counter_ns(); op(p); vals.append(ms(time.perf_counter_ns() - s))
            rows.append({"parameter": i, "name": layer.ctx.parameters[i].name(), "op": op_name, "shape": list(p.shape), "dtype": str(p.dtype), "device": str(p.device), "threads": torch.get_num_threads(), "mean_ms": float(np.mean(vals)), "p95_ms": float(np.percentile(vals, 95)), "max_ms": float(np.max(vals))})
    def flatten_call() -> Any:
        return cvx._flatten_and_batch_params(params, layer.ctx, ())
    for op_name, op in [("actual_flatten_and_batch", flatten_call), ("cat_all_flattened", lambda: torch.cat([p.reshape(-1) for p in params], -1))]:
        for _ in range(args.warmup): op()
        vals = []
        for _ in range(args.measured):
            s = time.perf_counter_ns(); op(); vals.append(ms(time.perf_counter_ns() - s))
        rows.append({"parameter": -1, "name": "all_parameters", "op": op_name, "shape": [int(sum(p.numel() for p in params))], "dtype": str(params[0].dtype), "device": str(params[0].device), "threads": torch.get_num_threads(), "mean_ms": float(np.mean(vals)), "p95_ms": float(np.percentile(vals, 95)), "max_ms": float(np.max(vals))})
    Path(args.output).mkdir(parents=True, exist_ok=True)
    (Path(args.output) / "microbench.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")


def long_sequence(args: argparse.Namespace) -> None:
    import torch
    if args.torch_threads:
        torch.set_num_threads(args.torch_threads)
    layer = build_layer(args.config, args.checkpoint)
    params = load_params(layer, Path(args.snapshot), requires_grad=not args.solve_only, contiguous=args.layout)
    rec = Recorder()
    if not args.no_hooks:
        Hooks(rec, detailed=False, solve_only=args.solve_only).install()
    for i in range(args.warmup): run_forward(layer, params, rec, -args.warmup + i, solve_only=args.solve_only)
    rec.events.clear()
    for i in range(args.measured): run_forward(layer, params, rec, i, solve_only=args.solve_only)
    save_events(Path(args.output), rec.events, runtime_inventory(), args)
    vals = np.array([e.get("wall_ms", np.nan) for e in rec.events], dtype=float)
    intervals = np.diff(np.flatnonzero(vals > args.spike_ms))
    summary = {"count": int(len(vals)), "mean_ms": float(np.nanmean(vals)), "p95_ms": float(np.nanpercentile(vals,95)), "max_ms": float(np.nanmax(vals)), "spike_threshold_ms": args.spike_ms, "spike_indices": np.flatnonzero(vals > args.spike_ms).tolist(), "spike_intervals": intervals.tolist(), "minor_faults": resource.getrusage(resource.RUSAGE_SELF).ru_minflt, "major_faults": resource.getrusage(resource.RUSAGE_SELF).ru_majflt, "max_rss": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss}
    (Path(args.output) / "long_sequence_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")


def direct_cvxpy(args: argparse.Namespace) -> None:
    import cvxpy as cp
    import torch
    if args.torch_threads:
        torch.set_num_threads(args.torch_threads)
    layer = build_layer(args.config, args.checkpoint)
    params = load_params(layer, Path(args.snapshot), requires_grad=False, contiguous=args.layout)
    problem = layer.ctx.problem

    def call(sequence: int) -> dict[str, Any]:
        p0, c0 = now()
        for obj, value in zip(layer.ctx.parameters, params):
            obj.value = value.detach().cpu().numpy()
        p1, c1 = now()
        problem.solve(solver=cp.ECOS, warm_start=False, verbose=False)
        p2, c2 = now()
        values = [np.array(v.value, copy=True) for v in layer.ctx.variables]
        p3, c3 = now()
        return {"sequence": sequence, "prep_wall_ms": ms(p1-p0), "prep_cpu_ms": ms(c1-c0), "solve_wall_ms": ms(p2-p1), "solve_cpu_ms": ms(c2-c1), "recovery_wall_ms": ms(p3-p2), "recovery_cpu_ms": ms(c3-c2), "wall_ms": ms(p3-p0), "cpu_ms": ms(c3-c0), "status": str(problem.status), "outputs": [arr_meta(v) for v in values]}
    for i in range(args.warmup): call(-args.warmup + i)
    events = [call(i) for i in range(args.measured)]
    save_events(Path(args.output), events, runtime_inventory(), args)
    (Path(args.output) / "direct_summary.json").write_text(json.dumps(summary_for_events(events, "wall_ms"), indent=2), encoding="utf-8")


def summary_for_events(events: list[dict[str, Any]], key: str) -> dict[str, Any]:
    values = np.asarray([e[key] for e in events], dtype=float)
    return {"count": int(len(values)), "mean_ms": float(values.mean()), "p50_ms": float(np.percentile(values, 50)), "p95_ms": float(np.percentile(values, 95)), "p99_ms": float(np.percentile(values, 99)), "max_ms": float(values.max()), "spikes_over_20_ms": int(np.sum(values > 20.0)), "spikes_over_100_ms": int(np.sum(values > 100.0))}


def mapping_benchmark(args: argparse.Namespace) -> None:
    import cvxpylayers.torch.cvxpylayer as cvx
    import torch
    if args.torch_threads:
        torch.set_num_threads(args.torch_threads)
    layer = build_layer(args.config, args.checkpoint)
    params = load_params(layer, Path(args.snapshot), requires_grad=False, contiguous=args.layout)
    layer.ctx.validate_params(list(params))
    p_stack = cvx._flatten_and_batch_params(params, layer.ctx, ())
    rows = []
    for name, matrix in [("q_mapping", layer._q_scipy), ("A_mapping", layer._A_scipy)]:
        for _ in range(args.warmup): matrix @ p_stack.detach().numpy()
        vals = []
        cpus = []
        for _ in range(args.measured):
            s, c = now(); matrix @ p_stack.detach().numpy(); e, d = now(); vals.append(ms(e-s)); cpus.append(ms(d-c))
        rows.append({"op": name, "shape": list(matrix.shape), "nnz": int(matrix.nnz), "parameter_vector_shape": list(p_stack.shape), "mean_ms": float(np.mean(vals)), "p95_ms": float(np.percentile(vals,95)), "max_ms": float(np.max(vals)), "cpu_mean_ms": float(np.mean(cpus)), "format": type(matrix).__name__})
    Path(args.output).mkdir(parents=True, exist_ok=True)
    (Path(args.output) / "mapping.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    (Path(args.output) / "runtime.json").write_text(json.dumps(runtime_inventory(), indent=2), encoding="utf-8")


def lifecycle_benchmark(args: argparse.Namespace) -> None:
    import torch
    import cvxpylayers.torch.cvxpylayer as cvx
    if args.torch_threads:
        torch.set_num_threads(args.torch_threads)
    base = build_layer(args.config, args.checkpoint)
    params = load_params(base, Path(args.snapshot), requires_grad=not args.solve_only, contiguous=args.layout)
    for _ in range(args.warmup): base(*params, solver_args={"solve_method": "ECOS"})
    persistent = []
    for i in range(args.measured):
        s, c = now(); base(*params, solver_args={"solve_method": "ECOS"}); e, d = now(); persistent.append({"sequence": i, "wall_ms": ms(e-s), "cpu_ms": ms(d-c)})
    fresh = []
    for i in range(args.fresh_measured):
        s, c = now()
        layer = cvx.CvxpyLayer(base.ctx.problem, parameters=base.ctx.parameters, variables=base.ctx.variables)
        layer(*params, solver_args={"solve_method": "ECOS"})
        e, d = now(); fresh.append({"sequence": i, "wall_ms": ms(e-s), "cpu_ms": ms(d-c)})
    Path(args.output).mkdir(parents=True, exist_ok=True)
    (Path(args.output) / "persistent.json").write_text(json.dumps({"summary": summary_for_events(persistent,"wall_ms"), "events": persistent}, indent=2), encoding="utf-8")
    (Path(args.output) / "fresh.json").write_text(json.dumps({"summary": summary_for_events(fresh,"wall_ms"), "events": fresh}, indent=2), encoding="utf-8")


def mutation_check(args: argparse.Namespace) -> None:
    import torch
    if args.torch_threads:
        torch.set_num_threads(args.torch_threads)
    layer = build_layer(args.config, args.checkpoint)
    params = load_params(layer, Path(args.snapshot), requires_grad=not args.solve_only, contiguous=args.layout)
    def digest(value: Any) -> str:
        if hasattr(value, "detach"):
            value = value.detach().cpu().numpy()
        return sha_array(value)
    before = {"params": [digest(x) for x in params], "q_data": digest(layer._q_scipy.data), "q_indices": digest(layer._q_scipy.indices), "q_indptr": digest(layer._q_scipy.indptr), "A_data": digest(layer._A_scipy.data), "A_indices": digest(layer._A_scipy.indices), "A_indptr": digest(layer._A_scipy.indptr)}
    for _ in range(args.measured): layer(*params, solver_args={"solve_method": "ECOS"})
    after = {"params": [digest(x) for x in params], "q_data": digest(layer._q_scipy.data), "q_indices": digest(layer._q_scipy.indices), "q_indptr": digest(layer._q_scipy.indptr), "A_data": digest(layer._A_scipy.data), "A_indices": digest(layer._A_scipy.indices), "A_indptr": digest(layer._A_scipy.indptr)}
    Path(args.output).mkdir(parents=True, exist_ok=True)
    (Path(args.output) / "mutation.json").write_text(json.dumps({"before": before, "after": after, "equal": before == after, "forward_count": args.measured}, indent=2), encoding="utf-8")


def profile_run(args: argparse.Namespace) -> None:
    import torch
    if args.torch_threads:
        torch.set_num_threads(args.torch_threads)
    layer = build_layer(args.config, args.checkpoint)
    params = load_params(layer, Path(args.snapshot), requires_grad=not args.solve_only, contiguous=args.layout)
    for _ in range(args.warmup): layer(*params, solver_args={"solve_method": "ECOS"})
    Path(args.output).mkdir(parents=True, exist_ok=True)
    profiler = cProfile.Profile()
    profiler.enable()
    for _ in range(args.measured): layer(*params, solver_args={"solve_method": "ECOS"})
    profiler.disable()
    profiler.dump_stats(str(Path(args.output) / "cprofile.prof"))
    with (Path(args.output) / "cprofile_top.txt").open("w", encoding="utf-8") as f:
        pstats.Stats(profiler, stream=f).sort_stats("cumulative").print_stats(40)
    with torch.profiler.profile(activities=[torch.profiler.ProfilerActivity.CPU], record_shapes=True) as tp:
        for _ in range(min(args.measured, 10)):
            layer(*params, solver_args={"solve_method": "ECOS"})
    (Path(args.output) / "torch_profiler_top.txt").write_text(str(tp.key_averages().table(sort_by="self_cpu_time_total", row_limit=40)), encoding="utf-8")


def allocation_run(args: argparse.Namespace) -> None:
    import sys as sysmod
    import torch
    if args.torch_threads:
        torch.set_num_threads(args.torch_threads)
    layer = build_layer(args.config, args.checkpoint)
    params = load_params(layer, Path(args.snapshot), requires_grad=not args.solve_only, contiguous=args.layout)
    for _ in range(args.warmup):
        layer(*params, solver_args={"solve_method": "ECOS"})
    def sample() -> tuple[float, int, int, int]:
        s = time.perf_counter_ns();
        for _ in range(args.measured): layer(*params, solver_args={"solve_method": "ECOS"})
        wall = ms(time.perf_counter_ns() - s)
        usage = resource.getrusage(resource.RUSAGE_SELF)
        return wall / args.measured, sysmod.getallocatedblocks(), usage.ru_minflt, usage.ru_majflt
    off = sample()
    tracemalloc.start()
    on = sample()
    current, peak = tracemalloc.get_traced_memory()
    snapshot = tracemalloc.take_snapshot()
    top = [{"trace": str(stat.traceback), "size": stat.size, "count": stat.count} for stat in snapshot.statistics("lineno")[:20]]
    tracemalloc.stop()
    Path(args.output).mkdir(parents=True, exist_ok=True)
    (Path(args.output) / "allocation.json").write_text(json.dumps({"measured": args.measured, "tracemalloc_off_mean_ms": off[0], "tracemalloc_on_mean_ms": on[0], "tracemalloc_overhead_percent": (on[0]/off[0]-1.0)*100.0, "allocated_blocks_off": off[1], "allocated_blocks_on": on[1], "minor_faults_off": off[2], "minor_faults_on": on[2], "major_faults_off": off[3], "major_faults_on": on[3], "tracemalloc_current_bytes": current, "tracemalloc_peak_bytes": peak, "top_allocations": top}, indent=2), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["capture", "microbench", "long", "direct", "mapping", "lifecycle", "mutation", "profile", "allocation"], required=True)
    parser.add_argument("--snapshot", required=True)
    parser.add_argument("--config", default="src/r1_neupan_integration/config/planner.yaml")
    parser.add_argument("--checkpoint", default="src/r1_neupan_integration/config/models/dune_model_5000.pth")
    parser.add_argument("--output", required=True)
    parser.add_argument("--warmup", type=int, default=30)
    parser.add_argument("--measured", type=int, default=300)
    parser.add_argument("--fresh-measured", type=int, default=30)
    parser.add_argument("--torch-threads", type=int, default=1)
    parser.add_argument("--interop-threads", type=int, default=1)
    parser.add_argument("--solve-only", action="store_true")
    parser.add_argument("--minimal", action="store_true")
    parser.add_argument("--no-hooks", action="store_true")
    parser.add_argument("--gc-disable", action="store_true")
    parser.add_argument("--layout", choices=["original", "contiguous", "clone"], default="original")
    parser.add_argument("--spike-ms", type=float, default=500.0)
    args = parser.parse_args()
    if args.mode == "capture": capture(args)
    elif args.mode == "microbench": microbench(args)
    elif args.mode == "long": long_sequence(args)
    elif args.mode == "direct": direct_cvxpy(args)
    elif args.mode == "mapping": mapping_benchmark(args)
    elif args.mode == "lifecycle": lifecycle_benchmark(args)
    elif args.mode == "mutation": mutation_check(args)
    elif args.mode == "profile": profile_run(args)
    else: allocation_run(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
