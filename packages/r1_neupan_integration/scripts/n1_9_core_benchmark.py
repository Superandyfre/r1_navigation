#!/usr/bin/env python3
"""Run the fixed N1 obstacle snapshot through all NeuPAN stages.

This is an experiment harness.  It does not alter a NeuPAN YAML or core
source, and starts a fresh process for each selected thread configuration.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import resource
import sys
import time
import traceback
from pathlib import Path
from typing import Any

import numpy as np
import torch
import yaml


THREAD_ENV_NAMES = ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS", "NUMEXPR_NUM_THREADS")


def stats(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "mean_ms": None, "median_ms": None, "p95_ms": None, "max_ms": None}
    ordered = np.asarray(values, dtype=float)
    return {
        "count": int(ordered.size), "mean_ms": float(ordered.mean()),
        "median_ms": float(np.median(ordered)), "p95_ms": float(np.percentile(ordered, 95)),
        "max_ms": float(ordered.max()),
    }


def value_stats(values: list[float]) -> dict[str, float | int | None]:
    result = stats(values)
    return {key.replace("_ms", ""): value for key, value in result.items()}


def sha256(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def set_path(planner: Any, initial_path: np.ndarray) -> None:
    path = [np.vstack([initial_path[:, index:index + 1], [[1.0]]]) for index in range(initial_path.shape[1])]
    try:
        planner.set_initial_path(path)
    except TypeError as exc:
        if "0-dimensional arrays" not in str(exc):
            raise
        # NumPy 2.x scalar compatibility only.  The frozen coordinates remain
        # untouched and this mirrors the N1.8 harness.
        def average_interval(points: list[np.ndarray]) -> float:
            if len(points) < 2:
                return 0.0
            return float(np.mean([
                math.hypot(float(np.asarray(b[:2] - a[:2]).reshape(2)[0]),
                           float(np.asarray(b[:2] - a[:2]).reshape(2)[1]))
                for a, b in zip(points, points[1:])
            ]))
        planner.ipath.cal_average_interval = average_interval
        planner.set_initial_path(path)


def finite_tree(value: Any) -> bool:
    if isinstance(value, dict):
        return all(finite_tree(item) for item in value.values())
    if isinstance(value, (list, tuple)):
        return all(finite_tree(item) for item in value)
    if isinstance(value, (str, bool, type(None))):
        return True
    try:
        if isinstance(value, torch.Tensor):
            value = value.detach()
        array = np.asarray(value)
        return bool(np.all(np.isfinite(array)))
    except (TypeError, ValueError):
        return True


class Instrument:
    def __init__(self, planner: Any):
        self.planner = planner
        self.full: list[float] = []
        self.dune: list[float] = []
        self.nrmp: list[float] = []
        self.pan: list[float] = []
        self.cpu_pct: list[float] = []
        self.iterations: list[float] = []
        self.solver_calls = 0
        self.solver_failures = 0
        self.solver_wall: list[float] = []
        self.solver_cpu: list[float] = []
        self.solver_events: list[dict[str, Any]] = []
        self.uncaught_exceptions: list[dict[str, str]] = []
        self.nan_inf_outputs = 0
        self._cycle_iterations = 0.0
        self._wrap()

    @staticmethod
    def _elapsed(start: int) -> float:
        return (time.perf_counter_ns() - start) / 1_000_000.0

    def _wrap(self) -> None:
        pan = self.planner.pan

        def wrap(module: Any, attr: str, values: list[float]) -> None:
            original = getattr(module, attr)
            def timed(*args: Any, **kwargs: Any) -> Any:
                started = time.perf_counter_ns()
                try:
                    return original(*args, **kwargs)
                finally:
                    values.append(self._elapsed(started))
            setattr(module, attr, timed)

        original_stop = pan.stop_criteria
        def counted_stop(*args: Any, **kwargs: Any) -> Any:
            self._cycle_iterations += 1.0
            return original_stop(*args, **kwargs)
        pan.stop_criteria = counted_stop

        solver = getattr(getattr(pan, "nrmp_layer", None), "nrmp_layer", None)
        if solver is not None and getattr(solver, "forward", None) is not None:
            original_solver = solver.forward
            def timed_solver(*args: Any, **kwargs: Any) -> Any:
                wall_start = time.perf_counter_ns()
                cpu_start = time.process_time_ns()
                self.solver_calls += 1
                event = {"status": "return-success"}
                try:
                    return original_solver(*args, **kwargs)
                except Exception as exc:
                    self.solver_failures += 1
                    event["status"] = "exception"
                    event["exception"] = f"{type(exc).__name__}: {exc}"
                    raise
                finally:
                    wall_ms = (time.perf_counter_ns() - wall_start) / 1_000_000.0
                    cpu_ms = (time.process_time_ns() - cpu_start) / 1_000_000.0
                    self.solver_wall.append(wall_ms)
                    self.solver_cpu.append(cpu_ms)
                    event.update({
                        "wall_ms": wall_ms,
                        "cpu_ms": cpu_ms,
                        "status_detail": "not exposed by cvxpylayers",
                        "iterations": "not exposed",
                        "objective": "not exposed",
                        "primal_residual": "not exposed",
                        "dual_residual": "not exposed",
                        "gap": "not exposed",
                        "setup_ms": "not exposed",
                    })
                    self.solver_events.append(event)
            solver.forward = timed_solver

        wrap(pan.dune_layer, "forward", self.dune)
        wrap(pan.nrmp_layer, "forward", self.nrmp)
        wrap(pan, "forward", self.pan)

    def reset(self) -> None:
        self.full.clear(); self.dune.clear(); self.nrmp.clear(); self.pan.clear(); self.cpu_pct.clear(); self.iterations.clear()
        self.solver_calls = 0; self.solver_failures = 0
        self.solver_wall.clear(); self.solver_cpu.clear(); self.solver_events.clear()
        self.uncaught_exceptions.clear(); self.nan_inf_outputs = 0

    def run_one(self, state: np.ndarray, points: np.ndarray) -> bool:
        self._cycle_iterations = 0.0
        wall_start = time.perf_counter_ns()
        cpu_start = time.process_time_ns()
        try:
            action, info = self.planner(state, points)
        except Exception as exc:
            self.uncaught_exceptions.append({"type": type(exc).__name__, "message": str(exc)})
            return False
        wall_ms = self._elapsed(wall_start)
        cpu_ms = (time.process_time_ns() - cpu_start) / 1_000_000.0
        self.full.append(wall_ms)
        if wall_ms > 0:
            self.cpu_pct.append(cpu_ms / wall_ms * 100.0)
        self.iterations.append(self._cycle_iterations)
        if not finite_tree(action) or not finite_tree(info):
            self.nan_inf_outputs += 1
            return False
        return True


def configure_runtime(intra: int, interop: int, env_policy: str) -> dict[str, Any]:
    before = {name: os.environ.get(name) for name in THREAD_ENV_NAMES}
    if env_policy == "match_intra":
        for name in THREAD_ENV_NAMES:
            os.environ[name] = str(intra)
    torch.set_num_threads(intra)
    if interop:
        torch.set_num_interop_threads(interop)
    return {
        "requested": {"torch_num_threads": intra, "torch_num_interop_threads": interop or None, "thread_env_policy": env_policy},
        "effective": {"torch_num_threads": torch.get_num_threads(), "torch_num_interop_threads": torch.get_num_interop_threads()},
        "environment_before": before,
        "environment_effective": {name: os.environ.get(name) for name in THREAD_ENV_NAMES},
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--snapshot", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--torch-num-threads", type=int, required=True, choices=(1, 2, 4, 8))
    parser.add_argument("--torch-num-interop-threads", type=int, default=0)
    parser.add_argument("--thread-env-policy", choices=("inherit", "match_intra"), default="inherit")
    parser.add_argument("--warmup", type=int, default=50)
    parser.add_argument("--samples", type=int, default=500)
    args = parser.parse_args()
    runtime = configure_runtime(args.torch_num_threads, args.torch_num_interop_threads, args.thread_env_policy)
    result: dict[str, Any] = {
        "schema": "r1_n1_9_core_benchmark_v1",
        "method": {"warmup": args.warmup, "samples": args.samples, "clock": "perf_counter_ns", "frozen_snapshot": True},
        "runtime": runtime,
        "snapshot": {"path": str(Path(args.snapshot).resolve()), "sha256": sha256(args.snapshot)},
        "config": {"path": str(Path(args.config).resolve()), "sha256": sha256(args.config)},
    }
    try:
        from neupan import neupan
        snapshot = np.load(args.snapshot)
        planner_yaml = yaml.safe_load(Path(args.config).read_text(encoding="utf-8")) or {}
        pan = dict(planner_yaml.get("pan") or {})
        pan["dune_checkpoint"] = args.checkpoint
        planner = neupan.init_from_yaml(args.config, pan=pan, device="cpu")
        set_path(planner, snapshot["initial_path"].astype(float))
        instrument = Instrument(planner)
        state = snapshot["robot_state"].astype(float)
        points = snapshot["obstacle_points"].astype(float)
        for _ in range(args.warmup):
            if not instrument.run_one(state, points):
                raise RuntimeError("warmup produced an invalid result")
        instrument.reset()
        for _ in range(args.samples):
            instrument.run_one(state, points)
        result["effective_pan"] = {
            "receding": planner.T, "iter_num": planner.pan.iter_num,
            "iter_threshold": planner.pan.iter_threshold, "dune_max_num": planner.pan.dune_max_num,
            "nrmp_max_num": planner.pan.nrmp_max_num,
        }
        result["timing_ms"] = {
            "dune": stats(instrument.dune), "nrmp": stats(instrument.nrmp),
            "pan": stats(instrument.pan), "full_forward": stats(instrument.full),
            "ecos_solve_wall": stats(instrument.solver_wall),
            "ecos_solve_cpu": stats(instrument.solver_cpu),
        }
        result["cpu_percent"] = value_stats(instrument.cpu_pct)
        result["validation"] = {
            "total_forwards": args.samples, "completed_forwards": len(instrument.full),
            "solver_calls": instrument.solver_calls, "ecos_success": instrument.solver_calls - instrument.solver_failures,
            "ecos_failure": instrument.solver_failures, "uncaught_solver_error": len(instrument.uncaught_exceptions),
            "fallback_count": 0, "nan_inf_outputs": instrument.nan_inf_outputs,
            "pan_iterations": value_stats(instrument.iterations), "exceptions": instrument.uncaught_exceptions[:10],
            "solver_events": instrument.solver_events[:20],
        }
    except Exception as exc:
        result["exception"] = {"type": type(exc).__name__, "message": str(exc), "traceback": traceback.format_exc()}
    result["process"] = {"cpu_user_sec": resource.getrusage(resource.RUSAGE_SELF).ru_utime, "cpu_system_sec": resource.getrusage(resource.RUSAGE_SELF).ru_stime, "max_rss_kb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss}
    destination = Path(args.output).resolve(); destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 1 if "exception" in result else 0


if __name__ == "__main__":
    raise SystemExit(main())
