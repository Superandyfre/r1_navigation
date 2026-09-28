#!/usr/bin/env python3
"""Core-only DUNE latency attribution with explicit, stable timing boundaries.

This is intentionally an experiment harness: it never modifies NeuPAN or a
planner YAML.  It wraps the instantiated objects to report the exact DUNE
call boundary, point-flow preparation and pure ObsPointNet time separately.
"""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import os
import platform
import resource
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
import torch
import yaml


def stats(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "mean_ms": None, "median_ms": None, "p95_ms": None, "max_ms": None}
    arr = np.asarray(values, dtype=float)
    return {"count": int(arr.size), "mean_ms": float(arr.mean()), "median_ms": float(np.median(arr)),
            "p95_ms": float(np.percentile(arr, 95)), "max_ms": float(arr.max())}


def ms(start_ns: int) -> float:
    return (time.perf_counter_ns() - start_ns) / 1_000_000.0


def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def env_value(name: str) -> str | None:
    return os.environ.get(name)


def cpu_observation() -> dict[str, Any]:
    def read(path: str) -> str | None:
        try:
            return Path(path).read_text().strip()
        except OSError:
            return None
    freq = {}
    for path in sorted(Path("/sys/devices/system/cpu").glob("cpu[0-9]*/cpufreq/scaling_cur_freq"))[:24]:
        value = read(str(path))
        if value is not None:
            freq[path.parts[-3]] = value
    return {
        "governor_cpu0": read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"),
        "frequency_khz": freq,
        "on_ac": read("/sys/class/power_supply/AC/online") or read("/sys/class/power_supply/ACAD/online"),
        "thermal": {p.name: read(str(p / "temp")) for p in Path("/sys/class/thermal").glob("thermal_zone*")},
    }


class Instrument:
    def __init__(self, planner: Any) -> None:
        self.planner = planner
        self.prepare: list[float] = []
        self.dune: list[float] = []
        self.model: list[float] = []
        self.calls: list[dict[str, Any]] = []
        self.forward_call_counts: list[int] = []
        self._within_forward = False
        self._calls_this_forward = 0
        self._wrap()

    def _wrap(self) -> None:
        pan = self.planner.pan
        original_prepare = pan.generate_point_flow
        original_dune = pan.dune_layer.forward
        original_model = pan.dune_layer.model.forward

        def timed_prepare(*args: Any, **kwargs: Any) -> Any:
            start = time.perf_counter_ns()
            value = original_prepare(*args, **kwargs)
            self.prepare.append(ms(start))
            return value

        def timed_model(*args: Any, **kwargs: Any) -> Any:
            start = time.perf_counter_ns()
            value = original_model(*args, **kwargs)
            self.model.append(ms(start))
            return value

        def timed_dune(point_flow: list[torch.Tensor], r_list: list[torch.Tensor], obs_list: list[torch.Tensor] | None = None) -> Any:
            start = time.perf_counter_ns()
            value = original_dune(point_flow, r_list, obs_list)
            elapsed = ms(start)
            self.dune.append(elapsed)
            self._calls_this_forward += 1
            self.calls.append({
                "call_index": len(self.calls) + 1,
                "elapsed_ms": elapsed,
                "point_flow_shapes": [list(x.shape) for x in point_flow],
                "model_input_shape": [sum(int(x.shape[1]) for x in point_flow), int(point_flow[0].shape[0])],
                "point_flow_dtype": str(point_flow[0].dtype), "point_flow_device": str(point_flow[0].device),
                "contiguous": bool(point_flow[0].is_contiguous()), "requires_grad": bool(point_flow[0].requires_grad),
                "receding_steps": len(point_flow) - 1,
                "points_per_step": int(point_flow[0].shape[1]),
            })
            return value

        pan.generate_point_flow = timed_prepare
        pan.dune_layer.model.forward = timed_model
        pan.dune_layer.forward = timed_dune

    def reset(self) -> None:
        self.prepare.clear(); self.dune.clear(); self.model.clear(); self.calls.clear(); self.forward_call_counts.clear()

    def one(self, state: np.ndarray, points: np.ndarray) -> tuple[Any, Any, float]:
        self._calls_this_forward = 0
        start = time.perf_counter_ns()
        action, info = self.planner(state, points)
        elapsed = ms(start)
        self.forward_call_counts.append(self._calls_this_forward)
        return action, info, elapsed


def set_path(planner: Any, initial_path: np.ndarray) -> None:
    path = [np.vstack([initial_path[:, i:i + 1], [[1.0]]]) for i in range(initial_path.shape[1])]
    try:
        planner.set_initial_path(path)
    except TypeError as exc:
        # NumPy 2.x preserves 1-element slices here; the official helper calls
        # math.hypot, which expects Python scalars.  This process-local shim
        # changes only scalar extraction, never the frozen path coordinates.
        if "0-dimensional arrays" not in str(exc):
            raise
        def scalar_average_interval(points: list[np.ndarray]) -> float:
            if len(points) < 2:
                return 0.0
            return float(np.mean([np.hypot(float(np.asarray(b[0:2] - a[0:2]).reshape(2)[0]),
                                           float(np.asarray(b[0:2] - a[0:2]).reshape(2)[1]))
                                  for a, b in zip(points, points[1:])]))
        planner.ipath.cal_average_interval = scalar_average_interval
        planner.set_initial_path(path)


def planner_from_config(config_path: str, checkpoint: str, device: str, threshold: float | None) -> tuple[Any, dict[str, Any]]:
    from neupan import neupan
    config = yaml.safe_load(Path(config_path).read_text()) or {}
    pan = dict(config.get("pan") or {})
    pan["dune_checkpoint"] = checkpoint
    if threshold is not None:
        pan["iter_threshold"] = threshold
    return neupan.init_from_yaml(config_path, pan=pan, device=device), pan


def descriptor(planner: Any) -> dict[str, Any]:
    dune = planner.pan.dune_layer
    parameter = next(dune.model.parameters())
    return {
        "planner": {"receding": planner.T, "iter_num": planner.pan.iter_num, "iter_threshold": planner.pan.iter_threshold,
                    "dune_max_num": planner.pan.dune_max_num, "nrmp_max_num": planner.pan.nrmp_max_num},
        "model": {"parameter_device": str(parameter.device), "parameter_dtype": str(parameter.dtype),
                  "training": bool(dune.model.training), "edge_dim": int(dune.edge_dim), "state_dim": int(dune.state_dim),
                  "checkpoint": str(getattr(dune, "abs_checkpoint_path", "")),
                  "checkpoint_sha256": sha256(str(dune.abs_checkpoint_path))},
    }


def run_snapshot(args: argparse.Namespace) -> dict[str, Any]:
    snapshot = np.load(args.snapshot)
    state = snapshot["robot_state"].astype(float)
    points = snapshot["obstacle_points"].astype(float)
    initial_path = snapshot["initial_path"].astype(float)
    planner, pan = planner_from_config(args.config, args.checkpoint, args.device, args.iter_threshold)
    set_path(planner, initial_path)
    inst = Instrument(planner)
    for _ in range(args.warmup):
        inst.one(state, points)
    inst.reset()
    full: list[float] = []
    for _ in range(args.samples):
        _, _, elapsed = inst.one(state, points)
        full.append(elapsed)
    model_per_call = inst.model
    result = {
        "kind": "frozen_snapshot", "snapshot": {"path": str(Path(args.snapshot).resolve()), "sha256": sha256(args.snapshot),
          "arrays": {k: {"shape": list(snapshot[k].shape), "dtype": str(snapshot[k].dtype)} for k in snapshot.files}},
        "config": {"path": str(Path(args.config).resolve()), "sha256": sha256(args.config), "pan_effective": pan},
        "descriptor": descriptor(planner), "input": {"state_shape": list(state.shape), "obstacle_shape": list(points.shape),
          "obstacle_points_before_selection": int(points.shape[1])},
        "timing_ms": {"full": stats(full), "point_flow_preparation": stats(inst.prepare), "dune_total": stats(inst.dune),
          "obs_point_net_model": stats(model_per_call),
          "dune_postprocess_estimate": stats([a - b for a, b in zip(inst.dune, model_per_call)])},
        "calls": {"calls_per_forward": stats([float(v) for v in inst.forward_call_counts]), "total": len(inst.calls), "samples": inst.calls[: min(12, len(inst.calls))]},
    }
    return result


def microbench_model(args: argparse.Namespace, planner: Any, input_tensor: torch.Tensor, device: str, dtype: torch.dtype) -> dict[str, Any]:
    model = planner.pan.dune_layer.model
    model.to(device=device, dtype=dtype)
    value_input = input_tensor.to(device=device, dtype=dtype)
    result: dict[str, Any] = {}
    modes = {"default": contextlib.nullcontext, "no_grad": torch.no_grad, "inference_mode": torch.inference_mode}
    for label, factory in modes.items():
        if args.micro_mode != "all" and label != args.micro_mode:
            continue
        values: list[float] = []
        for i in range(args.warmup + args.samples):
            if device == "cuda":
                torch.cuda.synchronize()
            start = time.perf_counter_ns()
            with factory():
                model(value_input)
            if device == "cuda":
                torch.cuda.synchronize()
            elapsed = ms(start)
            if i >= args.warmup:
                values.append(elapsed)
        result[label] = stats(values)
    return result


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--snapshot", required=True); p.add_argument("--config", required=True); p.add_argument("--checkpoint", required=True)
    p.add_argument("--output", required=True); p.add_argument("--device", choices=("cpu",), default="cpu")
    p.add_argument("--warmup", type=int, default=50); p.add_argument("--samples", type=int, default=500); p.add_argument("--iter-threshold", type=float, default=0.2)
    p.add_argument("--skip-snapshot", action="store_true"); p.add_argument("--skip-micro", action="store_true")
    p.add_argument("--threads", type=int, help="Process-local Torch intra-op thread count for an A/B run")
    p.add_argument("--micro-target", choices=("all", "cpu32", "cpu64", "cuda32"), default="all")
    p.add_argument("--micro-mode", choices=("all", "default", "no_grad", "inference_mode"), default="all")
    args = p.parse_args()
    if args.threads is not None:
        torch.set_num_threads(args.threads)
    result = {"method": {"core_only": True, "warmup": args.warmup, "samples": args.samples, "timing_clock": "perf_counter_ns", "device": args.device,
                          "requested_torch_threads": args.threads, "micro_target": args.micro_target, "micro_mode": args.micro_mode},
              "runtime": {"python": sys.version, "platform": platform.platform(), "torch": torch.__version__, "torch_cuda": torch.version.cuda,
                "numpy": np.__version__, "torch_threads": torch.get_num_threads(), "torch_interop_threads": torch.get_num_interop_threads(),
                "env_threads": {x: env_value(x) for x in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS")}, "cpu": cpu_observation()}}
    try:
        if not args.skip_snapshot:
            result["snapshot_run"] = run_snapshot(args)
        if not args.skip_micro:
            snapshot = np.load(args.snapshot)
            planner, _ = planner_from_config(args.config, args.checkpoint, args.device, args.iter_threshold)
            set_path(planner, snapshot["initial_path"].astype(float))
        # Build the real network input once through the normal planner path; this is pure-model only afterwards.
            state = snapshot["robot_state"].astype(float); points = snapshot["obstacle_points"].astype(float)
            nom = planner.ipath.generate_nom_ref_state(state, planner.cur_vel_array, planner.ref_speed)
            from neupan.configuration import np_to_tensor
            tensors = [np_to_tensor(x) for x in nom]
            obstacle = np_to_tensor(points)
            flows, _, _ = planner.pan.generate_point_flow(tensors[0], obstacle, None)
            model_input = torch.hstack(flows).T
            result["pure_model_microbenchmark"] = {"input": {"shape": list(model_input.shape), "dtype": str(model_input.dtype), "device": str(model_input.device),
              "contiguous": bool(model_input.is_contiguous()), "requires_grad": bool(model_input.requires_grad)}}
            if args.micro_target in ("all", "cpu32"):
                result["pure_model_microbenchmark"]["cpu_float32"] = microbench_model(args, planner, model_input, "cpu", torch.float32)
            if args.micro_target in ("all", "cpu64"):
                result["pure_model_microbenchmark"]["cpu_float64"] = microbench_model(args, planner, model_input, "cpu", torch.float64)
            if args.micro_target in ("all", "cuda32") and torch.cuda.is_available():
                result["pure_model_microbenchmark"]["cuda_float32"] = microbench_model(args, planner, model_input, "cuda", torch.float32)
    except Exception as exc:
        import traceback
        result["exception"] = {"type": type(exc).__name__, "message": str(exc), "traceback": traceback.format_exc()}
    result["process"] = {"cpu_user_s": resource.getrusage(resource.RUSAGE_SELF).ru_utime, "cpu_system_s": resource.getrusage(resource.RUSAGE_SELF).ru_stime}
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output).write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if "exception" not in result else 2


if __name__ == "__main__":
    raise SystemExit(main())
