#!/usr/bin/env python3
"""Reproducible, core-only N1.7 ECOS/Moreau benchmark on saved inputs."""

from __future__ import annotations

import argparse
import json
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


def percentile(values: list[float], q: float) -> float | None:
    return float(np.percentile(np.asarray(values, dtype=float), q)) if values else None


def summary(values: list[float]) -> dict[str, Any]:
    if not values:
        return {"count": 0, "mean_ms": None, "median_ms": None, "p95_ms": None, "max_ms": None}
    return {
        "count": len(values),
        "mean_ms": float(np.mean(values)),
        "median_ms": float(np.median(values)),
        "p95_ms": percentile(values, 95),
        "max_ms": float(np.max(values)),
    }


def sync(device: str) -> None:
    if device == "cuda":
        torch.cuda.synchronize()


def wrap_timed(module: Any, device: str, samples: list[float]) -> None:
    original = module.forward

    def timed(*args: Any, **kwargs: Any) -> Any:
        sync(device)
        started = time.perf_counter_ns()
        result = original(*args, **kwargs)
        sync(device)
        samples.append((time.perf_counter_ns() - started) / 1_000_000.0)
        return result

    module.forward = timed


def load_planner(args: argparse.Namespace):
    if args.neupan_source:
        sys.path.insert(0, args.neupan_source)
    if args.solver in ("moreau", "cuclarabel"):
        from r1_neupan_integration.n1_7_gpu_adapter import (
            install_cuclarabel_adapter,
            install_moreau_adapter,
        )

        if args.solver == "moreau":
            install_moreau_adapter()
        else:
            install_cuclarabel_adapter()
    from neupan import neupan

    with open(args.config, encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    pan = dict(config.get("pan") or {})
    pan["dune_checkpoint"] = args.checkpoint
    if args.iter_threshold is not None:
        pan["iter_threshold"] = args.iter_threshold
    # ``init_from_yaml`` merges kwargs in memory.  The stable config file is
    # never changed by the experiment.
    planner = neupan.init_from_yaml(args.config, pan=pan, device=args.device)
    return planner, pan


def set_path(planner: Any, initial_path: np.ndarray) -> None:
    path = [
        np.vstack([initial_path[:, index : index + 1], [[1.0]]])
        for index in range(initial_path.shape[1])
    ]
    try:
        planner.set_initial_path(path)
    except TypeError as exc:
        # The CuClarabel import path enables JAX x64 before the frozen
        # InitialPath implementation evaluates its 1-element NumPy slices.
        # Convert exactly those scalar distances without changing the path or
        # path geometry.  This process-local compatibility shim is inactive on
        # the ordinary CPU/Moreau paths.
        if "0-dimensional arrays" not in str(exc):
            raise

        def scalar_average_interval(points: list[np.ndarray]) -> float:
            if len(points) < 2:
                return 0.0
            values = []
            for point1, point2 in zip(points, points[1:]):
                delta = np.asarray(point2[0:2] - point1[0:2], dtype=float).reshape(2)
                values.append(float(np.hypot(delta[0], delta[1])))
            return float(np.mean(values))

        planner.ipath.cal_average_interval = scalar_average_interval
        planner.set_initial_path(path)


def serialise_output(action: Any, info: dict[str, Any]) -> dict[str, Any]:
    def array(value: Any) -> list[Any] | None:
        if value is None:
            return None
        if hasattr(value, "detach"):
            value = value.detach().cpu().numpy()
        return np.asarray(value).tolist()

    return {
        "action": array(action),
        "state": array(info.get("state_tensor")),
        "velocity": array(info.get("vel_tensor")),
        "distance": array(info.get("distance_tensor")),
        "stop": bool(info.get("stop", False)),
        "arrive": bool(info.get("arrive", False)),
        "collision": bool(info.get("collision", False)),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--snapshot", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--solver", choices=("ecos", "moreau", "cuclarabel"), required=True)
    parser.add_argument("--device", choices=("cpu", "cuda"), required=True)
    parser.add_argument("--warmup", type=int, default=50)
    parser.add_argument("--samples", type=int, default=300)
    parser.add_argument("--iter-threshold", type=float)
    parser.add_argument("--neupan-source", default="")
    args = parser.parse_args()

    snapshot = np.load(args.snapshot)
    state = snapshot["robot_state"].astype(float)
    points = snapshot["obstacle_points"].astype(float)
    initial_path = snapshot["initial_path"].astype(float)
    result: dict[str, Any] = {
        "method": {
            "scope": "core-only",
            "solver": args.solver,
            "device": args.device,
            "warmup": args.warmup,
            "samples": args.samples,
            "cuda_synchronized_timing": args.device == "cuda",
        },
        "snapshot": os.path.abspath(args.snapshot),
        "input": {
            "state_shape": list(state.shape),
            "obstacle_points": int(points.shape[1]),
            "initial_path_points": int(initial_path.shape[1]),
        },
        "success": False,
        "failures": [],
    }
    if args.device == "cuda":
        result["torch_cuda"] = {
            "available": torch.cuda.is_available(),
            "device_name": torch.cuda.get_device_name(0) if torch.cuda.is_available() else None,
            "capability": list(torch.cuda.get_device_capability(0)) if torch.cuda.is_available() else None,
        }
        torch.cuda.reset_peak_memory_stats()

    dune_ms: list[float] = []
    nrmp_ms: list[float] = []
    full_ms: list[float] = []
    outputs: list[dict[str, Any]] = []
    try:
        planner, pan = load_planner(args)
        set_path(planner, initial_path)
        if planner.pan.dune_layer is not None:
            wrap_timed(planner.pan.dune_layer, args.device, dune_ms)
        wrap_timed(planner.pan.nrmp_layer, args.device, nrmp_ms)
        for _ in range(args.warmup):
            planner(state, points)
        dune_ms.clear()
        nrmp_ms.clear()
        for _ in range(args.samples):
            sync(args.device)
            started = time.perf_counter_ns()
            action, info = planner(state, points)
            sync(args.device)
            full_ms.append((time.perf_counter_ns() - started) / 1_000_000.0)
            payload = serialise_output(action, info)
            flattened = np.asarray(payload["action"], dtype=float)
            if not np.isfinite(flattened).all():
                result["failures"].append("non-finite action")
            outputs.append(payload)
        result.update(
            {
                "success": not result["failures"],
                "planner_config": {"pan": pan},
                "timing_ms": {"dune": summary(dune_ms), "nrmp": summary(nrmp_ms), "full": summary(full_ms)},
                "raw_ms": {"dune": dune_ms, "nrmp": nrmp_ms, "full": full_ms},
                "last_output": outputs[-1] if outputs else None,
                "output_statistics": {
                    "linear_x": summary([float(item["action"][0][0]) for item in outputs]),
                    "angular_z": summary([float(item["action"][1][0]) for item in outputs]),
                },
            }
        )
    except Exception as exc:  # The raw exception is a benchmark result, not hidden.
        result["exception"] = {"type": type(exc).__name__, "message": str(exc), "traceback": traceback.format_exc()}
        result["failures"].append(type(exc).__name__)
    finally:
        usage = resource.getrusage(resource.RUSAGE_SELF)
        result["process"] = {"cpu_user_s": usage.ru_utime, "cpu_system_s": usage.ru_stime, "max_rss_kib": usage.ru_maxrss}
        if args.device == "cuda" and torch.cuda.is_available():
            result["gpu_memory"] = {
                "allocated_bytes": int(torch.cuda.memory_allocated()),
                "reserved_bytes": int(torch.cuda.memory_reserved()),
                "peak_allocated_bytes": int(torch.cuda.max_memory_allocated()),
                "peak_reserved_bytes": int(torch.cuda.max_memory_reserved()),
            }
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["success"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
