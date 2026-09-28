#!/usr/bin/env python3
"""Replay one exact N1.11 ECOS binding dump without ROS or CVXPY."""

from __future__ import annotations

import argparse
import hashlib
import json
import resource
import time
from pathlib import Path

import ecos
import numpy as np
import scipy.sparse as sp


def sha256_array(value: np.ndarray) -> str:
    array = np.ascontiguousarray(value)
    digest = hashlib.sha256()
    digest.update(str(array.dtype).encode())
    digest.update(np.asarray(array.shape, dtype=np.int64).tobytes())
    digest.update(array.tobytes())
    return digest.hexdigest()


def load_sparse(data: dict[str, np.ndarray], prefix: str) -> sp.csc_matrix | None:
    data_key = f"{prefix}_data"
    if data_key not in data:
        return None
    return sp.csc_matrix(
        (data[data_key], data[f"{prefix}_indices"], data[f"{prefix}_indptr"]),
        shape=tuple(json.loads(data["shape_json"].item())[prefix]),
    )


def sparse_from_meta(data: dict[str, np.ndarray], prefix: str, metadata: dict) -> sp.csc_matrix | None:
    if f"{prefix}_data" not in data:
        return None
    shape = tuple(metadata["array_meta"][prefix]["shape"])
    return sp.csc_matrix(
        (data[f"{prefix}_data"], data[f"{prefix}_indices"], data[f"{prefix}_indptr"]),
        shape=shape,
    )


def solve_once(problem: dict[str, object], mode: str) -> dict[str, object]:
    raw = np.load(problem["dump"], allow_pickle=False)
    metadata = problem["metadata"]
    c = raw["c"]
    h = raw["h"]
    b = raw["b"] if "b" in raw.files else None
    G = sparse_from_meta(raw, "G", metadata)
    A = sparse_from_meta(raw, "A", metadata)
    dims = metadata["cone_dimensions"]
    kwargs = dict(metadata.get("solver_kwargs") or {})
    if mode == "fresh":
        c = np.array(c, copy=True)
        h = np.array(h, copy=True)
        b = np.array(b, copy=True) if b is not None else None
        G = G.copy() if G is not None else None
        A = A.copy() if A is not None else None
    started_wall = time.perf_counter_ns()
    started_cpu = time.process_time_ns()
    status = "return-success"
    result = None
    error = None
    try:
        result = ecos.solve(c, G, h, dims, A, b, **kwargs)
    except Exception as exc:  # pragma: no cover - evidence path
        status = "exception"
        error = f"{type(exc).__name__}: {exc}"
    ended_wall = time.perf_counter_ns()
    ended_cpu = time.process_time_ns()
    output = {
        "status": status,
        "error": error,
        "wall_ms": (ended_wall - started_wall) / 1_000_000.0,
        "cpu_ms": (ended_cpu - started_cpu) / 1_000_000.0,
    }
    if isinstance(result, dict):
        info = result.get("info", {})
        output["info"] = {
            key: value for key, value in info.items()
            if isinstance(value, (str, int, float, bool)) or value is None
        }
        output["solution_hashes"] = {
            key: sha256_array(np.asarray(result[key]))
            for key in ("x", "y", "z") if key in result
        }
        output["objective"] = info.get("pcost")
        output["iterations"] = info.get("iter")
        output["exit_flag"] = info.get("exitFlag")
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dump", required=True)
    parser.add_argument("--metadata", required=True)
    parser.add_argument("--mode", choices=("fresh", "reuse"), required=True)
    parser.add_argument("--warmup", type=int, default=30)
    parser.add_argument("--samples", type=int, default=300)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    cycle_metadata = json.loads(Path(args.metadata).read_text(encoding="utf-8"))
    dump_name = Path(args.dump).name
    metadata = next(
        item for item in cycle_metadata.get("solver_files", [])
        if item.get("file") == dump_name
    )
    problem = {"dump": args.dump, "metadata": metadata}
    warmup = [solve_once(problem, args.mode) for _ in range(args.warmup)]
    measured = [solve_once(problem, args.mode) for _ in range(args.samples)]

    def summary(rows: list[dict[str, object]], key: str) -> dict[str, object]:
        values = np.asarray([float(row[key]) for row in rows if row[key] is not None], dtype=float)
        return {
            "count": int(values.size),
            "mean_ms": float(values.mean()) if values.size else None,
            "median_ms": float(np.median(values)) if values.size else None,
            "p95_ms": float(np.percentile(values, 95)) if values.size else None,
            "max_ms": float(values.max()) if values.size else None,
        }

    result = {
        "schema": "r1_n1_11_direct_ecos_replay_v1",
        "method": {"mode": args.mode, "warmup": args.warmup, "samples": args.samples, "independent_ros_process": True},
        "dump": str(Path(args.dump).resolve()),
        "metadata": str(Path(args.metadata).resolve()),
        "canonical_problem_hash": metadata.get("canonical_problem_hash"),
        "warmup": warmup[:5],
        "timing_ms": {
            "wall": summary(measured, "wall_ms"),
            "cpu": summary(measured, "cpu_ms"),
        },
        "validation": {
            "measured": len(measured),
            "success": sum(row["status"] == "return-success" for row in measured),
            "failure": sum(row["status"] != "return-success" for row in measured),
            "iterations": summary(measured, "iterations"),
            "exit_flags": sorted({row.get("exit_flag") for row in measured}),
            "objectives": sorted({row.get("objective") for row in measured if row.get("objective") is not None})[:5],
        },
        "process": {
            "cpu_user_sec": resource.getrusage(resource.RUSAGE_SELF).ru_utime,
            "cpu_system_sec": resource.getrusage(resource.RUSAGE_SELF).ru_stime,
            "max_rss_kb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        },
    }
    destination = Path(args.output).resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 1 if result["validation"]["failure"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
