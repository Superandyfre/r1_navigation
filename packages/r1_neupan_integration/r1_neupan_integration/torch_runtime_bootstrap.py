#!/usr/bin/env python3
"""Start a ROS Python entry point after applying an opt-in Torch policy.

This module deliberately lives in the R1 integration package rather than in
NeuPAN.  It is used as a process prefix by the N1 launch file, so the policy
is applied before ``neupan_node`` imports the NeuPAN core or instantiates any
Torch work.  A zero value means "leave Torch's default unchanged".
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import runpy
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


THREAD_ENV_NAMES = (
    "OMP_NUM_THREADS",
    "MKL_NUM_THREADS",
    "OPENBLAS_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
)


def _env_snapshot() -> dict[str, str | None]:
    return {name: os.environ.get(name) for name in THREAD_ENV_NAMES}


def _write_report(path: str, data: dict[str, Any]) -> None:
    if not path:
        return
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--torch-num-threads", type=int, default=0)
    parser.add_argument("--torch-num-interop-threads", type=int, default=0)
    parser.add_argument(
        "--thread-env-policy",
        choices=("inherit", "match_intra"),
        default="inherit",
        help="inherit records existing OMP/MKL variables; match_intra sets only this process' variables before Torch import",
    )
    parser.add_argument("--runtime-report-file", default="")
    parser.add_argument("target", help="Python console script to execute in this interpreter")
    parser.add_argument("target_args", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.torch_num_threads < 0 or args.torch_num_interop_threads < 0:
        parser.error("Torch thread values must be >= 0; zero means unchanged")
    if args.thread_env_policy == "match_intra" and args.torch_num_threads <= 0:
        parser.error("match_intra requires --torch-num-threads > 0")
    return args


def main() -> int:
    args = _parse_args()
    before_env = _env_snapshot()
    if args.thread_env_policy == "match_intra":
        requested = str(args.torch_num_threads)
        for name in THREAD_ENV_NAMES:
            os.environ[name] = requested

    # Keep this import after optional per-process environment setup.  Both
    # set_num_threads and set_num_interop_threads execute before target code.
    import torch

    report: dict[str, Any] = {
        "schema": "r1_n1_torch_runtime_policy_v1",
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "python": sys.version,
        "platform": platform.platform(),
        "torch": torch.__version__,
        "requested": {
            "torch_num_threads": args.torch_num_threads or None,
            "torch_num_interop_threads": args.torch_num_interop_threads or None,
            "thread_env_policy": args.thread_env_policy,
        },
        "environment_before": before_env,
        "environment_effective": _env_snapshot(),
    }
    try:
        if args.torch_num_threads:
            torch.set_num_threads(args.torch_num_threads)
        if args.torch_num_interop_threads:
            torch.set_num_interop_threads(args.torch_num_interop_threads)
        report["effective"] = {
            "torch_num_threads": torch.get_num_threads(),
            "torch_num_interop_threads": torch.get_num_interop_threads(),
        }
        _write_report(args.runtime_report_file, report)
    except Exception as exc:
        report["error"] = {"type": type(exc).__name__, "message": str(exc)}
        _write_report(args.runtime_report_file, report)
        raise

    # NumPy/SciPy/CVXPY are loaded by the target.  Capture their effective
    # OpenMP/BLAS pools after startup so a benchmark records more than merely
    # requested environment variables.  This does not alter any pool.
    if args.runtime_report_file:
        def capture_late_threadpools() -> None:
            time.sleep(10.0)
            try:
                from threadpoolctl import threadpool_info
                report["threadpools_after_target_import"] = threadpool_info()
            except Exception as exc:  # diagnostic-only; target must continue
                report["threadpools_after_target_import_error"] = (
                    f"{type(exc).__name__}: {exc}"
                )
            _write_report(args.runtime_report_file, report)
        threading.Thread(target=capture_late_threadpools, daemon=True).start()

    # Do not exec a fresh interpreter: its thread-pool configuration would be
    # lost.  Running the generated ROS console script in this interpreter
    # preserves the configured runtime while retaining normal ROS argv.
    sys.argv = [args.target, *args.target_args]
    runpy.run_path(args.target, run_name="__main__")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
