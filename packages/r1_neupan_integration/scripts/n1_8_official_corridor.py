#!/usr/bin/env python3
"""Uniform-instrumentation replay of the official NeuPAN corridor+diff demo."""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np
import torch

from n1_8_dune_attribution import Instrument, descriptor, stats


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--source", required=True); p.add_argument("--samples", type=int, default=300)
    p.add_argument("--output", required=True); p.add_argument("--threads", type=int)
    args = p.parse_args()
    if args.threads:
        torch.set_num_threads(args.threads)
    sys.path.insert(0, args.source)
    import irsim
    from neupan import neupan
    root = Path(args.source) / "example"
    env_yaml = root / "corridor/diff/env.yaml"
    planner_yaml = root / "corridor/diff/planner.yaml"
    result = {"kind": "official_corridor_diff", "source": str(Path(args.source).resolve()), "samples_requested": args.samples,
              "torch": torch.__version__, "torch_threads": torch.get_num_threads(), "runs": 0}
    dune: list[float] = []; model: list[float] = []; preparation: list[float] = []; full: list[float] = []; dune_total_per_forward: list[float] = []
    calls: list[dict] = []; calls_per_forward: list[float] = []; point_counts: list[int] = []
    while len(full) < args.samples:
        env = irsim.make(str(env_yaml), display=False)
        planner = neupan.init_from_yaml(str(planner_yaml))
        inst = Instrument(planner)
        result["descriptor"] = descriptor(planner)
        result["runs"] += 1
        for _ in range(1000):
            state = np.asarray(env.get_robot_state())
            scan = env.get_lidar_scan()
            points = planner.scan_to_point(state, scan)
            before = len(inst.dune)
            action, info, elapsed = inst.one(state, points)
            new = slice(before, len(inst.dune))
            if inst.dune[before:]:
                dune.extend(inst.dune[new]); model.extend(inst.model[new]); preparation.extend(inst.prepare[new]);
                calls.extend(inst.calls[before:]); calls_per_forward.append(float(inst.forward_call_counts[-1]));
                dune_total_per_forward.append(float(sum(inst.dune[new]))); full.append(elapsed); point_counts.append(int(points.shape[1]))
            if len(full) >= args.samples or info["arrive"] or env.done():
                break
            env.step(action)
        env.end(0)
    result.update({"timing_ms": {"dune_per_call": stats(dune), "dune_total_per_forward": stats(dune_total_per_forward), "obs_point_net_model": stats(model), "point_flow_preparation": stats(preparation), "full": stats(full),
                                         "dune_postprocess_estimate": stats([a-b for a,b in zip(dune,model)])},
                   "calls": {"calls_per_forward": stats(calls_per_forward), "samples": calls[:12]}, "raw": {"input_point_count": stats([float(x) for x in point_counts])}})
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output).write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
