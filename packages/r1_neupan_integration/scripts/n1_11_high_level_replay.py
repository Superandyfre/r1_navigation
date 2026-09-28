#!/usr/bin/env python3
"""Recanonicalize one N1.11 high-level snapshot in a fresh process."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "r1_neupan_integration" / "scripts"))

from n1_9_core_benchmark import set_path  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--snapshot", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    from neupan import neupan
    from neupan.configuration import np_to_tensor
    from r1_neupan_integration import n1_11_diagnostics

    snapshot_path = Path(args.snapshot).resolve()
    snapshot = np.load(snapshot_path, allow_pickle=False)
    raw_metadata = snapshot.get("metadata_json")
    metadata = json.loads(str(raw_metadata.item() if hasattr(raw_metadata, "item") else raw_metadata))
    cycle_id = int(metadata["cycle_id"])
    if "pan_current_nom_s" not in snapshot.files:
        raise RuntimeError("snapshot has no PAN pre-cycle state")

    # init_from_yaml replaces, rather than deep-merges, the supplied ``pan``
    # mapping.  Preserve the fixed N1.10 PAN values explicitly so replay uses
    # the same problem dimensions as the online launch.
    planner = neupan.init_from_yaml(
        args.config,
        pan={
            "iter_num": 2,
            "dune_max_num": 200,
            "nrmp_max_num": 10,
            "iter_threshold": 0.1,
            "dune_checkpoint": args.checkpoint,
        },
        device="cpu",
    )
    set_path(planner, snapshot["initial_path"].astype(float))
    planner.pan.current_nom_values = [
        np_to_tensor(snapshot["pan_current_nom_s"].astype(float)),
        np_to_tensor(snapshot["pan_current_nom_u"].astype(float)),
        [np_to_tensor(snapshot["pan_current_nom_mu"].astype(float))],
        [np_to_tensor(snapshot["pan_current_nom_lam"].astype(float))],
    ]
    if "robot_velocity" in snapshot.files:
        planner.cur_vel_array = snapshot["robot_velocity"].astype(float).copy()

    status = n1_11_diagnostics.install(args.output_dir)
    layer = planner.pan.nrmp_layer.nrmp_layer
    original = layer.forward

    def wrapped(*params, **kwargs):
        n1_11_diagnostics.layer_forward_start(
            layer, params, kwargs.get("solver_args")
        )
        try:
            return original(*params, **kwargs)
        finally:
            n1_11_diagnostics.layer_forward_end()

    layer.forward = wrapped
    n1_11_diagnostics.begin_cycle(cycle_id)
    try:
        planner(snapshot["robot_state"].astype(float), snapshot["obstacle_points"].astype(float))
    finally:
        record = n1_11_diagnostics.end_cycle()
    written = n1_11_diagnostics.write_cycle(record, args.output_dir)
    print(json.dumps({
        "cycle_id": cycle_id,
        "status": status,
        "layers": len(record.get("layer_events", [])),
        "solver_calls": len(record.get("ecos_events", [])),
        "written": written,
    }, indent=2, sort_keys=True, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
