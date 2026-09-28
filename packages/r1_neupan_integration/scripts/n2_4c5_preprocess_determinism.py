#!/usr/bin/env python3
"""N2.4C5-3: offline preprocess determinism for the frozen D1-4 capture.

Replays the same frozen (state, obstacle_points, path, pre-state) tuples
through the whole-core worker ``PASS_COUNT`` times, each pass in a freshly
spawned worker process, and requires the recomputed DUNE/NRMP obstacle
tensors (``planner_state_post``) to be digest-identical across every pass.
Restoring the recorded pre-state makes each call independent of the calls
before it, so any digest drift is real preprocess nondeterminism, not
cycle-order state.

Run with the neupan venv python and PYTHONPATH=src/r1_neupan_integration.
"""

from __future__ import annotations

import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "r1_neupan_integration"))
sys.path.insert(0, str(ROOT / "src" / "r1_neupan_integration" / "scripts"))

from n2_4c3_reproducibility_attribution import decode_capture  # noqa: E402
from r1_neupan_integration.n1_15_whole_core import WholeCoreClient  # noqa: E402

SOURCE_DIR = ROOT / "n2_4c2_phenotype_runs" / "d1_04"
OUTPUT_DIR = ROOT / "n2_4c5_results" / "preprocess_determinism"
CONFIG = ROOT / "src/r1_neupan_integration/config/planner.yaml"
CHECKPOINT = ROOT / "src/r1_neupan_integration/config/models/dune_model_5000.pth"
PASS_COUNT = 20
CYCLE_SLOTS = (0.0, 0.2, 0.4, 0.6, 0.8, 1.0)


def digest_array(value: Any = None, array: np.ndarray | None = None) -> str:  # noqa: ANN401
    data = np.ascontiguousarray(array if array is not None else value)
    return hashlib.sha256(data.tobytes() + str(data.dtype).encode() + str(data.shape).encode()).hexdigest()[:16]


def select_cycles(by_cycle: dict[int, dict]) -> list[int]:
    eligible = sorted(
        cycle
        for cycle, rec in by_cycle.items()
        if rec.get("input") and rec.get("planner_state_pre") and rec.get("request")
        and rec["input"].get("obstacle_points") is not None
        and rec["planner_state_pre"].get("dune", {}).get("obstacle_points") is not None
    )
    if not eligible:
        raise RuntimeError("no eligible frozen records in source capture")
    picked: list[int] = []
    for slot in CYCLE_SLOTS:
        idx = min(len(eligible) - 1, int(round(slot * (len(eligible) - 1))))
        if eligible[idx] not in picked:
            picked.append(eligible[idx])
    return picked


def run_pass(pass_index: int, capture: dict, cycles: list[int]) -> dict:
    identity = capture["manifest"]["config_identity"]
    pan = dict(identity["effective_yaml"]["pan"])
    pan["dune_checkpoint"] = str(CHECKPOINT)
    work = OUTPUT_DIR / f"pass_{pass_index:02d}"
    client = WholeCoreClient(
        str(work / "worker"), str(CONFIG), str(CHECKPOINT), pan,
        receding=int(identity["pan_runtime"]["receding"]), torch_intraop=1,
        torch_interop=24, warmup_count=0, timeout_sec=30.0,
        exact_capture_dir=str(work / "capture"), exact_capture_max_records=64,
    )
    rows = []
    try:
        for cycle in cycles:
            rec = capture["by_cycle"][cycle]
            request = rec["request"]
            outputs, _row = client.solve(
                np.asarray(rec["input"]["state"], dtype=np.float64),
                np.asarray(rec["input"]["obstacle_points"], dtype=np.float64),
                np.asarray(rec["input"]["effective_path"], dtype=np.float64),
                int(request["path_version"]), int(request["scan_counter"]),
                float(request["scan_stamp_sec"]), goal=None,
                goal_version=int(request["goal_version"]), cycle_id=int(request["cycle_id"]),
                navigation_loop_mode="terminal", active_gear=int(request["active_gear"]),
                restore_planner_state=rec["planner_state_pre"],
            )
            rows.append({
                "cycle": cycle,
                "action_digest": digest_array(array=np.asarray(outputs["action"], dtype=float)),
            })
        dumped = client.dump_exact_capture(metadata={"pass": pass_index, "purpose": "n2_4c5_3_preprocess_determinism"})
    finally:
        client.close()
    payload_path = work / "capture" / "numeric_payload.npz"
    if not payload_path.exists() or payload_path.stat().st_size == 0:
        raise RuntimeError(f"capture payload missing on pass {pass_index}: dump={dumped}")
    with np.load(payload_path, allow_pickle=False) as payload:
        tensors = {}
        for key in payload.files:
            if ".input.obstacle_points" in key or ".planner_state_post.dune.obstacle_points" in key \
                    or ".planner_state_post.nrmp.obstacle_points" in key \
                    or ".planner_state_pre.dune.obstacle_points" in key \
                    or ".planner_state_pre.nrmp.obstacle_points" in key:
                tensors[key] = digest_array(array=payload[key])
    return {"pass": pass_index, "rows": rows, "tensor_digests": tensors}


def main() -> int:
    started = time.time()
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    capture = decode_capture(SOURCE_DIR)
    cycles = select_cycles(capture["by_cycle"])
    print(f"[setup] source={SOURCE_DIR} cycles={cycles} passes={PASS_COUNT}", flush=True)

    passes = [run_pass(i + 1, capture, cycles) for i in range(PASS_COUNT)]

    report_rows = []
    all_match = True
    for position, cycle in enumerate(cycles):
        per_pass = []
        for entry in passes:
            digests = entry["tensor_digests"]
            row = {
                "post_dune": digests.get(f"record[{position}].planner_state_post.dune.obstacle_points", "MISSING"),
                "post_nrmp": digests.get(f"record[{position}].planner_state_post.nrmp.obstacle_points", "MISSING"),
                "pre_dune": digests.get(f"record[{position}].planner_state_pre.dune.obstacle_points", "MISSING"),
                "pre_nrmp": digests.get(f"record[{position}].planner_state_pre.nrmp.obstacle_points", "MISSING"),
                "input": digests.get(f"record[{position}].input.obstacle_points", "MISSING"),
            }
            per_pass.append(row)
        reference = per_pass[0]
        stable = all(row == reference for row in per_pass)
        action_stable = len({tuple(sorted(p["rows"][position].items())) for p in passes}) == 1
        all_match = all_match and stable and action_stable
        report_rows.append({
            "cycle": cycle,
            "digest_stable_across_passes": stable,
            "action_digest_stable": action_stable,
            "reference_digests": reference,
            "distinct_digest_sets": len({json.dumps(row, sort_keys=True) for row in per_pass}),
        })
        print(f"[cycle {cycle}] stable={stable} distinct={report_rows[-1]['distinct_digest_sets']}", flush=True)

    summary = {
        "task": "N2.4C5-3 preprocess determinism",
        "source": str(SOURCE_DIR),
        "passes": PASS_COUNT,
        "cycles": cycles,
        "fresh_worker_per_pass": True,
        "restore_planner_state": True,
        "torch_intraop": 1,
        "all_pass": bool(all_match),
        "rows": report_rows,
        "duration_sec": round(time.time() - started, 1),
    }
    (OUTPUT_DIR / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True))
    print(f"[result] all_pass={all_match} duration={summary['duration_sec']}s", flush=True)
    return 0 if all_match else 1


if __name__ == "__main__":
    sys.exit(main())
