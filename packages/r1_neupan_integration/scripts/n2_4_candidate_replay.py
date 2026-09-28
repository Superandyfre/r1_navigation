#!/usr/bin/env python3
"""Candidate-only N2.4 planner replay with auditable output boundaries.

This entry point reuses the existing isolated ``WholeCoreClient`` and the
existing R1 safety-gate core.  It does not publish ROS commands, start Gazebo,
or change NeuPAN parameters.  Captured N1.11 tensors provide the planner
inputs; the N2.4 shadow JSON provides the candidate reanchor geometry.

The integrated trajectory is a downstream planar kinematic trace of gated
commands.  It is evidence for this candidate replay only, not a Gazebo
physics or recovery-success claim.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from r1_neupan_integration.n1_15_whole_core import WholeCoreClient, _json_safe
from r1_neupan_integration.n2_pre_safety_gate import SafetyGateCore


PRIMARY_SCENES = ("b1_single_table", "b4_center_obstacle", "d1_corner_obstacle")
DEFAULT_SNAPSHOT_DIR = Path("n1_results/n1_11_online_obstacle_v2/canonical")
DEFAULT_SHADOW_JSON = Path("n2_4_results/n2_4_shadow.json")
DEFAULT_CONFIG = Path("src/r1_neupan_integration/config/planner.yaml")
DEFAULT_CHECKPOINT = Path("src/r1_neupan_integration/config/models/dune_model_5000.pth")
DEFAULT_OUTPUT_DIR = Path("n2_4_results/n2_4_candidate_replay")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def require_file(path: Path, label: str) -> Path:
    path = path.expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"{label} does not exist: {path}")
    return path


def number(value: Any, default: float | None = None) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def load_snapshots(directory: Path, count: int) -> tuple[list[dict[str, Any]], list[Path]]:
    directory = directory.expanduser().resolve()
    paths = sorted(directory.glob("*_high_level.npz"))
    if len(paths) < count:
        raise RuntimeError(
            f"need at least {count} high-level snapshots in {directory}, found {len(paths)}"
        )
    snapshots: list[dict[str, Any]] = []
    selected = paths[:count]
    for path in selected:
        with np.load(path, allow_pickle=False) as archive:
            required = ("robot_state", "obstacle_points", "initial_path")
            missing = [key for key in required if key not in archive.files]
            if missing:
                raise ValueError(f"snapshot {path} is missing {missing}")
            snapshot = {
                "robot_state": np.asarray(archive["robot_state"], dtype=np.float64).copy(),
                "obstacle_points": np.asarray(archive["obstacle_points"], dtype=np.float64).copy(),
                "initial_path": np.asarray(archive["initial_path"], dtype=np.float64).copy(),
            }
            metadata = {}
            if "metadata_json" in archive.files:
                raw = archive["metadata_json"]
                raw = raw.item() if hasattr(raw, "item") else raw
                try:
                    metadata = json.loads(str(raw))
                except (TypeError, ValueError, json.JSONDecodeError):
                    metadata = {}
            snapshot["metadata"] = metadata
        if snapshot["robot_state"].shape != (3, 1):
            raise ValueError(f"snapshot {path} robot_state shape is {snapshot['robot_state'].shape}")
        if snapshot["obstacle_points"].ndim != 2 or snapshot["obstacle_points"].shape[0] != 2:
            raise ValueError(
                f"snapshot {path} obstacle_points shape is {snapshot['obstacle_points'].shape}"
            )
        if snapshot["initial_path"].ndim != 2 or snapshot["initial_path"].shape[0] not in (3, 4):
            raise ValueError(f"snapshot {path} initial_path shape is {snapshot['initial_path'].shape}")
        if not all(np.isfinite(snapshot[key]).all() for key in ("robot_state", "obstacle_points", "initial_path")):
            raise ValueError(f"snapshot {path} contains NaN or Inf")
        snapshots.append(snapshot)
    baseline = snapshots[0]["initial_path"]
    if any(not np.array_equal(baseline, item["initial_path"]) for item in snapshots[1:]):
        raise RuntimeError("selected snapshots do not share one initial path")
    return snapshots, selected


def path_matrix(points: Any, gear: int = 1) -> np.ndarray:
    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 2 or values.shape[0] < 2:
        raise ValueError(f"candidate path must be an Nx2 matrix with N>=2, got {values.shape}")
    if not np.isfinite(values).all():
        raise ValueError("candidate path contains NaN or Inf")
    headings = []
    for index in range(len(values)):
        if index + 1 < len(values):
            delta = values[index + 1] - values[index]
        else:
            delta = values[index] - values[index - 1]
        if np.linalg.norm(delta) <= 1.0e-12:
            delta = values[index - 1] - values[index - 2] if index >= 2 else np.asarray((1.0, 0.0))
        headings.append(math.atan2(float(delta[1]), float(delta[0])))
    return np.vstack((values.T, np.asarray(headings), np.full(len(values), int(gear), dtype=np.float64))).astype(np.float64)


def candidate_from_shadow(shadow: dict[str, Any], scene: str) -> tuple[np.ndarray, dict[str, Any]]:
    scene_data = shadow.get("scenes", {}).get(scene)
    if not isinstance(scene_data, dict):
        raise ValueError(f"shadow JSON has no scene {scene!r}")
    reanchor = scene_data.get("reanchor") or {}
    rebuilt = reanchor.get("rebuilt_path")
    if not isinstance(rebuilt, list) or len(rebuilt) < 2:
        raise ValueError(f"scene {scene} has no usable reanchor path")
    candidate = path_matrix(rebuilt)
    evidence = {
        "scene": scene,
        "triggered": bool(scene_data.get("triggered")),
        "trigger_cycle": (scene_data.get("timeline") or {}).get("trigger_cycle"),
        "reanchor": reanchor,
        "heading_selection": scene_data.get("heading_selection"),
        "source_classification": shadow.get("classification"),
    }
    return candidate, evidence


def metadata_scan_stamp(snapshot: dict[str, Any], cycle_id: int) -> float:
    metadata = snapshot.get("metadata") or {}
    for key in ("scan_stamp_sec", "scan_stamp", "stamp_sec"):
        value = number(metadata.get(key))
        if value is not None:
            return value
    return float(cycle_id)


def pose_from_state(state: np.ndarray) -> list[float]:
    return [float(state[index, 0]) for index in range(3)]


def integrate_pose(pose: list[float], linear: float, angular: float, dt: float) -> list[float]:
    x, y, yaw = pose
    return [
        x + float(linear) * math.cos(yaw) * dt,
        y + float(linear) * math.sin(yaw) * dt,
        math.atan2(math.sin(yaw + float(angular) * dt), math.cos(yaw + float(angular) * dt)),
    ]


def decision_json(decision: Any) -> dict[str, Any]:
    return {
        "linear_x": float(decision.linear),
        "angular_z": float(decision.angular),
        "reasons": list(decision.reasons),
        "clamped": bool(decision.clamped),
        "zeroed": bool(decision.zeroed),
        "ages": {str(key): float(value) for key, value in decision.ages.items()},
        "negative_input": bool(decision.negative_input),
        "negative_clamped": bool(decision.negative_clamped),
    }


def replay_scene(
    *,
    scene: str,
    snapshots: list[dict[str, Any]],
    snapshot_paths: list[Path],
    shadow: dict[str, Any],
    shadow_path: Path,
    config_path: Path,
    checkpoint_path: Path,
    output_root: Path,
    pan: dict[str, Any],
    receding: int | None,
    warmup_count: int,
    torch_intraop: int,
    torch_interop: int,
    baseline_cycles: int,
    dt: float,
    navigation_mode: str,
) -> dict[str, Any]:
    candidate, shadow_evidence = candidate_from_shadow(shadow, scene)
    scene_root = output_root / scene
    worker_root = scene_root / "worker"
    scene_root.mkdir(parents=True, exist_ok=True)
    baseline = snapshots[0]["initial_path"].copy()
    if baseline.shape[0] == 3:
        baseline = np.vstack((baseline, np.ones((1, baseline.shape[1]), dtype=np.float64)))
    baseline = np.ascontiguousarray(baseline)

    client = WholeCoreClient(
        str(worker_root), str(config_path), str(checkpoint_path), pan, receding,
        torch_intraop, torch_interop, warmup_count,
    )
    gate = SafetyGateCore()
    gate.release()
    # Establish the gate clock before the first formal replay cycle.
    gate.step(
        0.0, 0.0, 0.0,
        command_seen=0.0, scan_seen=0.0, odom_seen=0.0,
        tf_seen=0.0, timing_seen=0.0, planner_result_seen=0.0,
    )
    pose = pose_from_state(snapshots[0]["robot_state"])
    trajectory: list[dict[str, Any]] = []
    path_version_trace: list[dict[str, Any]] = []
    replay_error: dict[str, str] | None = None
    try:
        for index, snapshot in enumerate(snapshots):
            cycle_id = index + 1
            use_candidate = index >= baseline_cycles
            path = candidate if use_candidate else baseline
            path_version = 2 if use_candidate else 1
            if not path_version_trace or path_version_trace[-1]["path_version"] != path_version:
                path_version_trace.append({
                    "cycle_id": cycle_id,
                    "path_version": path_version,
                    "kind": "candidate_reanchor" if use_candidate else "captured_baseline",
                    "path_shape": list(path.shape),
                })
            state = snapshot["robot_state"]
            points = snapshot["obstacle_points"]
            outputs, record = client.solve(
                state, points, path, path_version,
                scan_counter=cycle_id,
                scan_stamp_sec=metadata_scan_stamp(snapshot, cycle_id),
                cycle_id=cycle_id,
                navigation_loop_mode=navigation_mode,
                active_gear=1,
            )
            action = np.asarray(outputs.get("action", np.empty((0,), dtype=np.float64))).reshape(-1)
            if action.size < 2 or not np.isfinite(action[:2]).all():
                raise RuntimeError(f"worker returned invalid action at cycle {cycle_id}: {action.tolist()}")
            raw_linear, raw_angular = float(action[0]), float(action[1])
            now = float(cycle_id) * dt
            decision = gate.step(
                now, raw_linear, raw_angular,
                command_seen=now, scan_seen=now, odom_seen=now,
                tf_seen=now, timing_seen=now, planner_result_seen=now,
            )
            before = list(pose)
            pose = integrate_pose(pose, decision.linear, decision.angular, dt)
            trajectory.append({
                "cycle_id": cycle_id,
                "source_snapshot": str(snapshot_paths[index].resolve()),
                "source_snapshot_sha256": sha256(snapshot_paths[index]),
                "mode": "candidate_reanchor" if use_candidate else "captured_baseline",
                "path_version": path_version,
                "raw_action": {"linear_x": raw_linear, "angular_z": raw_angular},
                "gated_action": decision_json(decision),
                "pose_before": before,
                "pose_after": pose,
                "worker": {
                    "request_id": record.get("request_id"),
                    "worker_sequence": record.get("worker_sequence"),
                    "status": record.get("status"),
                    "finite": bool(record.get("finite")),
                    "info": record.get("info", {}),
                    "output_shapes": record.get("outputs", {}),
                },
            })
    except Exception as exc:
        replay_error = {"type": type(exc).__name__, "message": str(exc)}
    finally:
        client.finalize()

    worker_summary_path = worker_root / "summary.json"
    if not worker_summary_path.is_file():
        raise RuntimeError(f"WholeCoreClient did not write worker summary: {worker_summary_path}")
    worker_summary = json.loads(worker_summary_path.read_text(encoding="utf-8"))
    worker_summary_payload = {
        "schema": "r1_n2_4_candidate_worker_summary_v1",
        "candidate_only": True,
        "scene": scene,
        "worker_summary_path": str(worker_summary_path.resolve()),
        "whole_core_summary": worker_summary,
    }
    (scene_root / "worker_summary.json").write_text(
        json.dumps(_json_safe(worker_summary_payload), indent=2, sort_keys=True), encoding="utf-8"
    )

    gate_clamp_count = sum(1 for row in trajectory if row["gated_action"]["negative_clamped"])
    trajectory_payload = {
        "schema": "r1_n2_4_candidate_trajectory_v1",
        "candidate_only": True,
        "scene": scene,
        "model": "planar kinematic integration of SafetyGateCore output",
        "dynamics_integrated": False,
        "gazebo_executed": False,
        "ros_commands_published": False,
        "dt_sec": dt,
        "baseline_cycles": baseline_cycles,
        "path_version_trace": path_version_trace,
        "trajectory": trajectory,
        "summary": {
            "cycles_requested": len(snapshots),
            "cycles_completed": len(trajectory),
            "final_pose": pose,
            "distance_integrated_m": sum(
                math.hypot(row["pose_after"][0] - row["pose_before"][0], row["pose_after"][1] - row["pose_before"][1])
                for row in trajectory
            ),
            "negative_raw_action_count": sum(row["raw_action"]["linear_x"] < 0.0 for row in trajectory),
            "negative_linear_clamp_count": gate_clamp_count,
            "all_gated_linear_nonnegative": all(row["gated_action"]["linear_x"] >= 0.0 for row in trajectory),
            "all_commands_finite": all(
                math.isfinite(row["gated_action"]["linear_x"])
                and math.isfinite(row["gated_action"]["angular_z"])
                for row in trajectory
            ),
        },
        "replay_error": replay_error,
    }
    (scene_root / "trajectory.json").write_text(
        json.dumps(_json_safe(trajectory_payload), indent=2, sort_keys=True), encoding="utf-8"
    )

    source_payload = {
        "schema": "r1_n2_4_candidate_input_source_v1",
        "candidate_only": True,
        "scene": scene,
        "source_role": {
            "planner_tensors": "N1.11 high-level captured NPZ snapshots",
            "recovery_geometry": "N2.4 shadow JSON reanchor result",
            "safety_policy": "existing SafetyGateCore defaults",
        },
        "snapshots": [
            {
                "path": str(path.resolve()),
                "sha256": sha256(path),
                "cycle_id": index + 1,
                "keys_used": ["robot_state", "obstacle_points", "initial_path"],
                "shapes": {
                    key: list(snapshots[index][key].shape)
                    for key in ("robot_state", "obstacle_points", "initial_path")
                },
            }
            for index, path in enumerate(snapshot_paths)
        ],
        "shadow_json": {
            "path": str(shadow_path.resolve()),
            "sha256": sha256(shadow_path),
            "scene_evidence": shadow_evidence,
        },
        "planner_inputs": {
            "config_path": str(config_path.resolve()),
            "config_sha256": sha256(config_path),
            "checkpoint_path": str(checkpoint_path.resolve()),
            "checkpoint_sha256": sha256(checkpoint_path),
            "pan_from_yaml": pan,
            "receding_from_yaml": receding,
            "navigation_mode": navigation_mode,
        },
        "candidate_path": {
            "path_version": 2,
            "matrix_shape": list(candidate.shape),
            "matrix": candidate.tolist(),
        },
        "evidence_boundary": (
            "The N2 scene JSON supplies recovery geometry but not raw N2 planner tensors. "
            "This run therefore replays captured N1.11 tensors through the candidate path; "
            "the trajectory is not a Gazebo or recovery-success result."
        ),
    }
    (scene_root / "input_source.json").write_text(
        json.dumps(_json_safe(source_payload), indent=2, sort_keys=True), encoding="utf-8"
    )
    return {
        "scene": scene,
        "cycles_completed": len(trajectory),
        "worker_summary": str((scene_root / "worker_summary.json").resolve()),
        "trajectory": str((scene_root / "trajectory.json").resolve()),
        "input_source": str((scene_root / "input_source.json").resolve()),
        "replay_error": replay_error,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot-dir", type=Path, default=DEFAULT_SNAPSHOT_DIR)
    parser.add_argument("--shadow-json", type=Path, default=DEFAULT_SHADOW_JSON)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--scene", action="append", choices=PRIMARY_SCENES, dest="scenes")
    parser.add_argument("--cycles", type=int, default=6)
    parser.add_argument("--baseline-cycles", type=int, default=3)
    parser.add_argument("--warmup", type=int, default=3)
    parser.add_argument("--torch-intraop", type=int, default=1)
    parser.add_argument("--torch-interop", type=int, default=24)
    parser.add_argument("--dt", type=float, default=0.25)
    parser.add_argument("--navigation-mode", choices=("cyclic", "terminal"), default="terminal")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.cycles < 2 or args.baseline_cycles < 1 or args.baseline_cycles >= args.cycles:
        raise ValueError("require cycles >= 2 and 1 <= baseline-cycles < cycles")
    if args.warmup < 0 or args.dt <= 0.0:
        raise ValueError("warmup must be nonnegative and dt must be positive")
    snapshot_dir = args.snapshot_dir.expanduser().resolve()
    shadow_path = require_file(args.shadow_json, "shadow JSON")
    config_path = require_file(args.config, "planner config")
    checkpoint_path = require_file(args.checkpoint, "DUNE checkpoint")
    snapshots, snapshot_paths = load_snapshots(snapshot_dir, args.cycles)
    shadow = json.loads(shadow_path.read_text(encoding="utf-8"))
    with config_path.open("r", encoding="utf-8") as stream:
        planner_yaml = yaml.safe_load(stream) or {}
    pan = dict(planner_yaml.get("pan") or {})
    receding_value = planner_yaml.get("receding")
    receding = None if receding_value is None else int(receding_value)
    scenes = tuple(args.scenes or PRIMARY_SCENES)
    output_root = args.output_dir.expanduser().resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    results = []
    for scene in scenes:
        results.append(replay_scene(
            scene=scene,
            snapshots=snapshots,
            snapshot_paths=snapshot_paths,
            shadow=shadow,
            shadow_path=shadow_path,
            config_path=config_path,
            checkpoint_path=checkpoint_path,
            output_root=output_root,
            pan=pan,
            receding=receding,
            warmup_count=args.warmup,
            torch_intraop=args.torch_intraop,
            torch_interop=args.torch_interop,
            baseline_cycles=args.baseline_cycles,
            dt=args.dt,
            navigation_mode=args.navigation_mode,
        ))
    aggregate = {
        "schema": "r1_n2_4_candidate_replay_v1",
        "candidate_only": True,
        "scenes": results,
        "constraints": {
            "ros_commands_published": False,
            "gazebo_executed": False,
            "neupan_math_modified": False,
            "neupan_parameters_modified": False,
            "robot_description_modified": False,
            "gazebo_configuration_modified": False,
        },
    }
    aggregate_path = output_root / "replay_manifest.json"
    aggregate_path.write_text(
        json.dumps(_json_safe(aggregate), indent=2, sort_keys=True), encoding="utf-8"
    )
    print(json.dumps(_json_safe({"manifest": str(aggregate_path), "scenes": results}), indent=2, sort_keys=True))
    return 1 if any(item["replay_error"] for item in results) else 0


if __name__ == "__main__":
    raise SystemExit(main())
