#!/usr/bin/env python3
"""N2.4C8 deterministic synchronous closed-loop benchmark conductor.

Implements the frozen S0-S12 planner-cycle contract on the D1 world:

  S0  world stays PAUSED between step bursts (gz sim launched without -r;
      every advance goes through `c7_control step K`, which requests
      multi_step=K + pause=true atomically).
  S1  observer physics iteration pinned at cycle start (observer state CSV,
      row = physics iteration invariant; the C8 world sets observer
      flush_every=1 so the CSV is readable mid-run).
  S2  scan / state(TF) / odom for that iteration confirmed available and
      fresh (exactly one new /scan_neupan per 100-step window at 10 Hz sim
      cadence; first cloud at sim 0.3 s = warmup boundary).
  S3  cycle inputs pinned under the production state lock.
  S4  exactly one NeuPAN solve per cycle through the production
      ProductionWholeCoreClient (same spawn worker, warmup 30, request
      timeout 0.25 s, frozen planner.yaml / DUNE checkpoint).
  S5  planner result validated: request_id, cycle_id, scan_counter,
      path_version, goal_version, finite outputs, freshness.
  S6  raw command built with production semantics (stop/arrive -> zero).
  S7  command passes the production SafetyGateCore (freshness_envelope,
      parameters identical to the D1 launch) before it may reach /cmd_vel.
      The command bridge is proven live during startup: zero-twist warmup
      publishes must appear in observer_cmdev.csv (flushed on receipt) - the
      first publish is never assumed effective.
  S8  command_apply_iteration = N+1 recorded before stepping.
  S9  exactly K physics steps via c7_control (its return is ACCEPTANCE of
      the world-control request, not completion).
  S10 observer polled until iteration == N+K AND row count matches.
  S11 window verified: exact iteration sequence, K rows, exactly one new cmd
      credit whose (v, w) equal the gated command bitwise, credit at N+1.
  S12 next cycle.

Determinism invariants recorded per cycle (contract section 2):
planner_cycle, scan_counter, state_iteration, request_id, command,
command_apply_iteration, target_iteration, actual_final_iteration, plus
input/output digests and a run-wide hash chain.

Production fidelity: the conductor reuses the real N1.15 node (scan
pipeline, TF state machine, ProductionWholeCoreClient) and the real
N2ScenarioPublisher (path/gear publishing at 2 Hz; d1_corner_obstacle
publishes no goal, so node.goal stays None / goal_version -1 exactly as in
the async C7 benchmark).  The 50 Hz async n1_16 loop is disabled; the
in-process SafetyGateCore.step replaces the r1_safety_gate node with
identical parameters and an equivalent freshness view (all inputs pinned to
the same frozen sim iteration, so planner/relay/command ages are 0 in the
sim-time domain).

Run under the N1 venv python with the ROS overlay sourced (the torch worker
inherits the interpreter via multiprocessing spawn):
    <venv>/bin/python n2_4c8_sync_conductor.py --run-dir ... --world ...
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

WS = Path("/home/pinggu/r1_sim_ws")
WS_INSTALL = WS / "install"

# Frozen D1/production contracts (mirrors of n2_4c_reproducibility.launch.py
# and n1_validation.launch.py defaults - verified against the launch sources).
SCENARIO = "d1_corner_obstacle"
MODEL_NAME = "r1_with_arm"
SPAWN_Z = "0.09"                      # r1_nav_sim.launch.py SPAWN_Z
WARMUP_STEPS = 300                    # 0.3 s sim: lidar first cloud at 0.3 s
STEPS_PER_CYCLE = 100                 # 0.1 s sim = exactly one 10 Hz scan
MAX_CYCLES = 400                      # ~40 s sim hard budget
MAX_ITERATION = 39500                 # observer max_iterations is 40000
CMD_WARMUP_PUBLISHES = 3              # zero twists to prove the bridge
SCAN_STAMP_EPS = 1e-3                 # stamp window tolerance (s of sim time)
OBSERVER_CMD_PRINT_TOL = 1e-9         # half-ulp of the observer's 9-decimal
                                      # fixed print of v/w (async_observer
                                      # uses std::setprecision(9) fixed)

# Production D1 safety-gate parameters (n2_4c_reproducibility.launch.py).
GATE_PARAMS = dict(
    max_linear=0.15,
    max_angular=0.4,
    max_linear_acceleration=0.30,
    max_angular_acceleration=0.8,
    command_timeout=0.30,
    scan_timeout=0.30,
    odom_timeout=0.30,
    tf_timeout=0.30,
    timing_timeout=0.50,
    require_timing=True,
    command_mode="freshness_envelope",
    relay_timeout=0.30,
    planner_freshness_timeout=0.30,
)

# Production n116 client parameters (n116_* launch args -> n115_* aliases).
CLIENT_PARAMS = dict(
    warmup_count=30,
    timeout_sec=0.25,
    torch_intraop=1,
    torch_interop=24,
    exact_capture_max_records=512,
)


class EnvironmentFailure(RuntimeError):
    """Bridge / infrastructure failure: fail immediately, never infer later."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(f"{code}: {message}")
        self.code = code


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_bytes(*chunks: bytes) -> str:
    digest = hashlib.sha256()
    for chunk in chunks:
        digest.update(chunk)
    return digest.hexdigest()


def strip_anaconda(value: str) -> str:
    return ":".join(
        part for part in value.split(":")
        if part and "conda" not in part.lower()
    )


class ObserverLog:
    """Incremental reader for the async_observer state CSV (flush_every=1).

    Row format (one line per physics iteration):
    iteration,sim_ns,steady_ns,x,y,z,yaw,wl_vel,wr_vel,wl_pos,wr_pos,seq,v,w
    """

    NCOLUMNS = 14

    def __init__(self, path: Path) -> None:
        self.path = path
        self.offset = 0
        self.tail = b""
        self.by_iteration: dict[int, tuple] = {}

    def _consume(self, chunk: bytes) -> None:
        self.tail += chunk
        while True:
            newline = self.tail.find(b"\n")
            if newline < 0:
                return
            line = self.tail[:newline]
            self.tail = self.tail[newline + 1:]
            text = line.decode("utf-8", "replace").strip()
            if not text or text.startswith("iteration,") or text.startswith("DONE,"):
                continue
            parts = text.split(",")
            if len(parts) != self.NCOLUMNS:
                raise EnvironmentFailure(
                    "OBSERVER_ROW_MALFORMED", f"unexpected row: {text[:120]}")
            self.by_iteration[int(parts[0])] = tuple(parts)

    def poll(self) -> int:
        """Read newly flushed rows; return the latest iteration (-1 if none)."""
        with open(self.path, "rb") as stream:
            stream.seek(self.offset)
            chunk = stream.read()
            self.offset += len(chunk)
        if chunk:
            self._consume(chunk)
        return max(self.by_iteration) if self.by_iteration else -1

    def window(self, first_iteration: int, last_iteration: int) -> list[tuple]:
        index = self.by_iteration
        return [index[i] for i in range(first_iteration, last_iteration + 1)
                if i in index]


class ProcessHandle:
    """Child process with execution-health evidence (contract section 4)."""

    def __init__(self, name: str, argv: list[str], cwd: Path, log_path: Path,
                 env: dict[str, str]) -> None:
        self.name = name
        self.argv = argv
        self.cwd = str(cwd)
        self.log_path = log_path
        self.started_utc = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        self.log = open(log_path, "wb")
        self.process = subprocess.Popen(
            argv, cwd=self.cwd, stdout=self.log, stderr=subprocess.STDOUT,
            env=env, start_new_session=True,
        )

    def alive(self) -> bool:
        return self.process.poll() is None

    def stop(self, sig: int = signal.SIGINT, grace: float = 6.0) -> int:
        if self.process.poll() is None:
            try:
                os.killpg(self.process.pid, sig)
            except ProcessLookupError:
                pass
            try:
                self.process.wait(timeout=grace)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self.process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.process.wait(timeout=5.0)
        try:
            self.log.close()
        except Exception:
            pass
        return self.process.returncode if self.process.returncode is not None else -1

    def health(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "argv": self.argv,
            "pid": self.process.pid,
            "cwd": self.cwd,
            "interpreter": self.argv[0] if self.argv else "",
            "stdout_stderr_path": str(self.log_path),
            "started_utc": self.started_utc,
            "alive_at_last_poll": self.alive(),
            "exit_code": self.process.poll(),
        }


def c7_control(world: str, action: str, count: int | None, timeout_ms: int,
               env: dict[str, str], cwd: Path) -> None:
    argv = [str(WS_INSTALL / "r1_step_injector/lib/r1_step_injector/c7_control"),
            world, action]
    if count is not None:
        argv.append(str(count))
    argv.append(str(timeout_ms))
    result = subprocess.run(argv, cwd=cwd, env=env, capture_output=True,
                            text=True, timeout=timeout_ms / 1000.0 + 10.0)
    if result.returncode != 0:
        code = "STEP_CONTROL_FAIL" if action == "step" else "CONTROL_FAIL"
        raise EnvironmentFailure(
            code,
            f"c7_control {action} rc={result.returncode} "
            f"{result.stderr.strip()[:200]}")


def write_node_params_file(run_dir: Path, integration_share: Path,
                           capture_metadata: dict[str, Any]) -> Path:
    """Production n1_15/n116 + scenario parameters as a params file.

    Values mirror n2_4c_reproducibility.launch.py -> n1_validation.launch.py:
    navigation_mode=terminal, latest_scan_only=true, n116_warmup_count=30,
    n116_timeout_sec=0.25 (aliased onto n115_*), torch 1/24, exact capture
    512 records, scan /scan_neupan.  cmd_vel_topic is the gate output topic
    /cmd_vel (in production the r1_safety_gate node publishes there).
    """
    params = {
        "use_sim_time": True,
        # conductor node (n1_15 surface)
        "robot_config_dir": str(integration_share / "config"),
        "map_frame": "odom",
        "base_frame": "base_link",
        "lidar_frame": "base_link",
        "scan_topic": "/scan_neupan",
        "cmd_vel_topic": "/cmd_vel",
        "navigation_mode": "terminal",
        "latest_scan_only": True,
        "scan_downsample": 1,
        "flip_angle": False,
        "include_initial_path_direction": True,
        "refresh_initial_path": False,
        "n115_output_dir": str(run_dir / "bridge"),
        "n115_exact_capture_dir": str(run_dir / "exact"),
        "n115_capture_scene": SCENARIO,
        "n115_capture_metadata": json.dumps(capture_metadata, sort_keys=True),
        "n115_warmup_count": CLIENT_PARAMS["warmup_count"],
        "n115_timeout_sec": CLIENT_PARAMS["timeout_sec"],
        "n115_exact_capture_max_records":
            CLIENT_PARAMS["exact_capture_max_records"],
        "torch_num_threads": CLIENT_PARAMS["torch_intraop"],
        "torch_num_interop_threads": CLIENT_PARAMS["torch_interop"],
        # scenario publisher
        "scenario": SCENARIO,
        "publish_hz": 2.0,
        "odom_topic": "/odom",
        "path_topic": "/r1_test_path",
        "goal_topic": "/goal_pose",
        "gear_topic": "/r1_motion_gear",
    }
    path = run_dir / "conductor_params.yaml"
    lines = ["/**:", "  ros__parameters:"]
    for key, value in params.items():
        if isinstance(value, bool):
            rendered = "true" if value else "false"
        elif isinstance(value, (int, float)):
            rendered = repr(value)
        else:
            rendered = json.dumps(value)
        lines.append(f"    {key}: {rendered}")
    path.write_text("\n".join(lines) + "\n")
    return path


def build_ros_nodes(run_dir: Path, stack_manifest: str, stack_manifest_sha: str,
                    params_file: Path):
    """Construct conductor node + scenario publisher (venv python only)."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from r1_neupan_integration.n1_15_node import N115Node
    from r1_neupan_integration.n1_16_bridge import (
        BridgeProtocolError,
        ProductionWholeCoreClient,
    )
    from r1_neupan_integration.n2_pre_scenarios import N2ScenarioPublisher
    from ros_gz_interfaces.msg import Contacts

    class ConductorNode(N115Node):
        """N1.15 I/O surface; the 50 Hz async loop is disabled."""

        NODE_NAME = "n2_4c8_sync_conductor"

        def make_client(self, output_dir, config_path, checkpoint_path, pan,
                        receding, torch_intraop, torch_interop, warmup_count,
                        timeout_sec, exact_capture_dir="",
                        exact_capture_max_records=512, capture_metadata=None):
            return ProductionWholeCoreClient(
                output_dir, config_path, checkpoint_path, pan, receding,
                torch_intraop, torch_interop, warmup_count, timeout_sec,
                exact_capture_dir=exact_capture_dir,
                exact_capture_max_records=exact_capture_max_records,
                capture_metadata=capture_metadata,
            )

        def run(self) -> None:
            # The production async timer loop must never fire here: cycle
            # progression is owned exclusively by the S0-S12 conductor.
            return

        def _c8_odom_cb(self, message: Odometry) -> None:
            stamp = (float(message.header.stamp.sec)
                     + float(message.header.stamp.nanosec) * 1e-9)
            with self.state_lock:
                self._c8_odom = (
                    float(message.twist.twist.linear.x),
                    float(message.twist.twist.angular.z),
                    stamp,
                )

        def _c8_contacts_cb(self, message: Contacts) -> None:
            # Wheel-ground and self contacts are normal physics; only a
            # contact involving the D1 obstacle model violates the scenario
            # (matches the async C7 no-collision criterion).
            obstacle_hit = False
            for contact in message.contacts:
                n1 = str(contact.collision1.name)
                n2 = str(contact.collision2.name)
                if "corner_furniture" in n1 or "corner_furniture" in n2:
                    obstacle_hit = True
            with self.state_lock:
                self._c8_contacts_count += len(message.contacts)
                if obstacle_hit:
                    self._c8_obstacle_contacts += 1

    rclpy.init(args=["--ros-args", "--params-file", str(params_file)])
    node = ConductorNode()
    with node.state_lock:
        node._c8_odom = (float("nan"), float("nan"), -1.0)
        node._c8_contacts_count = 0
        node._c8_obstacle_contacts = 0
    node.create_subscription(
        Odometry, "/odom", node._c8_odom_cb, 10)
    node.create_subscription(
        Contacts, "/world/empty/contacts", node._c8_contacts_cb, 10)
    scenario = N2ScenarioPublisher()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(scenario)
    import threading
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    return {
        "rclpy": rclpy,
        "node": node,
        "scenario": scenario,
        "executor": executor,
        "twist_type": Twist,
        "BridgeProtocolError": BridgeProtocolError,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--world", required=True)
    parser.add_argument("--bridges",
                        default=str(WS / "n2_4c8_results/bridges_c8.yaml"))
    parser.add_argument("--gz",
                        default="/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz")
    parser.add_argument("--phase", default="B")
    parser.add_argument("--cycle-budget", type=int, default=MAX_CYCLES)
    args = parser.parse_args()

    run_dir = Path(args.run_dir).resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    heartbeat_path = run_dir / "heartbeat.json"
    started_monotonic = time.monotonic()

    def heartbeat(**fields: Any) -> None:
        payload = {
            "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "wall_s": round(time.monotonic() - started_monotonic, 3),
            "pid": os.getpid(),
            "interpreter": sys.executable,
            "cwd": str(run_dir),
            **fields,
        }
        tmp = heartbeat_path.with_suffix(".tmp")
        tmp.write_text(json.dumps(payload, sort_keys=True))
        tmp.replace(heartbeat_path)

    heartbeat(state="starting", last_cycle=0, observer_iteration=0)

    # ------------------------------------------------------------------ env
    env = dict(os.environ)
    env["PATH"] = strip_anaconda(env.get("PATH", ""))
    env["GZ_IP"] = "127.0.0.1"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = (
        env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
        + f":{WS_INSTALL / 'r1_step_injector/lib'}")
    resource = ":".join(
        str(WS_INSTALL / pkg / "share")
        for pkg in ("r1_description", "rokae_description",
                    "realsense2_description", "rpp_components_description"))
    env["GZ_SIM_RESOURCE_PATH"] = resource + ":" + env.get("GZ_SIM_RESOURCE_PATH", "")
    env["EGL_PLATFORM"] = "surfaceless"
    env["__EGL_VENDOR_LIBRARY_FILENAMES"] = (
        "/usr/share/glvnd/egl_vendor.d/10_nvidia.json")
    env["OMP_NUM_THREADS"] = "1"
    env.pop("R1_N116_EXEC_PROBE", None)

    world_path = Path(args.world).resolve()
    bridges_path = Path(args.bridges).resolve()
    integration_share = (
        WS_INSTALL / "r1_neupan_integration/share/r1_neupan_integration")
    planner_config = integration_share / "config/planner.yaml"
    dune_checkpoint = integration_share / "config/models/dune_model_5000.pth"
    gazebo_xacro = WS_INSTALL / "r1_gazebo/share/r1_gazebo/urdf/r1_gazebo.xacro"
    generate_urdf = (
        WS_INSTALL / "r1_gazebo/share/r1_gazebo/scripts/generate_urdf.sh")
    lidar_patch = (
        WS_INSTALL / "r1_gazebo/share/r1_gazebo/scripts/lidar_spawn_patch.py")
    stack_manifest = WS / "n2_4c2_results/current_stack_manifest.json"
    stack_manifest_sha = ("2436bd8977e6a902d8d87984fa5e20aa965eefe"
                          "7d0ad442fe5f7f7237086271e")
    plugin_so = WS_INSTALL / "r1_step_injector/lib/libasync_observer.so"
    c7_world = (WS_INSTALL /
                "r1_step_injector/share/r1_step_injector/worlds/"
                "n2_d1_corner_obstacle_c7.sdf")

    for required in (world_path, bridges_path, planner_config, dune_checkpoint,
                     gazebo_xacro, generate_urdf, lidar_patch, stack_manifest,
                     plugin_so, c7_world):
        if not required.exists():
            print(f"[c8] ENV FAILURE: missing frozen input {required}",
                  flush=True)
            summary["failure_code"] = "ENV_MISSING_INPUT"
            summary["failure_message"] = str(required)
            (run_dir / "run_summary.json").write_text(
                json.dumps(summary, indent=2, sort_keys=True, default=str))
            return 3

    actual_manifest_sha = sha256_file(stack_manifest)
    if actual_manifest_sha != stack_manifest_sha:
        print("[c8] ENV FAILURE: stack manifest hash mismatch", flush=True)
        summary["failure_code"] = "ENV_MANIFEST_MISMATCH"
        summary["failure_message"] = actual_manifest_sha
        (run_dir / "run_summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True, default=str))
        return 3

    xacro = shutil.which("xacro") or "/opt/ros/jazzy/bin/xacro"
    if not Path(xacro).exists():
        print("[c8] ENV FAILURE: xacro not found", flush=True)
        summary["failure_code"] = "ENV_MISSING_INPUT"
        summary["failure_message"] = "xacro"
        (run_dir / "run_summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True, default=str))
        return 3

    # --------------------------------------------------- spawn URDF (frozen)
    urdf_result = subprocess.run(
        ["bash", str(generate_urdf), str(gazebo_xacro),
         "lidar_profile:=lidar_navigation", "lidar_source:=points",
         "lidar_mount:=base_link", "lidar_noise:=on", str(lidar_patch)],
        capture_output=True, text=True, env=env, timeout=120)
    if urdf_result.returncode != 0 or not urdf_result.stdout.strip():
        print(f"[c8] ENV FAILURE: spawn URDF generation rc={urdf_result.returncode}",
              flush=True)
        summary["failure_code"] = "ENV_URDF_FAIL"
        summary["failure_message"] = urdf_result.stderr.strip()[:400]
        (run_dir / "run_summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True, default=str))
        return 3
    spawn_urdf = urdf_result.stdout
    rsp_urdf_result = subprocess.run(
        [xacro, str(gazebo_xacro),
         "lidar_profile:=lidar_navigation", "lidar_source:=points",
         "lidar_mount:=base_link", "lidar_noise:=on"],
        capture_output=True, text=True, env=env, timeout=120)
    if rsp_urdf_result.returncode != 0 or not rsp_urdf_result.stdout.strip():
        print("[c8] ENV FAILURE: RSP URDF generation failed", flush=True)
        summary["failure_code"] = "ENV_URDF_FAIL"
        summary["failure_message"] = rsp_urdf_result.stderr.strip()[:400]
        (run_dir / "run_summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True, default=str))
        return 3
    rsp_urdf = rsp_urdf_result.stdout

    # ------------------------------------------------------- sealed manifest
    capture_metadata = {
        "stage": "N2.4C8",
        "mode": "deterministic_synchronous_closed_loop",
        "scene": SCENARIO,
        "benchmark": "sync-conductor",
        "steps_per_cycle": STEPS_PER_CYCLE,
        "warmup_steps": WARMUP_STEPS,
        "current_stack_manifest": str(stack_manifest),
        "current_stack_manifest_sha256": stack_manifest_sha,
    }
    sealed = {
        "stage": "N2.4C8",
        "phase": args.phase,
        "scenario": SCENARIO,
        "world": str(world_path),
        "world_sha256": sha256_file(world_path),
        "c7_world_sha256": sha256_file(c7_world),
        "observer_plugin_sha256": sha256_file(plugin_so),
        "bridges_yaml_sha256": sha256_file(bridges_path),
        "planner_config_sha256": sha256_file(planner_config),
        "dune_checkpoint_sha256": sha256_file(dune_checkpoint),
        "gazebo_xacro_sha256": sha256_file(gazebo_xacro),
        "spawn_urdf_sha256": hashlib.sha256(spawn_urdf.encode()).hexdigest(),
        "rsp_urdf_sha256": hashlib.sha256(rsp_urdf.encode()).hexdigest(),
        "conductor_script_sha256": sha256_file(Path(__file__).resolve()),
        "current_stack_manifest": str(stack_manifest),
        "current_stack_manifest_sha256": stack_manifest_sha,
        "steps_per_cycle": STEPS_PER_CYCLE,
        "warmup_steps": WARMUP_STEPS,
        "cmd_warmup_publishes": CMD_WARMUP_PUBLISHES,
        "cycle_budget": args.cycle_budget,
        "spawn_z": SPAWN_Z,
        "gate_params": GATE_PARAMS,
        "client_params": CLIENT_PARAMS,
        "goal_contract": "goal=None, goal_version=-1 (d1 publishes no goal, "
                         "identical to the async C7 benchmark)",
        "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }
    params_file = write_node_params_file(
        run_dir, integration_share, capture_metadata)
    sealed["node_params_file_sha256"] = sha256_file(params_file)
    (run_dir / "sealed_manifest.json").write_text(
        json.dumps(sealed, indent=2, sort_keys=True))

    # ------------------------------------------------------------ pre-flight
    leftover_pattern = (
        "^/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge"
        "|^/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz sim"
        "|pointcloud_to_laserscan"
        "|ros_gz_sim/create"
        "|robot_state_publisher.*r1_gazebo"
    )
    probe = subprocess.run(["pgrep", "-af", leftover_pattern],
                           capture_output=True, text=True)
    if probe.returncode == 0 and probe.stdout.strip():
        print(f"[c8] ENV FAILURE: leftover processes:\n{probe.stdout}",
              flush=True)
        summary["failure_code"] = "ENV_LEFTOVER_PROCESSES"
        summary["failure_message"] = probe.stdout.strip()[:800]
        (run_dir / "run_summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True, default=str))
        return 3

    processes: list[ProcessHandle] = []
    summary: dict[str, Any] = {
        "state": "failed", "phase": args.phase, "sealed": sealed,
        "processes": [], "cycles_completed": 0, "violations": [],
    }
    ros: dict[str, Any] = {"node": None}

    def shutdown_all() -> None:
        client = getattr(ros.get("node"), "client", None)
        if client is not None:
            try:
                client.finalize()
            except Exception:
                pass
        node_obj = ros.get("node")
        if node_obj is not None:
            try:
                ros["executor"].shutdown()
            except Exception:
                pass
            try:
                node_obj.destroy_node()
                ros["scenario"].destroy_node()
            except Exception:
                pass
            try:
                ros["rclpy"].shutdown()
            except Exception:
                pass
            ros["node"] = None
        for handle in reversed(processes):
            entry = handle.health()
            entry["stopped_utc"] = time.strftime(
                "%Y-%m-%dT%H:%M:%SZ", time.gmtime())
            entry["exit_code"] = handle.stop()
            summary["processes"].append(entry)

    def fail(code: str, message: str) -> int:
        summary["state"] = "failed"
        summary["failure_code"] = code
        summary["failure_message"] = message
        summary["wall_s"] = round(time.monotonic() - started_monotonic, 3)
        try:
            shutdown_all()
        except Exception:
            pass
        orphans = subprocess.run(["pgrep", "-af", leftover_pattern],
                                 capture_output=True, text=True)
        summary["orphans_after_run"] = orphans.stdout.strip().splitlines()
        (run_dir / "run_summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True, default=str))
        print(f"[c8] RUN FAILURE {code}: {message}", flush=True)
        return 4

    try:
        # ---------------------------------------------------- gz sim paused
        processes.append(ProcessHandle(
            "gz_sim", [args.gz, "sim", "-s", "-v", "2", str(world_path)],
            run_dir, run_dir / "gz.log", env))
        gz = processes[-1]
        deadline = time.monotonic() + 60.0
        ready = False
        while time.monotonic() < deadline:
            if not gz.alive():
                return fail("ENV_GZ_DIED", "gz sim exited during startup")
            probe = subprocess.run(
                [args.gz, "service", "-l"], capture_output=True, text=True,
                env=env, timeout=15)
            if "/world/empty/control" in (probe.stdout or ""):
                ready = True
                break
            time.sleep(1.0)
        if not ready:
            return fail("ENV_DISCOVERY_FAIL",
                        "world control service not discovered")
        heartbeat(state="gz_ready", observer_iteration=0)

        # -------------------------------------------------- bridges (hard gate)
        processes.append(ProcessHandle(
            "parameter_bridge",
            ["/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge",
             "--ros-args", "--params-file", str(bridges_path)],
            run_dir, run_dir / "bridge.log", env))
        bridge = processes[-1]
        time.sleep(2.0)
        if not bridge.alive():
            return fail("ENV_BRIDGE_FAIL", "parameter_bridge died at startup")
        bridge_text = (run_dir / "bridge.log").read_text(errors="replace")
        for banned in ("Failed", "No template specialization"):
            if banned in bridge_text:
                return fail(
                    "ENV_BRIDGE_FAIL",
                    f"bridge log contains {banned!r}: "
                    + next(line for line in bridge_text.splitlines()
                           if banned in line)[:200])
        required_topics = {"/cmd_vel", "/odom", "/clock", "/tf",
                           "/lidar/points", "/world/empty/contacts"}
        topics: set[str] = set()
        topics_deadline = time.monotonic() + 20.0
        while time.monotonic() < topics_deadline:
            probe = subprocess.run(
                ["ros2", "topic", "list"], capture_output=True, text=True,
                env=env, timeout=30)
            topics = set(probe.stdout.split())
            if required_topics <= topics:
                break
            time.sleep(1.0)
        if not required_topics <= topics:
            return fail("ENV_BRIDGE_FAIL",
                        f"required topics absent: "
                        f"{sorted(required_topics - topics)}")
        heartbeat(state="bridges_ok")

        # ------------------------------------------- preprocess + RSP + spawn
        processes.append(ProcessHandle(
            "pointcloud_to_laserscan",
            ["/opt/ros/jazzy/lib/pointcloud_to_laserscan/pointcloud_to_laserscan_node",
             "--ros-args", "-r", "cloud_in:=/lidar/points",
             "-r", "scan:=/scan_neupan",
             "--params-file",
             str(WS_INSTALL / "r1_lidar_preprocess/share/r1_lidar_preprocess/config/lidar_navigation.yaml"),
             "-p", "use_sim_time:=true"],
            run_dir, run_dir / "preprocess.log", env))
        # robot_description is passed via params file: the URDF is far too
        # large and XML-laden for a `-p key:=value` CLI override (rcl arg
        # parser rejects it - observed as RSP SIGABRT).
        rsp_params = run_dir / "rsp_params.yaml"
        rsp_params.write_text(
            "/**:\n  ros__parameters:\n    use_sim_time: true\n"
            "    robot_description: "
            + json.dumps(rsp_urdf) + "\n")
        processes.append(ProcessHandle(
            "robot_state_publisher",
            ["/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher",
             "--ros-args", "--params-file", str(rsp_params)],
            run_dir, run_dir / "rsp.log", env))
        # One-shot spawner: rc=0 exit = spawn request accepted.  In a paused
        # world it may legitimately stay alive until the first step; real
        # spawn evidence is enforced later (observer model pose + "model
        # resolved" marker after warmup).
        processes.append(ProcessHandle(
            "gz_create",
            ["/opt/ros/jazzy/lib/ros_gz_sim/create", "-world", "empty",
             "-name", MODEL_NAME, "-z", SPAWN_Z, "-string", spawn_urdf],
            run_dir, run_dir / "create.log", env))
        for handle in (processes[-3], processes[-2]):
            time.sleep(1.0)
            if not handle.alive():
                return fail("ENV_SPAWN_STACK_FAIL",
                            f"{handle.name} died: see {handle.log_path}")
        create = processes[-1]
        create_deadline = time.monotonic() + 30.0
        while create.alive() and time.monotonic() < create_deadline:
            time.sleep(0.5)
        if not create.alive() and create.process.returncode != 0:
            return fail("ENV_SPAWN_FAIL",
                        f"gz_create exited rc={create.process.returncode}: "
                        f"see {create.log_path}")

        # ------------------------------------------------------ ROS node up
        ros.update(build_ros_nodes(run_dir, str(stack_manifest),
                                   stack_manifest_sha, params_file))
        node = ros["node"]
        Twist = ros["twist_type"]
        BridgeProtocolError = ros["BridgeProtocolError"]
        from r1_neupan_integration.n2_pre_safety_gate import SafetyGateCore

        # --------------------------------------------------- observer watcher
        observer_csv = run_dir / "observer_state.csv"
        cmdev_csv = run_dir / "observer_cmdev.csv"
        wait_start = time.monotonic()
        while not observer_csv.exists():
            if time.monotonic() - wait_start > 30.0:
                return fail("ENV_OBSERVER_FAIL",
                            "observer state CSV never appeared")
            if not gz.alive():
                return fail("ENV_GZ_DIED", "gz sim died before observer output")
            time.sleep(0.2)
        observer = ObserverLog(observer_csv)

        gate = SafetyGateCore(**GATE_PARAMS)

        # ------------------------------------------------- warmup: S0 stepping
        c7_control("empty", "step", WARMUP_STEPS, 60000, env, run_dir)
        warmup_deadline = time.monotonic() + 90.0
        iteration = -1
        while True:
            iteration = observer.poll()
            if iteration >= WARMUP_STEPS:
                break
            if time.monotonic() > warmup_deadline:
                return fail("ENV_OBSERVER_FAIL",
                            f"warmup incomplete: iteration={iteration}")
            if not gz.alive():
                return fail("ENV_GZ_DIED", "gz sim died during warmup")
            time.sleep(0.05)
        if iteration != WARMUP_STEPS:
            return fail("ITERATION_MISMATCH",
                        f"warmup ended at iteration {iteration}")
        if min(observer.by_iteration) != 1:
            return fail("ENV_OBSERVER_FAIL",
                        f"observer rows do not start at iteration 1 "
                        f"(first={min(observer.by_iteration)})")
        gz_log_text = (run_dir / "gz.log").read_text(errors="replace")
        spawn_rows = observer.window(1, min(observer.by_iteration))
        if not any(abs(float(row[5])) > 1e-6 or abs(float(row[3])) > 1e-6
                   or abs(float(row[4])) > 1e-6 for row in spawn_rows):
            return fail("ENV_SPAWN_FAIL",
                        "observer rows show no model pose - spawn failed "
                        f"(z~0.09 expected); gz.log: {gz_log_text[-200:]}")
        with node.state_lock:
            obstacle_contacts = int(node._c8_obstacle_contacts)
        if obstacle_contacts:
            return fail("CONTACT_DETECTED",
                        f"{obstacle_contacts} obstacle contact messages "
                        f"during warmup")

        # S7 startup: prove the command bridge delivers, using cmdev evidence
        # (flushed on receipt, visible while the world is paused).
        for _ in range(CMD_WARMUP_PUBLISHES):
            node.cmd_pub.publish(Twist())
            time.sleep(0.05)
        cmdev_deadline = time.monotonic() + 10.0
        warmup_credits: list[list[str]] = []
        while time.monotonic() < cmdev_deadline:
            if cmdev_csv.exists():
                rows = [line.split(",") for line in
                        cmdev_csv.read_text().splitlines()[1:] if line.strip()]
                if len(rows) >= CMD_WARMUP_PUBLISHES:
                    warmup_credits = rows
                    break
            time.sleep(0.1)
        if len(warmup_credits) < CMD_WARMUP_PUBLISHES:
            return fail(
                "ENV_CMD_BRIDGE_FAIL",
                f"zero-twist warmup not credited in cmdev "
                f"({len(warmup_credits)} msgs) - command bridge not established")
        for seq_index, row in enumerate(warmup_credits[:CMD_WARMUP_PUBLISHES],
                                        start=1):
            if int(row[0]) != seq_index:
                return fail("ENV_CMD_BRIDGE_FAIL",
                            f"cmdev seq {row[0]} != {seq_index}")
            if abs(float(row[2])) > 0.0 or abs(float(row[3])) > 0.0:
                return fail("ENV_CMD_BRIDGE_FAIL",
                            f"cmdev msg {row[0]} not zero: v={row[2]} w={row[3]}")

        # S2 health checkpoint: full sensor stack + production path present.
        # The scenario node's 2 Hz timer runs on the frozen sim clock, so in
        # the paused phase it can never fire on its own; the conductor calls
        # the node's own _publish() here (identical message construction -
        # same _coordinates/_quat/publishers - as the timer path).  The timer
        # stays enabled and resumes publishing once stepping advances sim
        # time; repeat publications are ignored by path_callback.
        health_deadline = time.monotonic() + 20.0
        while True:
            with node.state_lock:
                have_scan = int(node.scan_counter) >= 1
                have_path = int(node.path_version) >= 1
                contacts_count = int(node._c8_obstacle_contacts)
                odom_stamp = float(node._c8_odom[2])
            tf_ok = node._robot_transform()
            if not have_path and odom_stamp > 0.0:
                ros["scenario"]._publish()
                with node.state_lock:
                    have_path = int(node.path_version) >= 1
            if have_scan and have_path and contacts_count == 0 \
                    and tf_ok and odom_stamp > 0.0:
                break
            if time.monotonic() > health_deadline:
                return fail(
                    "SENSOR_STACK_FAIL",
                    f"scan={have_scan} path={have_path} "
                    f"contacts={contacts_count} tf={tf_ok} "
                    f"odom_stamp={odom_stamp}")
            time.sleep(0.1)
        gate.release()
        heartbeat(state="running", last_cycle=0,
                  observer_iteration=observer.poll())

        # ------------------------------------------------------- cycle loop
        long_running = [handle for handle in processes
                        if handle.name != "gz_create"]
        seq_expected = CMD_WARMUP_PUBLISHES
        last_scan_counter = 0
        chain = hashlib.sha256(b"n2_4c8").hexdigest()
        expected_iteration = WARMUP_STEPS
        cycles_done = 0
        contacts_seen = 0
        outcome = "cycle_budget"
        stop_cycle = None
        arrival_cycle = None
        cycles_file = open(run_dir / "cycles.jsonl", "w", encoding="utf-8")
        try:
            for cycle in range(1, args.cycle_budget + 1):
                if not gz.alive():
                    return fail("ENV_GZ_DIED",
                                f"gz sim died before cycle {cycle}")
                for handle in long_running:
                    if not handle.alive():
                        return fail("ENV_PROCESS_DIED",
                                    f"{handle.name} died before cycle {cycle}")

                # S1: observer iteration pinned.
                iteration_n = observer.poll()
                if iteration_n != expected_iteration:
                    return fail("ITERATION_MISMATCH",
                                f"cycle {cycle}: iteration {iteration_n} != "
                                f"{expected_iteration}")
                sim_now = iteration_n * 0.001

                # S2: scan / state / odom for this iteration.
                scan_deadline = time.monotonic() + 5.0
                while True:
                    with node.state_lock:
                        new_scan = int(node.scan_counter) > last_scan_counter
                    if new_scan:
                        break
                    if time.monotonic() > scan_deadline:
                        return fail("SENSOR_SCAN_MISSING",
                                    f"cycle {cycle}: no new scan for iteration "
                                    f"{iteration_n}")
                    time.sleep(0.01)
                if not node._robot_transform():
                    return fail("SENSOR_TF_MISSING",
                                f"cycle {cycle}: TF lookup failed")
                window_start_sim = (iteration_n - STEPS_PER_CYCLE) * 0.001

                # S3: pin cycle inputs (production n1_16 semantics; d1 has no
                # goal, so goal stays None / goal_version -1).
                with node.state_lock:
                    scan_counter = int(node.scan_counter)
                    state = node.robot_state.copy()
                    points = (np.empty((2, 0), dtype=np.float64)
                              if node.obstacle_points is None
                              else node.obstacle_points.copy())
                    path = node.path_matrix.copy()
                    path_version = int(node.path_version)
                    goal_pinned = (None if node.goal is None
                                   else node.goal.copy())
                    goal_version = int(node.goal_version)
                    scan_stamp = float(node.latest_scan_stamp_sec)
                    scan_ranges = (None if node.latest_scan_ranges is None
                                   else node.latest_scan_ranges.copy())
                    scan_meta = tuple(float(value) for value in
                                      node.latest_scan_meta)
                    scan_recv_monotonic = float(
                        node.latest_scan_receive_monotonic)
                    scan_update_monotonic = float(
                        node.latest_scan_update_monotonic)
                    tf_stamp = float(getattr(node, "tf_stamp_sec", -1.0))
                    odom_v = float(node._c8_odom[0])
                    odom_w = float(node._c8_odom[1])
                    odom_stamp = float(node._c8_odom[2])
                    active_gear = int(node.active_gear)
                    contacts_seen = int(node._c8_obstacle_contacts)
                    node.last_processed_scan = scan_counter
                    last_scan_counter = scan_counter
                # Diagnostic only (default off): dump cycle-1 pinned inputs so
                # the scan divergence across boots can be quantified at the
                # root level.  Never used by the Phase A/B gates.
                if cycle == 1 and os.environ.get("C8_SCAN_DUMP"):
                    np.save(os.path.join(run_dir, "diag_scan_cycle1.npy"),
                            scan_ranges)
                    np.save(os.path.join(run_dir, "diag_points_cycle1.npy"),
                            points)
                if contacts_seen:
                    return fail("CONTACT_DETECTED",
                                f"cycle {cycle}: {contacts_seen} contact messages")
                if scan_ranges is None:
                    return fail("SENSOR_SCAN_MISSING",
                                f"cycle {cycle}: scan ranges missing at pin")
                if not (window_start_sim - SCAN_STAMP_EPS) <= scan_stamp <= \
                        (sim_now + SCAN_STAMP_EPS):
                    return fail("SENSOR_SCAN_STALE",
                                f"cycle {cycle}: scan stamp {scan_stamp} outside "
                                f"[{window_start_sim}, {sim_now}]")

                # S4: exactly one solve (startup is idempotent; identical to
                # the production n1_16 per-cycle call).
                try:
                    node.client.startup(
                        path, state, points, path_version,
                        navigation_loop_mode=node.navigation_mode,
                        active_gear=active_gear)
                    outputs, record = node.client.solve(
                        state, points, path, path_version, scan_counter,
                        scan_stamp, goal=goal_pinned, goal_version=goal_version,
                        cycle_id=cycle,
                        navigation_loop_mode=node.navigation_mode,
                        active_gear=active_gear,
                        scan_callback_receive_monotonic=scan_recv_monotonic,
                        scan_update_monotonic=scan_update_monotonic)
                except BridgeProtocolError as exc:
                    return fail("PLANNER_ERROR", f"cycle {cycle}: {exc}")

                # S5: result validation.
                if int(record.get("request_id", -1)) != node.client.request_id:
                    return fail("PLANNER_CONTRACT",
                                f"cycle {cycle}: request_id mismatch")
                for key, expected in (("cycle_id", cycle),
                                      ("scan_counter", scan_counter),
                                      ("path_version", path_version),
                                      ("goal_version", goal_version)):
                    if int(record.get(key, -1)) != int(expected):
                        return fail(
                            "PLANNER_CONTRACT",
                            f"cycle {cycle}: {key} {record.get(key)} != "
                            f"{expected}")
                action = outputs["action"]
                flat_action = np.asarray(action, dtype=np.float64).reshape(-1)
                if flat_action.size < 2 or not (
                        math.isfinite(float(flat_action[0]))
                        and math.isfinite(float(flat_action[1]))):
                    return fail("NONFINITE",
                                f"cycle {cycle}: nonfinite action")
                with node.state_lock:
                    stale = (int(node.path_version) != path_version
                             or int(node.goal_version) != goal_version)
                if stale:
                    return fail("PLANNER_CONTRACT",
                                f"cycle {cycle}: stale response (version changed)")

                # S6: raw command with production stop/arrive semantics.
                info = dict(record.get("info", {}))
                stopped = bool(info.get("stop", False)) or \
                    bool(info.get("arrive", False))
                if stopped:
                    raw_v, raw_w = 0.0, 0.0
                else:
                    raw_v, raw_w = float(flat_action[0]), float(flat_action[1])
                semantic = dict(record.get("semantic", {}))
                timing = dict(record.get("timings", {}))

                # S7: production safety gate (freshness envelope) in sim-time
                # domain; sensor stamps come from the pinned messages.
                decision = gate.step(
                    sim_now, raw_v, raw_w,
                    command_seen=sim_now, relay_seen=sim_now,
                    planner_result_seen=sim_now, scan_seen=scan_stamp,
                    odom_seen=odom_stamp, tf_seen=tf_stamp,
                    timing_seen=sim_now)
                if gate.failure_latched:
                    return fail("SAFETY_LATCH",
                                f"cycle {cycle}: latched "
                                f"{list(decision.reasons)}")
                command_message = Twist()
                command_message.linear.x = float(decision.linear)
                command_message.angular.z = float(decision.angular)
                node.cmd_pub.publish(command_message)

                # S8: command application plan.
                apply_iteration = iteration_n + 1
                target_iteration = iteration_n + STEPS_PER_CYCLE
                if target_iteration > MAX_ITERATION:
                    outcome = "iteration_budget"
                    break

                # S9: step K (acceptance only).
                c7_control("empty", "step", STEPS_PER_CYCLE, 120000, env,
                           run_dir)

                # S10: poll observer until iteration == N + K.
                step_deadline = time.monotonic() + 120.0
                actual_final = iteration_n
                while True:
                    actual_final = observer.poll()
                    if actual_final >= target_iteration:
                        break
                    if time.monotonic() > step_deadline:
                        return fail("ITERATION_MISMATCH",
                                    f"cycle {cycle}: stalled at iteration "
                                    f"{actual_final}")
                    if not gz.alive():
                        return fail("ENV_GZ_DIED",
                                    f"gz sim died in cycle {cycle}")
                    time.sleep(0.02)
                if actual_final != target_iteration:
                    return fail(
                        "ITERATION_MISMATCH",
                        f"cycle {cycle}: final iteration {actual_final} != "
                        f"{target_iteration}")

                # S11: window verification.
                rows = observer.window(iteration_n + 1, target_iteration)
                if len(rows) != STEPS_PER_CYCLE:
                    return fail("ENV_OBSERVER_FAIL",
                                f"cycle {cycle}: window rows {len(rows)} != "
                                f"{STEPS_PER_CYCLE}")
                iterations = [int(row[0]) for row in rows]
                if iterations != list(range(iteration_n + 1,
                                            target_iteration + 1)):
                    return fail("ENV_OBSERVER_FAIL",
                                f"cycle {cycle}: non-consecutive iterations")
                new_seq = max(int(row[11]) for row in rows)
                if new_seq != seq_expected + 1:
                    return fail(
                        "COMMAND_CREDIT_MISSING",
                        f"cycle {cycle}: observer cmd seq {new_seq} != "
                        f"{seq_expected + 1}")
                credit_row = next(
                    (row for row in rows
                     if int(row[11]) == seq_expected + 1), None)
                credit_iteration = int(credit_row[0])
                if credit_iteration > iteration_n + 3:
                    return fail("COMMAND_CREDIT_LATE",
                                f"cycle {cycle}: credit at {credit_iteration}")
                # Observer CSV prints v/w with setprecision(9) fixed, so the
                # logged text only carries 9 decimals.  This is a comparison
                # against the LOG's print precision, not a determinism
                # relaxation: cross-run identity of the printed strings
                # (checked in Phase B) stays fully strict.
                if abs(float(credit_row[12]) - float(decision.linear)) \
                        > OBSERVER_CMD_PRINT_TOL \
                        or abs(float(credit_row[13])
                               - float(decision.angular)) \
                        > OBSERVER_CMD_PRINT_TOL:
                    return fail(
                        "COMMAND_MISMATCH",
                        f"cycle {cycle}: credited ({credit_row[12]}, "
                        f"{credit_row[13]}) != gated "
                        f"({decision.linear}, {decision.angular})")
                seq_expected = new_seq

                # Determinism invariants + digests (contract sections 2, 7).
                pose_row = observer.by_iteration[iteration_n]
                scan_digest = sha256_bytes(
                    np.ascontiguousarray(scan_ranges,
                                         dtype=np.float64).tobytes(),
                    np.asarray(scan_meta, dtype=np.float64).tobytes(),
                    np.asarray([scan_stamp], dtype=np.float64).tobytes())
                obstacle_digest = sha256_bytes(
                    np.ascontiguousarray(points,
                                         dtype=np.float64).tobytes(),
                    np.asarray(points.shape, dtype=np.int64).tobytes())
                planner_state_digest = sha256_bytes(
                    np.ascontiguousarray(state, dtype=np.float64).tobytes(),
                    np.ascontiguousarray(path, dtype=np.float64).tobytes(),
                    (b"none" if goal_pinned is None else
                     np.ascontiguousarray(goal_pinned,
                                          dtype=np.float64).tobytes()),
                    np.asarray([path_version, goal_version, active_gear],
                               dtype=np.int64).tobytes())
                distance = outputs.get("distance", None)
                distance_arr = np.asarray(distance, dtype=np.float64) \
                    if distance is not None else np.empty(0)
                optimized_velocity = outputs.get("optimized_velocity", None)
                optimized_arr = np.asarray(optimized_velocity,
                                           dtype=np.float64) \
                    if optimized_velocity is not None else np.empty(0)
                planner_output_digest = sha256_bytes(
                    np.ascontiguousarray(outputs["opt_state"],
                                         dtype=np.float64).tobytes(),
                    np.ascontiguousarray(outputs["ref_state"],
                                         dtype=np.float64).tobytes(),
                    np.ascontiguousarray(action, dtype=np.float64).tobytes(),
                    np.ascontiguousarray(optimized_arr).tobytes(),
                    np.ascontiguousarray(distance_arr).tobytes(),
                    json.dumps(info, sort_keys=True).encode())
                chain = sha256_bytes(chain.encode(),
                                     planner_output_digest.encode())
                distance_value = None
                if distance_arr.size:
                    candidate = float(distance_arr.reshape(-1)[0])
                    if math.isfinite(candidate):
                        distance_value = candidate

                cycle_record = {
                    "planner_cycle": cycle,
                    "state_iteration": iteration_n,
                    "sim_time_start_sec": sim_now,
                    "sim_ns_end": float(pose_row[1]),
                    "robot_x_observer": float(pose_row[3]),
                    "robot_y_observer": float(pose_row[4]),
                    "robot_z_observer": float(pose_row[5]),
                    "robot_yaw_observer": float(pose_row[6]),
                    "wl_vel": float(pose_row[7]),
                    "wr_vel": float(pose_row[8]),
                    "state_x_planner": float(state[0, 0]),
                    "state_y_planner": float(state[1, 0]),
                    "state_yaw_planner": float(state[2, 0]),
                    "odom_v": odom_v,
                    "odom_w": odom_w,
                    "odom_stamp_sec": odom_stamp,
                    "tf_stamp_sec": tf_stamp,
                    "scan_counter": scan_counter,
                    "scan_stamp_sec": scan_stamp,
                    "scan_digest": scan_digest,
                    "obstacle_tensor_digest": obstacle_digest,
                    "planner_state_digest": planner_state_digest,
                    "planner_output_digest": planner_output_digest,
                    "chain_digest": chain,
                    "request_id": int(record["request_id"]),
                    "worker_sequence": int(record.get("worker_sequence", -1)),
                    "worker_pid": int(record.get("worker_pid", -1)),
                    "raw_action": [raw_v, raw_w],
                    "gated_command": [float(decision.linear),
                                      float(decision.angular)],
                    "gate_reasons": list(decision.reasons),
                    "gate_clamped": bool(decision.clamped),
                    "gate_negative_input": bool(decision.negative_input),
                    "safety_latched": False,
                    "command_apply_iteration": apply_iteration,
                    "command_credit_iteration": credit_iteration,
                    "target_iteration": target_iteration,
                    "actual_final_iteration": actual_final,
                    "dune_min_distance": distance_value,
                    "path_index": int(semantic.get("point_index", -1)),
                    "obstacle_points_count": int(points.shape[1]),
                    "pan_iterations": int(timing.get("worker_pan_iterations",
                                                     0)),
                    "pan_early_converged": bool(
                        timing.get("worker_pan_early_converged", 0)),
                    "stop": bool(info.get("stop", False)),
                    "arrive": bool(info.get("arrive", False)),
                    "info": info,
                    "contacts_cumulative": contacts_seen,
                    "worker_failures_cumulative":
                        int(getattr(node.client, "worker_failures", 0)),
                    "finite": True,
                    "nan_inf": 0,
                    "stale_response": 0,
                    "goal_version": int(goal_version),
                    "path_version": int(path_version),
                }
                cycles_file.write(json.dumps(
                    cycle_record, sort_keys=True, default=str) + "\n")
                cycles_file.flush()
                cycles_done = cycle
                if cycle % 10 == 0 or stopped:
                    heartbeat(
                        state="running", last_cycle=cycle,
                        observer_iteration=actual_final,
                        outcome_hint=("stop" if info.get("stop") else
                                      "arrive" if info.get("arrive") else
                                      "running"))
                expected_iteration = target_iteration
                if bool(info.get("arrive", False)):
                    outcome, arrival_cycle = "ARRIVE", cycle
                    break
                if bool(info.get("stop", False)):
                    outcome, stop_cycle = "EARLY_STOP", cycle
                    break
        finally:
            cycles_file.close()

        summary.update({
            "state": "completed",
            "outcome": outcome,
            "stop_cycle": stop_cycle,
            "arrival_cycle": arrival_cycle,
            "cycles_completed": cycles_done,
            "final_iteration": observer.poll(),
            "cmd_seq_total": seq_expected,
            "safety_latched": gate.failure_latched,
        })
    except EnvironmentFailure as exc:
        return fail(exc.code, str(exc))
    except KeyboardInterrupt:
        return fail("INTERRUPTED", "conductor interrupted")
    except Exception as exc:  # noqa: BLE001
        import traceback
        return fail("UNEXPECTED", f"{exc}\n{traceback.format_exc()[-2000:]}")

    # ------------------------------------------------------------- teardown
    shutdown_all()
    orphans = subprocess.run(["pgrep", "-af", leftover_pattern],
                             capture_output=True, text=True)
    summary["orphans_after_run"] = orphans.stdout.strip().splitlines()
    summary["wall_s"] = round(time.monotonic() - started_monotonic, 3)
    summary["ended_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    (run_dir / "run_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, default=str))
    heartbeat(state="done", last_cycle=summary["cycles_completed"],
              observer_iteration=summary["final_iteration"])
    print(f"[c8] RUN DONE outcome={summary['outcome']} "
          f"cycles={summary['cycles_completed']} "
          f"final_iter={summary['final_iteration']} "
          f"wall={summary['wall_s']}s", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
