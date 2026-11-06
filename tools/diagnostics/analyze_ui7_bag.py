#!/usr/bin/env python3
"""Summarize a closed sqlite rosbag; timestamps are recorder receive times."""
import argparse
import json
import math
import sqlite3
from collections import defaultdict
from pathlib import Path

OBSERVE = {"/r1_collision_guard/status", "/r1_localization_health",
           "/neupan_controller_status", "/neupan_worker_health", "/neupan_execution_context",
           "/neupan_semantics"}
VELOCITIES = {"/neupan_nav2_raw_cmd_vel", "/neupan_nav2_smoothed_cmd_vel", "/cmd_vel", "/odom"}

def percentile(sorted_values, q):
    return sorted_values[min(len(sorted_values)-1, int(q*(len(sorted_values)-1)))] if sorted_values else None

def analyze(paths):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    topic_times, transitions, velocity, errors = defaultdict(list), [], defaultdict(list), []
    for path in paths:
        db = sqlite3.connect("file:" + str(path.resolve()) + "?mode=ro", uri=True)
        try:
            topics = {i:(name, typ) for i,name,typ in db.execute("select id,name,type from topics")}
            types = {}
            for ident,(name,typ) in topics.items():
                if name in OBSERVE | VELOCITIES:
                    try: types[ident] = get_message(typ)
                    except (ImportError, AttributeError, ValueError) as exc:
                        errors.append({"topic":name,"error":str(exc)})
            previous = {}
            for ident,stamp,data in db.execute("select topic_id,timestamp,data from messages order by timestamp"):
                name, _ = topics[ident]
                topic_times[name].append(stamp)
                if ident not in types: continue
                try:
                    msg = deserialize_message(data, types[ident])
                    if name in OBSERVE:
                        payload = json.loads(msg.data)
                        if name == "/neupan_semantics":
                            info = payload.get("info") or {}
                            gate = payload.get("gear_transition_gate") or {}
                            dune = payload.get("dune") or {}
                            signature = {"stop": info.get("stop"), "arrive": info.get("arrive"),
                                         "active_gear": payload.get("active_gear"),
                                         "requested_gear": payload.get("requested_gear"),
                                         "gear_state": gate.get("state"), "gear_reason": gate.get("reason"),
                                         "path_version": payload.get("path_version"),
                                         "goal_version": payload.get("goal_version")}
                            state = dict(signature, dune_min_m=dune.get("min_distance_m"),
                                         obstacle_points=payload.get("obstacle_points_count"),
                                         raw_linear_x=gate.get("raw_linear_x"),
                                         output_linear_x=gate.get("linear_x"),
                                         odom_fresh=gate.get("odom_fresh"),
                                         goal_distance=payload.get("goal_distance"),
                                         scan_filter=payload.get("scan_filter"))
                        else:
                            state = {key:payload.get(key) for key in ("state","reason","authorization_state",
                                "authorization_reason","fault_latched","fault_reason","navigation_attempt_id") if key in payload}
                            signature = state
                        if signature != previous.get(name):
                            transitions.append(dict(topic=name, receive_ns=stamp, state=state))
                            previous[name] = signature
                    else:
                        twist = msg.twist.twist if name == "/odom" else msg
                        x, w = float(twist.linear.x), float(twist.angular.z)
                        is_zero = math.isfinite(x) and math.isfinite(w) and x == 0.0 and w == 0.0
                        # Store transitions only; this is a diagnostic, not a proof of physical motion.
                        state = "zero" if is_zero else "nonzero_or_nonfinite"
                        if state != previous.get(name):
                            velocity[name].append(dict(receive_ns=stamp, state=state, linear_x=x if math.isfinite(x) else None, angular_z=w if math.isfinite(w) else None))
                            previous[name] = state
                except (ValueError, TypeError, AttributeError) as exc:
                    errors.append(dict(topic=name, receive_ns=stamp, error=str(exc)))
        finally:
            db.close()
    summary = {}
    for name,times in topic_times.items():
        times.sort();gaps=sorted((b-a)/1e6 for a,b in zip(times,times[1:]))
        summary[name] = dict(count=len(times), first_receive_ns=times[0], last_receive_ns=times[-1],
            gap_p50_ms=percentile(gaps,.5),gap_p99_ms=percentile(gaps,.99),
            gap_p999_ms=percentile(gaps,.999),gap_max_ms=max(gaps) if gaps else None)
    return dict(schema="ui7_bag_observation_v1", topics=summary,
        status_transitions=sorted(transitions,key=lambda e:e["receive_ns"]),
        velocity_transitions=dict(velocity), decode_errors=errors,
        limitations=["recorder receive timestamps are not sensor sampling times",
                     "topic transitions show correlation, not proven causation",
                     "zero commands do not prove physical stop; inspect odometry and hardware"])

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag",type=Path,help="closed .db3 file or bag directory")
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    paths=sorted(args.bag.glob("*.db3")) if args.bag.is_dir() else [args.bag]
    if not paths: parser.error("no .db3 files found")
    with args.output.open("x") as stream:
        json.dump(analyze(paths),stream,indent=2,allow_nan=False)
        stream.write("\n")

if __name__ == "__main__": main()
