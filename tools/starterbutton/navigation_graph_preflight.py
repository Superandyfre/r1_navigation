#!/usr/bin/env python3
"""Observe the current ROS graph before UI7 starts; never command the robot."""
import json
import time

# These names belong to the navigation runtime, not the externally owned
# hardware, localization or map servers. Never kill or alter discovered nodes.
EXCLUSIVE_NODES = frozenset({
    "/planner_server", "/controller_server", "/bt_navigator",
    "/neupan_velocity_smoother", "/r1_collision_guard", "/r1_arm_footprint",
    "/r1_scan_fusion", "/n1_16_production_bridge", "/ui7_runtime_supervisor",
})

def conflicts(nodes, cmd_publishers):
    duplicates = sorted(set(nodes) & EXCLUSIVE_NODES)
    publishers = sorted(set(cmd_publishers))
    return {"nodes": duplicates, "cmd_vel_publishers": publishers}

def main():
    import rclpy
    rclpy.init(args=[])
    node = rclpy.create_node("ui7_navigation_graph_preflight")
    names, pubs = set(), set()
    try:
        # Discovery is asynchronous; retain every conflict seen in the window.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            names.update(ns.rstrip("/") + "/" + name for name, ns in node.get_node_names_and_namespaces())
            pubs.update(info.node_namespace.rstrip("/") + "/" + info.node_name
                        for info in node.get_publishers_info_by_topic("/cmd_vel"))
        observed = conflicts(names, pubs)
        ok = not any(observed.values())
        print(json.dumps(dict(ok=ok, conflicts=observed,
            note="DDS discovery snapshot; cannot prevent a later concurrent launch")))
        return 0 if ok else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    raise SystemExit(main())
