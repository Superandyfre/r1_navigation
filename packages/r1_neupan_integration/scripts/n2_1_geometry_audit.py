#!/usr/bin/env python3
"""Audit the R1 navigation geometry contract without changing planner code.

The input URDF may be the expanded Gazebo overlay.  The report separates the
physical navigation envelope from the simulation-only traction spheres and
from the complete articulated robot collision inventory.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


WS = Path(os.environ.get("R1_SIM_WS", "/home/pinggu/r1_sim_ws"))
WHEEL_LINKS = {"wheel_L_link", "wheel_R_link"}
CASTER_LINKS = {"caster_LF_link", "caster_RF_link", "caster_LB_link", "caster_RB_Link"}
NAV_LINKS = {"base_link", *WHEEL_LINKS, *CASTER_LINKS}
TRACTION_RADIUS = 0.0831


def _vec(value: str | None, default=(0.0, 0.0, 0.0)) -> tuple[float, float, float]:
    if not value:
        return default
    values = [float(item) for item in value.split()]
    return tuple((values + list(default))[:3])


def _package_path(uri: str) -> Path | None:
    match = re.fullmatch(r"package://([^/]+)/(.+)", uri)
    if not match:
        return None
    package, relative = match.groups()
    roots = [
        WS / "install" / package / "share" / package,
        WS / "src" / package,
        WS / "r1_sim_export" / "ros2_src" / package,
    ]
    for root in roots:
        candidate = root / relative
        if candidate.exists():
            return candidate
    return None


def _stl_aabb(path: Path) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0] if len(data) >= 84 else 0
    points = []
    if len(data) == 84 + count * 50 and not data[:5].lower() == b"solid":
        for index in range(count):
            points.extend(struct.unpack_from("<3f", data, 84 + index * 50 + 12))
        xyz = [points[offset::3] for offset in range(3)]
    else:
        values = re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data)
        xyz = [[float(value) for value in column] for column in zip(*values)] if values else [[], [], []]
    if not xyz[0]:
        raise ValueError(f"no STL vertices: {path}")
    lower = tuple(min(column) for column in xyz)
    upper = tuple(max(column) for column in xyz)
    return tuple((lower[index], upper[index]) for index in range(3))


def _origin(element: ET.Element) -> tuple[float, float, float]:
    origin = element.find("origin")
    return _vec(origin.get("xyz") if origin is not None else None)


def _bounds_union(bounds: list[tuple[tuple[float, float], tuple[float, float]]]):
    return (
        (min(item[0][0] for item in bounds), max(item[0][1] for item in bounds)),
        (min(item[1][0] for item in bounds), max(item[1][1] for item in bounds)),
    )


def _dims(bounds):
    return {"x_min": bounds[0][0], "x_max": bounds[0][1],
            "y_min": bounds[1][0], "y_max": bounds[1][1],
            "length": bounds[0][1] - bounds[0][0],
            "width": bounds[1][1] - bounds[1][0]}


def _planner_contract(path: Path) -> dict:
    import yaml

    config = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    robot = dict(config.get("robot") or {})
    if robot.get("vertices") is not None:
        vertices = [[float(value) for value in point[:2]] for point in robot["vertices"]]
        bounds = ((min(point[0] for point in vertices), max(point[0] for point in vertices)),
                  (min(point[1] for point in vertices), max(point[1] for point in vertices)))
        definition = "explicit polygon vertices"
    else:
        length = float(robot["length"])
        width = float(robot["width"])
        wheelbase = float(robot.get("wheelbase") or 0.0)
        x0 = -(length - wheelbase) / 2.0
        bounds = ((x0, x0 + length), (-width / 2.0, width / 2.0))
        vertices = [[x0, -width / 2.0], [x0 + length, -width / 2.0],
                    [x0 + length, width / 2.0], [x0, width / 2.0]]
        definition = "robot.length/width rectangle; wheelbase=%g" % wheelbase
    return {"file": str(path), "length": bounds[0][1] - bounds[0][0],
            "width": bounds[1][1] - bounds[1][0], "bounds": _dims(bounds),
            "vertices": vertices, "definition": definition,
            "yaml_robot": robot,
            "actual_use": {
                "loader": "/home/pinggu/neupan_n0/NeuPAN/neupan/neupan.py:init_from_yaml",
                "constructor": "neupan.robot.robot.__init__",
                "conversion": "robot.cal_vertices -> gen_inequal_from_vertex -> robot.G, robot.h",
            }}


def audit(urdf_path: Path, planner_path: Path) -> dict:
    root = ET.parse(urdf_path).getroot()
    links = {link.get("name"): link for link in root.findall("link")}
    joints = {
        joint.find("child").get("link"): joint
        for joint in root.findall("joint")
        if joint.find("child") is not None
    }
    collisions = []
    physical_bounds = []
    gazebo_bounds = []
    link_details = {}

    for name, link in links.items():
        for collision in link.findall("collision"):
            geometry = collision.find("geometry")
            if geometry is None:
                continue
            mesh = geometry.find("mesh")
            shape = "unknown"
            local = None
            if mesh is not None:
                shape = "mesh"
                mesh_path = _package_path(mesh.get("filename", ""))
                if mesh_path is not None:
                    local = _stl_aabb(mesh_path)
            box = geometry.find("box")
            sphere = geometry.find("sphere")
            if box is not None:
                shape = "box"
                size = _vec(box.get("size"))
                local = tuple((-size[i] / 2.0, size[i] / 2.0) for i in range(3))
            if sphere is not None:
                shape = "sphere"
                radius = float(sphere.get("radius"))
                local = tuple((-radius, radius) for _ in range(3))
            if local is None:
                continue
            origin = _origin(collision)
            xy = ((local[0][0] + origin[0], local[0][1] + origin[0]),
                  (local[1][0] + origin[1], local[1][1] + origin[1]))
            collisions.append({"link": name, "shape": shape, "xy": _dims(xy)})
            link_details.setdefault(name, []).append({"shape": shape, "xy": _dims(xy)})

    # The exported chassis links are direct children of base_link, so their
    # 2D bounds can be placed with the joint origin without a full FK engine.
    for name in NAV_LINKS:
        for item in link_details.get(name, []):
            joint = joints.get(name)
            offset = _origin(joint) if joint is not None else (0.0, 0.0, 0.0)
            bounds = ((item["xy"]["x_min"] + offset[0], item["xy"]["x_max"] + offset[0]),
                      (item["xy"]["y_min"] + offset[1], item["xy"]["y_max"] + offset[1]))
            if name in NAV_LINKS:
                physical_bounds.append(bounds)
            if name == "base_link":
                gazebo_bounds.append(bounds)

    # The simulator overlay replaces wheel meshes with spherical traction
    # contacts and caster meshes with spheres.  Those are physics contacts,
    # not the physical 2D navigation footprint.
    for name in WHEEL_LINKS:
        joint = joints.get(name)
        offset = _origin(joint) if joint is not None else (0.0, 0.0, 0.0)
        gazebo_bounds.append(((offset[0] - TRACTION_RADIUS, offset[0] + TRACTION_RADIUS),
                              (offset[1] - TRACTION_RADIUS, offset[1] + TRACTION_RADIUS)))
    for name in CASTER_LINKS:
        joint = joints.get(name)
        offset = _origin(joint) if joint is not None else (0.0, 0.0, 0.0)
        radius = max(max(item["xy"]["x_max"] - item["xy"]["x_min"],
                         item["xy"]["y_max"] - item["xy"]["y_min"]) / 2.0
                      for item in link_details.get(name, [{"xy": {"x_min": 0.0, "x_max": 0.0,
                                                                   "y_min": 0.0, "y_max": 0.0}}]))
        gazebo_bounds.append(((offset[0] - radius, offset[0] + radius),
                              (offset[1] - radius, offset[1] + radius)))

    physical = _dims(_bounds_union(physical_bounds))
    gazebo = _dims(_bounds_union(gazebo_bounds))
    planner = _planner_contract(planner_path)
    source_counts = {}
    for item in collisions:
        source_counts[item["shape"]] = source_counts.get(item["shape"], 0) + 1

    joints_of_interest = {}
    for name, joint in joints.items():
        parent = joint.find("parent")
        if name in WHEEL_LINKS | CASTER_LINKS or (parent is not None and parent.get("link") == "base_link"):
            joints_of_interest[joint.get("name")] = {
                "child": name, "parent": parent.get("link") if parent is not None else None,
                "origin": list(_origin(joint)), "axis": list(_vec(joint.find("axis").get("xyz") if joint.find("axis") is not None else None)),
            }

    return {
        "schema": "r1_n2_1_geometry_contract_v1",
        "urdf": str(urdf_path),
        "planner": planner,
        "source_collision_counts": source_counts,
        "source_collision_link_count": len({item["link"] for item in collisions}),
        "navigation_links": sorted(NAV_LINKS),
        "physical_navigation_footprint": physical,
        "gazebo_overlay_contact_envelope": gazebo,
        "base_link_collision_aabb": next(item["xy"] for item in collisions if item["link"] == "base_link"),
        "gazebo_overlay_rules": {
            "base_link": "source mesh AABB converted to box",
            "wheels": "source wheel mesh collision deleted; one 0.0831 m traction sphere per wheel",
            "casters": "source caster mesh AABB converted to sphere",
            "other_links": "mesh collisions deleted by chassis_sim_patch.py",
        },
        "joints": joints_of_interest,
        "lidar_geometry": {
            "planner_input_frame": "base_link",
            "preprocess_target_frame": "base_link",
            "lidar_joint_origin": joints_of_interest.get("lidar_joint", {}).get("origin"),
            "laser_joint_origin": joints_of_interest.get("laser_joint", {}).get("origin"),
            "relation": "pointcloud_to_laserscan transforms lidar_link points into base_link before /scan_neupan; planner lidar_frame is base_link",
        },
        "safety_gate_geometry": "none; r1_safety_gate enforces velocity, freshness, e-stop and no-reverse invariants only",
        "contract": {
            "correct_navigation_definition": "physical base/caster/drive-wheel 2D projected envelope in base_link",
            "planner_matches_physical": math.isclose(planner["length"], physical["length"], rel_tol=0.01) and math.isclose(planner["width"], physical["width"], rel_tol=0.01),
            "correction_required": not (math.isclose(planner["length"], physical["length"], rel_tol=0.01) and math.isclose(planner["width"], physical["width"], rel_tol=0.01)),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", required=True, type=Path)
    parser.add_argument("--planner", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = audit(args.urdf, args.planner)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps({
        "base_link_collision_aabb": result["base_link_collision_aabb"],
        "physical_navigation_footprint": result["physical_navigation_footprint"],
        "gazebo_overlay_contact_envelope": result["gazebo_overlay_contact_envelope"],
        "planner": {key: result["planner"][key] for key in ("length", "width", "definition")},
        "correction_required": result["contract"]["correction_required"],
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
