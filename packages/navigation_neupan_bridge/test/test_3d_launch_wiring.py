#!/usr/bin/env python3
"""Inspect actual launch actions without starting ROS nodes or hardware."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.utilities import evaluate_parameters

path=Path(__file__).resolve().parents[1]/"launch/r1_neupan_hw1_motion.launch.py"
spec=importlib.util.spec_from_file_location("r1_motion_launch",path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class MotionWiring(unittest.TestCase):
    def build(self,**overrides):
        context=LaunchContext()
        for action in module.generate_launch_description().entities:
            if isinstance(action,DeclareLaunchArgument):
                context.launch_configurations[action.name]=perform_substitutions(context,action.default_value)
        context.launch_configurations.update(calibration_record="/home/rpp/rpp_ws/config/r1_hw1_calibration.json",
            scan_source="dual",use_mesh_self_filter="true")
        context.launch_configurations.update(overrides)
        nodes={n._Node__node_name:n for n in module._build(context)}
        return context,nodes
    def params(self,c,node):
        result={}
        for row in evaluate_parameters(c,node._Node__parameters):
            if isinstance(row,dict):result.update(row)
        return result
    def remaps(self,c,node):
        return {perform_substitutions(c,a):perform_substitutions(c,b)
                for a,b in node._Node__remappings}
    def test_existing_2d_mode_is_unchanged(self):
        c,n=self.build(mesh_filter_mode="2d")
        self.assertNotIn("r1_arm_self_filter_3d",n)
        self.assertAlmostEqual(self.params(c,n["r1_arm_footprint"])["mesh_projection_margin_m"],0.06)
        self.assertEqual(self.remaps(c,n["neupan_pointcloud_to_scan"])["cloud_in"],"/lidar/pointcloud")
        self.assertNotIn("rear_scan_topic",self.params(c,n["r1_scan_fusion"]))
        self.assertEqual(self.params(c,n["n1_16_production_bridge"])["scan_topic"],"/scan_neupan_mesh_shadow")
    def test_dual_3d_mode_has_one_filtered_navigation_path(self):
        c,n=self.build(mesh_filter_mode="3d")
        f=self.params(c,n["r1_arm_self_filter_3d"])
        self.assertEqual(tuple(f["front_echo_box_base_xyz"]), (0.045, 0.235, -0.405, -0.305, 0.670, 0.795))
        self.assertAlmostEqual(self.params(c,n["r1_arm_footprint"])["mesh_projection_margin_m"],0.0)
        self.assertAlmostEqual(self.params(c,n["n1_16_production_bridge"])["dynamic_footprint_clearance_m"],0.04)
        self.assertFalse(self.params(c,n["r1_localization_monitor"])["static_map_overlap_hard_fault"])
        custom,cn=self.build(mesh_filter_mode="3d",mesh_3d_projection_margin_m="0.02")
        self.assertAlmostEqual(self.params(custom,cn["r1_arm_footprint"])["mesh_projection_margin_m"],0.02)
        with self.assertRaises(RuntimeError):self.build(mesh_filter_mode="3d",mesh_3d_projection_margin_m="0.11")
        self.assertTrue(f["rear_scan_enabled"])
        self.assertEqual(f["output_cloud_topic"],"/r1_arm_self_filter_3d/cloud")
        self.assertEqual(self.remaps(c,n["neupan_pointcloud_to_scan"])["cloud_in"],f["output_cloud_topic"])
        fusion=self.params(c,n["r1_scan_fusion"])
        self.assertEqual(fusion["rear_scan_topic"],f["output_scan_topic"])
        self.assertEqual(fusion["output_scan_topic"],"/scan_neupan_mesh_shadow")
        self.assertEqual(self.params(c,n["n1_16_production_bridge"])["scan_topic"],fusion["output_scan_topic"])
        self.assertNotIn("scan_topic",self.params(c,n["r1_collision_guard"]))
        self.assertEqual(self.remaps(c,n["r1_arm_footprint"])["/scan_neupan_mesh_shadow"],"/r1_arm_legacy_2d_scan")
    def test_front_only_3d_mode_and_incompatible_option(self):
        c,n=self.build(mesh_filter_mode="3d",scan_source="livox_only")
        self.assertNotIn("r1_scan_fusion",n)
        self.assertFalse(self.params(c,n["r1_arm_self_filter_3d"])["rear_scan_enabled"])
        self.assertEqual(self.params(c,n["r1_arm_self_filter_3d"])["input_cloud_topic"],"/lidar/pointcloud")
        alt,alternate=self.build(mesh_filter_mode="3d",scan_source="livox_only",input_cloud="/custom/timed_cloud")
        self.assertEqual(self.params(alt,alternate["r1_arm_self_filter_3d"])["input_cloud_topic"],"/custom/timed_cloud")
        self.assertEqual(self.remaps(c,n["neupan_pointcloud_to_scan"])["scan"],"/scan_neupan_mesh_shadow")
        c,n=self.build(mesh_filter_mode="3d",use_mesh_self_filter="false")
        self.assertNotIn("r1_arm_self_filter_3d",n)
        self.assertEqual(self.params(c,n["n1_16_production_bridge"])["scan_topic"],"/scan_neupan")
    def test_attachment_is_identical_in_filter_and_footprint(self):
        with tempfile.TemporaryDirectory() as d:
            file=Path(d)/"meshes.yaml"
            file.write_text("schema: r1_arm_extra_meshes_v1\nmeshes:\n  - frame: held_object\n    uri: package://example/mesh.stl\n    scale: 1.0\n")
            c,n=self.build(mesh_filter_mode="3d",arm_extra_mesh_config=str(file))
            a=self.params(c,n["r1_arm_self_filter_3d"]);b=self.params(c,n["r1_arm_footprint"])
            for key in ("additional_mesh_frames","additional_mesh_uris","additional_mesh_scales"):
                self.assertEqual(a[key],b[key])
            file.write_text("schema: r1_arm_extra_meshes_v1\nmeshes:\n  - frame: held_object\n    uri: x\n    scale: -1\n")
            with self.assertRaises(RuntimeError):self.build(mesh_filter_mode="3d",arm_extra_mesh_config=str(file))

if __name__=="__main__":unittest.main()
