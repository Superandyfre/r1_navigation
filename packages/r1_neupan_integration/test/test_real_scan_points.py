"""Real scan callback: filtered environment points must not move or duplicate."""
from collections import deque
import threading
import time
import unittest
from unittest.mock import Mock

import numpy as np
from sensor_msgs.msg import LaserScan
from r1_neupan_integration.n1_15_node import N115Node


class RealScanPoints(unittest.TestCase):
    def test_live_arm_geometry_does_not_change_environment_points(self):
        node = N115Node.__new__(N115Node)
        node.state_lock = threading.Lock()
        node.scans_received = node.duplicate_scans = node.regressed_scan_stamps = node.scan_counter = 0
        node._recent_scan_stamps = deque()
        node._recent_scan_stamp_set = set()
        node._last_accepted_scan_stamp_ns = -1
        node.robot_state = np.array([[2.0], [-1.0], [.7]])
        node.path_matrix = None
        node.dynamic_footprint_enabled = True
        node.dynamic_footprint = np.array([[-.4,-.4],[.8,-.4],[.8,.6],[-.4,.6]])
        node.dynamic_footprint_received_monotonic = time.monotonic()
        node.dynamic_footprint_timeout_sec = 10.0
        node.dynamic_mesh_components_enabled = True
        node.dynamic_mesh_components = [np.array([[.2,.3],[.7,.3],[.7,.6],[.2,.6]])]
        node.dynamic_mesh_components_received_monotonic = time.monotonic()
        node.dynamic_mesh_components_timeout_sec = 10.0
        node.mesh_overlap_diagnostics_enabled = False
        node.dynamic_footprint_clearance_m = .04
        node.scan_downsample = 1
        node.scan_range = (.15,8.)
        node.scan_angle_range = (-np.pi,np.pi)
        node.flip_angle = False
        node.lidar_frame = 'base_link'
        node.self_filter_enabled = True
        node.self_filter_polygon = np.array([[-.341,-.266],[.341,-.266],[.341,.266],[-.341,.266]])
        node.self_filter_polygon_margin_m = 0.
        node._planner_input_updated = Mock()
        scan = LaserScan()
        scan.header.frame_id='base_link'
        scan.header.stamp.sec=1
        scan.angle_min=-.4;scan.angle_max=.4;scan.angle_increment=.4
        scan.range_min=.15;scan.range_max=8.;scan.ranges=[.5,.2,.5]
        node.scan_callback(scan)
        angles=np.array([float(scan.angle_min),float(scan.angle_max)])
        points=np.vstack((.5*np.cos(angles),.5*np.sin(angles)))
        yaw=.7; rotation=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
        expected=rotation@points+np.array([[2.],[-1.]])
        np.testing.assert_allclose(node.obstacle_points,expected,atol=1e-7)
        np.testing.assert_allclose(node.raw_environment_points,expected,atol=1e-7)
        self.assertEqual(node.obstacle_points.shape,(2,2))
        self.assertEqual(node.latest_scan_filter['self_rejected_points'],1)
        self.assertEqual(node.latest_scan_filter['dynamic_obstacle_mapping'],'none')
        self.assertEqual(node.latest_scan_filter['dynamic_obstacle_max_displacement_m'],0.)
        node._planner_input_updated.assert_called_once()
        # A rear wall survives self-filtering without point relocation, and
        # diagnostics identify how many rear points reached the planner.
        scan.header.stamp.sec=2
        scan.angle_min=-np.pi+.1;scan.angle_max=np.pi-.1
        scan.angle_increment=(scan.angle_max-scan.angle_min)/2
        scan.ranges=[.5,.2,.5]
        node.scan_callback(scan)
        self.assertEqual(node.latest_scan_filter['rear_input_points'],2)
        self.assertEqual(node.latest_scan_filter['rear_retained_points'],2)
        nearest=node.latest_scan_filter['nearest_rear_base_link']
        self.assertLess(nearest[0],-.49)
        self.assertAlmostEqual(abs(nearest[1]),.5*np.sin(.1),places=6)
        # Rear returns between x=-.341 and -.371 are outside the actual
        # base_link mesh and must survive both configured self-filter stages.
        import yaml
        from pathlib import Path
        root=Path(__file__).resolve().parents[3]
        cfg=yaml.safe_load((root/'packages/navigation_neupan_bridge/config/robot_r1_hw1.yaml').read_text())
        polygon=cfg['n1_16_production_bridge']['ros__parameters']['self_filter_polygon_xy']
        node.self_filter_polygon=np.asarray(polygon).reshape(-1,2)
        scan.header.stamp.sec=3;scan.ranges=[.36,.2,.36]
        node.scan_callback(scan)
        self.assertEqual(node.latest_scan_filter['rear_retained_points'],2)
        self.assertGreater(node.latest_scan_filter['nearest_rear_base_link'][0],-.371)

if __name__ == '__main__': unittest.main()
