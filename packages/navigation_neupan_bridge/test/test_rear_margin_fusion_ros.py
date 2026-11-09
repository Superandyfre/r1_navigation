#!/usr/bin/env python3
"""Exercise the real scan fusion node with isolated synthetic sensor inputs."""
import math, os, signal, subprocess, time, unittest
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

@unittest.skipUnless(os.environ.get('NEUPAN_ISOLATED_ROS_TEST') == '1', 'isolated opt-in only')
class RearMarginFusion(unittest.TestCase):
    def test_rear_wall_is_preserved_and_sensor_specific_masks_remain(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '197')
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'), '1')
        rclpy.init()
        node = Node('rear_margin_test_peer')
        front = node.create_publisher(LaserScan, '/rear_margin_test/front', qos_profile_sensor_data)
        rear = node.create_publisher(LaserScan, '/rear_margin_test/rear', qos_profile_sensor_data)
        outputs = []
        node.create_subscription(LaserScan, '/rear_margin_test/fused', outputs.append, qos_profile_sensor_data)
        cmd = [os.environ['NEUPAN_TEST_SCAN_FUSION'], '--ros-args', '--params-file',
               os.environ['NEUPAN_TEST_FUSION_CONFIG'], '-p', 'livox_scan_topic:=/rear_margin_test/front',
               '-p', 'rear_scan_topic:=/rear_margin_test/rear', '-p', 'output_scan_topic:=/rear_margin_test/fused',
               '-p', 'status_topic:=/rear_margin_test/status']
        proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        def index(point): return round((math.atan2(point[1],point[0])+math.pi)/(math.pi/360))
        rear_wall=(-.381,-.18); rear_body=(-.36,-.10); front_margin=(-.381,.10); front_clear=(-.65,.22)
        def scan(points):
            msg=LaserScan();msg.header.stamp=node.get_clock().now().to_msg();msg.header.frame_id='base_link'
            msg.angle_min=-math.pi;msg.angle_increment=math.pi/360;msg.angle_max=math.pi
            msg.range_min=.01;msg.range_max=8.;msg.ranges=[float('inf')]*721
            for point in points: msg.ranges[index(point)]=math.hypot(*point)
            return msg
        try:
            end=time.monotonic()+12
            while time.monotonic()<end and not outputs:
                self.assertIsNone(proc.poll(), 'fusion process exited')
                front.publish(scan([front_margin,front_clear]));rear.publish(scan([rear_wall,rear_body]))
                rclpy.spin_once(node,timeout_sec=.1)
            self.assertTrue(outputs, 'no fused scan received')
            m=outputs[-1]
            self.assertTrue(math.isfinite(m.ranges[index(rear_wall)]), 'rear wall was filtered')
            self.assertTrue(math.isinf(m.ranges[index(rear_body)]), 'body point was retained')
            self.assertTrue(math.isinf(m.ranges[index(front_margin)]), 'front margin changed')
            self.assertTrue(math.isfinite(m.ranges[index(front_clear)]), 'clear front return lost')
            self.assertFalse(any(name=='/cmd_vel' for name,_ in node.get_topic_names_and_types()))
        finally:
            if proc.poll() is None: proc.send_signal(signal.SIGINT)
            try: proc.wait(timeout=5)
            except subprocess.TimeoutExpired: proc.kill();proc.wait(timeout=3)
            node.destroy_node();rclpy.shutdown()
if __name__=='__main__': unittest.main()
