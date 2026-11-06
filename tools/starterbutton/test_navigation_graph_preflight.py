import unittest
from navigation_graph_preflight import conflicts

class GraphConflicts(unittest.TestCase):
    def test_hardware_does_not_block(self):
        self.assertEqual(conflicts(["/rpp_ros_driver", "/livox_node"], []),
                         {"nodes": [], "cmd_vel_publishers": []})
    def test_running_navigation_and_velocity_publishers_block(self):
        result = conflicts(["/planner_server", "/planner_server", "/bt_navigator"],
                           ["/teleop", "/teleop"])
        self.assertEqual(result["nodes"], ["/bt_navigator", "/planner_server"])
        self.assertEqual(result["cmd_vel_publishers"], ["/teleop"])

if __name__ == "__main__": unittest.main()
