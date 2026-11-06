import json
from pathlib import Path
import sqlite3
import tempfile
import unittest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message
from std_msgs.msg import String
from analyze_ui7_bag import analyze

class BagAnalysis(unittest.TestCase):
    def test_gaps_and_status_transitions_from_closed_bag(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/"bag.db3"
            db=sqlite3.connect(path)
            db.execute("create table topics (id integer, name text, type text)")
            db.execute("create table messages (topic_id integer, timestamp integer, data blob)")
            db.executemany("insert into topics values (?,?,?)",[(1,"/cmd_vel","geometry_msgs/msg/Twist"),
                (2,"/r1_collision_guard/status","std_msgs/msg/String"),
                (3,"/neupan_semantics","std_msgs/msg/String"),
                (4,"/odom","nav_msgs/msg/Odometry")])
            move=Twist();move.linear.x=.1
            db.executemany("insert into messages values (?,?,?)",[(1,1000000000,bytes(serialize_message(move))),
                (1,1010000000,bytes(serialize_message(Twist()))),
                (2,1005000000,bytes(serialize_message(String(data=json.dumps(dict(state="STOP",reason="odom_stale"))))))])
            semantics = dict(info=dict(stop=True, arrive=False), dune=dict(min_distance_m=-0.03),
                active_gear=-1, requested_gear=-1, path_version=2, goal_version=1,
                obstacle_points_count=17, gear_transition_gate=dict(state="STOPPING",
                reason="waiting_for_zero_speed", raw_linear_x=-0.3, linear_x=0.0, odom_fresh=True))
            odom=Odometry(); odom.twist.twist.linear.x=-0.1
            db.executemany("insert into messages values (?,?,?)", [
                (3, 1011000000, bytes(serialize_message(String(data=json.dumps(semantics))))),
                (4, 1012000000, bytes(serialize_message(odom)))])
            db.commit();db.close()
            result=analyze([path])
            self.assertEqual(result["topics"]["/cmd_vel"]["gap_max_ms"],10.)
            self.assertEqual([v["state"] for v in result["velocity_transitions"]["/cmd_vel"]],
                             ["nonzero_or_nonfinite","zero"])
            self.assertEqual(result["status_transitions"][0]["state"]["reason"],"odom_stale")
            diagnostic = next(x["state"] for x in result["status_transitions"] if x["topic"] == "/neupan_semantics")
            self.assertEqual(diagnostic["gear_reason"], "waiting_for_zero_speed")
            self.assertEqual(diagnostic["dune_min_m"], -0.03)
            self.assertEqual(diagnostic["raw_linear_x"], -0.3)
            self.assertEqual(result["velocity_transitions"]["/odom"][0]["linear_x"], -0.1)
            self.assertEqual(result["decode_errors"],[])

if __name__=="__main__": unittest.main()
