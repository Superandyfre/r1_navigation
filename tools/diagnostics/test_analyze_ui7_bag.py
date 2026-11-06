import json
from pathlib import Path
import sqlite3
import tempfile
import unittest
from geometry_msgs.msg import Twist
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
                (2,"/r1_collision_guard/status","std_msgs/msg/String")])
            move=Twist();move.linear.x=.1
            db.executemany("insert into messages values (?,?,?)",[(1,1000000000,bytes(serialize_message(move))),
                (1,1010000000,bytes(serialize_message(Twist()))),
                (2,1005000000,bytes(serialize_message(String(data=json.dumps(dict(state="STOP",reason="odom_stale"))))))])
            db.commit();db.close()
            result=analyze([path])
            self.assertEqual(result["topics"]["/cmd_vel"]["gap_max_ms"],10.)
            self.assertEqual([v["state"] for v in result["velocity_transitions"]["/cmd_vel"]],
                             ["nonzero_or_nonfinite","zero"])
            self.assertEqual(result["status_transitions"][0]["state"]["reason"],"odom_stale")
            self.assertEqual(result["decode_errors"],[])

if __name__=="__main__": unittest.main()
