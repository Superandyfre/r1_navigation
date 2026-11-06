#!/usr/bin/env python3
"""Real NeuPAN solver, 3D shadow filter and dual-sensor fusion in domain 192."""
import json,os,signal,subprocess,sys,time,unittest
from pathlib import Path
from unittest.mock import Mock
from std_msgs.msg import String
import failstop_chain_ros as chain
chain.CONTROLLER=Path(os.environ["NEUPAN_TEST_CONTROLLER"])
chain.GUARD=Path(os.environ["NEUPAN_TEST_GUARD"])
chain.SMOOTHER=Path(os.environ["NEUPAN_TEST_SMOOTHER"])

class FilterSolver(chain.FailstopChainROS):
    worker_parameters_override=dict(obstacle_motion_enabled=True,neupan_bypass_enabled=True,
        auto_gear_transition_enabled=True,auto_gear_opposite_direction_commit_cycles=4,
        command_stabilizer_enabled=False)
    guard_parameters_override=dict(collision_check_enabled=False,require_dynamic_footprint=False)
    def setUp(self):
        self.sidecars=[];self.sidecar_logs=[];self.gapflag=Path("/tmp")/("r1-shadow-gap-"+str(os.getpid()))
        self.addCleanup(self.stop_sidecars)
        helper=Path(__file__).with_name("self_filter_3d_sensor_peer.py")
        commands=[
          [sys.executable,str(helper),str(self.gapflag)],
          [os.environ["NEUPAN_TEST_3D_SHADOW"]],
          [os.environ["NEUPAN_TEST_CLOUD_TO_SCAN"],"--ros-args","--params-file",os.environ["NEUPAN_TEST_CLOUD_CONFIG"],
           "-r","cloud_in:=/r1_self_filter_3d_shadow/cloud","-r","scan:=/test_3d_front"],
          [os.environ["NEUPAN_TEST_SCAN_FUSION"],"--ros-args","--params-file",os.environ["NEUPAN_TEST_FUSION_CONFIG"],
           "-p","livox_scan_topic:=/test_3d_front","-p","rear_scan_topic:=/r1_self_filter_3d_shadow/rear_scan",
           "-p","output_scan_topic:="+chain.PREFIX+"worker_scan","-p","status_topic:=/test_3d_fusion_status"]]
        for command in commands:
            log=open("/tmp/r1-shadow-chain-"+str(os.getpid())+"-"+str(len(self.sidecars))+".log","w+")
            self.sidecar_logs.append(log)
            self.sidecars.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT))
        self.filter_status=[]
        super().setUp()
        self.filter_sub=self.node.create_subscription(String,"/r1_self_filter_3d_shadow/status",
            lambda m:self.filter_status.append((time.monotonic(),json.loads(m.data))),30)

    def tick(self):
        if hasattr(self,"pubs") and "worker_scan" in self.pubs and not isinstance(self.pubs["worker_scan"],Mock):
            self.pubs["worker_scan"]=Mock()
        for p in self.sidecars:
            if p.poll() is not None:
                for log in self.sidecar_logs:log.flush();log.seek(0);print(log.read())
                self.fail("3D pipeline exited")
        super().tick()

    def stop_sidecars(self):
        self.gapflag.unlink(missing_ok=True)
        for p in self.sidecars:
            if p.poll() is None:p.send_signal(signal.SIGINT)
        for p in self.sidecars:
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:p.kill();p.wait(timeout=3)
        for f in self.sidecar_logs:f.close()

    def test_real_solver_continues_through_short_tf_gap(self):
        self.assertEqual(self.worker_mode,"actual-solver")
        handle=self.goal();result=handle.get_result_async()
        self.until(lambda:any(v>.04 for _,v,_ in self.outputs) or result.done(),50.)
        self.assertFalse(result.done())
        start=time.monotonic()
        while time.monotonic()<start+1.:self.tick()
        marker=time.monotonic();self.gapflag.touch()
        while time.monotonic()<marker+.16:self.tick()
        self.gapflag.unlink()
        while time.monotonic()<marker+1.2:self.tick()
        self.assertFalse(result.done())
        rows=[(t,v,w) for t,v,w in self.outputs if marker-.15<t<marker+1.]
        self.assertGreater(len(rows),30)
        self.assertTrue(all(abs(v)>1e-8 or abs(w)>1e-8 for _,v,w in rows),rows)
        self.assertTrue(any(t>marker and j.get("reason") in ("stable_tf_fallback","reused_last_filtered_original_stamp")
            for t,j in self.filter_status),self.filter_status[-8:])
        self.future(handle.cancel_goal_async());self.future(result)
        (self.report/"3d-filter-solver-result.json").write_text(json.dumps(dict(
            samples=len(rows),zero_samples=0,tf_gap_sec=.16,physical_motion=False),indent=2))

if __name__=="__main__":unittest.main()
