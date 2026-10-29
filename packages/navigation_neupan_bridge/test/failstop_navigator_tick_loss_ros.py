#!/usr/bin/env python3
"""Pause actual FollowPath BT ticks without freezing Navigator ROS services."""
import json
import os
from pathlib import Path
import time
import unittest
import yaml
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from failstop_navigator_owner_ros import NavigatorOwner
import failstop_navigator_ros as nav


class NavigatorTickLoss(NavigatorOwner):
    def start(self,exe,params,remaps):
        if exe!=nav.NAVIGATOR:return super().start(exe,params,remaps)
        library=Path(os.environ['NEUPAN_TEST_TICK_PLUGIN_PATH']).resolve(strict=True)
        self.assertEqual(library.name,'libtest_neupan_pause_tick_bt_node.so')
        self.pause_file=self.report/'pause-tick'
        data=yaml.safe_load(params.read_text());config=data['bt_navigator']['ros__parameters']
        config['plugin_lib_names'].append('test_neupan_pause_tick_bt_node')
        for key in ('default_nav_to_pose_bt_xml','default_nav_through_poses_bt_xml'):
            source=Path(config[key]);xml=source.read_text()
            child='<NeuPANFollowPath path="{path}" controller_id="FollowPath"/>'
            self.assertEqual(xml.count(child),1)
            target=self.report/source.name
            target.write_text(xml.replace(child,'<TestPauseTick pause_file="'+str(self.pause_file)+'">'+child+'</TestPauseTick>'))
            config[key]=str(target)
        params.write_text(yaml.safe_dump(data))
        previous=os.environ.get('LD_LIBRARY_PATH')
        os.environ['LD_LIBRARY_PATH']=str(library.parent)+os.pathsep+(previous or '')
        try:super().start(exe,params,remaps)
        finally:
            if previous is None:os.environ.pop('LD_LIBRARY_PATH',None)
            else:os.environ['LD_LIBRARY_PATH']=previous

    def test_navigator_tick_loss(self):
        client,handle,result=self.send_top()
        attempt=self.moving_attempt(result)
        marker=time.monotonic();self.pause_file.touch(exist_ok=False)
        self.until(lambda:any(t>marker and j.get('fault_latched') and
            j.get('fault_reason')=='owner_lease_expired' for t,j in self.runtime_statuses),3.)
        fault_at=next(t for t,j in self.runtime_statuses if t>marker and
            j.get('fault_latched') and j.get('fault_reason')=='owner_lease_expired')
        service=self.node.create_client(GetState,'/bt_navigator/get_state')
        self.until(service.service_is_ready,2.)
        state=self.future(service.call_async(GetState.Request()),2.)
        self.assertEqual(state.current_state.id,3,'Navigator must still be ACTIVE')
        self.until(lambda:time.monotonic()>fault_at+.7,2.)
        self.assert_owner_retired(attempt,marker)
        self.assertFalse(result.done(),'gate should retain top-level RUNNING before resuming ticks')
        self.assertTrue(any(t>fault_at+.4 and j['state']=='RUNNING' for t,j in self.healths))
        self.assertFalse(any(j['state']=='FAULT_LATCHED' for _,j in self.healths))
        self.assertTrue(any(t>fault_at+.15 for t,_,_ in self.outputs))
        self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs if t>fault_at+.15))
        self.pause_file.unlink()
        self.assertEqual(self.future(result,4.).status,GoalStatus.STATUS_ABORTED)
        newclient,newhandle,newresult=self.send_top(6.)
        self.assertEqual(self.future(newresult,4.).status,GoalStatus.STATUS_ABORTED)
        self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs if t>fault_at+.15))
        (self.report/'navigator-tick-loss-result.json').write_text(json.dumps(dict(
            attempt=attempt,pause_at=marker,fault_at=fault_at,fault='owner_lease_expired',
            navigator_active_service_response=True,worker_healthy=True,
            resumed_top='ABORTED',new_goal='ABORTED',physical_motion=False),indent=2))
        client.destroy();newclient.destroy()


def load_tests(loader,tests,pattern):
    return unittest.TestSuite([NavigatorTickLoss('test_navigator_tick_loss')])


if __name__=='__main__':unittest.main()
