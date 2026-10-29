#!/usr/bin/env python3
"""Real installed owner gate: replay, wrong identities, sequence/owner conflicts."""
import json
import time
import unittest
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
import failstop_chain_ros as chain


class OwnerProtocol(chain.FailstopChainROS):
    def moving(self):
        handle=self.goal();result=handle.get_result_async()
        self.until(lambda:any(v>.04 for _,v,_ in self.outputs) or result.done(),50)
        self.assertFalse(result.done(),'no initial executable grant')
        self.owner_enabled=False
        self.until(lambda:bool(self.owners),1.)
        packet=dict(self.owners[-1][1])
        self.assertEqual(packet['navigation_attempt_id'],bytes(handle.goal_id.uuid).hex())
        return result,packet

    def send_owner(self,packet):
        self.owner_pub.publish(String(data=json.dumps(packet)))

    def verify_latched(self,result,reason,marker):
        self.assertEqual(self.future(result,3.).status,GoalStatus.STATUS_ABORTED)
        self.until(lambda:any(j.get('fault_reason')==reason for _,j in self.runtime_statuses))
        terminal=time.monotonic()
        self.owner_enabled=True
        retry=self.goal()
        self.assertEqual(self.future(retry.get_result_async()).status,GoalStatus.STATUS_ABORTED)
        self.until(lambda:time.monotonic()>terminal+.6)
        self.assertFalse(any(v!=0. or w!=0. for t,v,w in self.outputs if t>terminal+.15))
        self.assertFalse(any(j['state']=='FAULT_LATCHED' for _,j in self.healths))
        (self.report/'owner-protocol-result.json').write_text(json.dumps(dict(
            fault=reason,action='ABORTED',new_goal='ABORTED',
            observed_terminal_after=terminal-marker,worker_fatal=False,physical_motion=False),indent=2))

    def test_exact_replay_cannot_renew(self):
        result,packet=self.moving();marker=time.monotonic()
        while not result.done() and time.monotonic()<marker+2.:
            self.send_owner(packet);self.tick()
        self.verify_latched(result,'owner_lease_expired',marker)

    def test_fresh_wrong_boot_or_attempt_cannot_renew(self):
        result,packet=self.moving();marker=time.monotonic();sequence=10**12
        while not result.done() and time.monotonic()<marker+2.:
            for key in ('authority_boot_id','controller_boot_id','navigation_attempt_id'):
                message=dict(packet,sequence=sequence,monotonic=time.monotonic())
                message[key]='retired-'+message[key];sequence+=1
                self.send_owner(message)
            self.tick()
        self.verify_latched(result,'owner_lease_expired',marker)

    def test_same_sequence_conflict_latches(self):
        result,packet=self.moving();marker=time.monotonic()
        packet.update(sequence=10**12,monotonic=time.monotonic())
        self.send_owner(packet)
        # Same publisher orders both packets; the repeated sequence has a
        # different original timestamp, not an arrival-time refresh.
        self.tick();self.tick()
        packet['monotonic']=time.monotonic();self.send_owner(packet)
        self.verify_latched(result,'owner_sequence_conflict',marker)

    def test_changed_owner_latches(self):
        result,packet=self.moving();marker=time.monotonic()
        packet.update(owner_id='different-client',sequence=10**12,monotonic=time.monotonic())
        self.send_owner(packet)
        self.verify_latched(result,'owner_identity_conflict',marker)


def load_tests(loader,tests,pattern):
    return unittest.TestSuite(OwnerProtocol(name) for name in (
        'test_exact_replay_cannot_renew','test_fresh_wrong_boot_or_attempt_cannot_renew',
        'test_same_sequence_conflict_latches','test_changed_owner_latches'))


if __name__=='__main__':unittest.main()
