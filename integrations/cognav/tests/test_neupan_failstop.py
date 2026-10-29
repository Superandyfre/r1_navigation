import unittest
from robot_live.neupan_failstop import NeuPANFailstop


def messages(now=100., seq=1):
    return dict(worker=dict(schema='r1_neupan_worker_health_v1',boot_id='worker',sequence=seq,
                            monotonic=now,state='STARTING',worker_generation=7),
                controller=dict(schema='r1_neupan_execution_status_v1',controller_boot_id='controller',
                                sequence=seq,monotonic=now,plugin_active=True,fault_latched=False),
                guard=dict(schema='r1_collision_guard_status_v1',guard_boot_id='guard',sequence=seq,
                           monotonic=now,authorization_protocol='r1_neupan_motion_grant_v1',
                           source_protocol='r1_neupan_planner_source_v2',authorization_state='DISARMED'))

class GateTests(unittest.TestCase):
    def ready(self):
        gate=NeuPANFailstop()
        for kind,row in messages().items():gate.observe(kind,row,100.)
        return gate
    def test_missing_and_fresh_lazy_worker(self):
        self.assertTrue(NeuPANFailstop().reason(100.))
        self.assertEqual(self.ready().reason(100.),'')
    def test_duplicate_cannot_refresh_or_wash_out_fatal(self):
        gate=self.ready()
        for kind,row in messages(100.4).items():gate.observe(kind,row,100.4)
        self.assertTrue(gate.reason(100.6))
        fatal=messages(100.1,2)['worker'];fatal.update(state='FAULT_LATCHED',fault_id='f',fault_code='timeout')
        gate.observe('worker',fatal,100.1)
        for kind,row in messages(100.2,3).items():gate.observe(kind,row,100.2)
        self.assertIn('worker_fatal',gate.reason(100.2))
    def test_each_boot_change_is_latched(self):
        for kind in messages():
            gate=self.ready();row=messages(100.1,2)[kind];row[gate.BOOTS[kind]]='new'
            gate.observe(kind,row,100.1)
            self.assertEqual(gate.reason(100.1),kind+'_boot_changed')
    def test_controller_and_guard_fatal_latched(self):
        for kind in ['controller','guard']:
            gate=self.ready();row=messages(100.1,2)[kind]
            if kind=='controller':row['fault_latched']=True
            else:row['authorization_state']='FAULT_LATCHED'
            gate.observe(kind,row,100.1)
            for k,r in messages(100.2,3).items():gate.observe(k,r,100.2)
            self.assertIn(kind+'_fatal',gate.reason(100.2))
    def test_source_hold_keeps_heartbeat_without_latching_or_cancelling(self):
        gate=self.ready();row=messages(100.4,2)['guard']
        row['authorization_state']='SOURCE_STALE_HOLD'
        gate.observe('guard',row,100.4)
        for kind in ['worker','controller']:gate.observe(kind,messages(100.4,2)[kind],100.4)
        self.assertEqual(gate.reason(100.6),'')
        self.assertFalse(gate.fault)
    def test_generation_change_denied(self):
        gate=self.ready();row=messages(100.1,2)['worker'];row['worker_generation']=8
        gate.observe('worker',row,100.1)
        self.assertEqual(gate.reason(100.1),'worker_generation_changed')
    def test_malformed_future_and_wrong_protocol_never_create_readiness(self):
        for update in [dict(monotonic=float('nan')),dict(monotonic=101.),dict(sequence=True),dict(boot_id='')]:
            gate=NeuPANFailstop();row=messages()['worker'];row.update(update)
            gate.observe('worker',row,100.);self.assertNotIn('worker',gate.records)
        for row in [None, [], 'invalid', 1]:
            gate=NeuPANFailstop();gate.observe('worker',row,100.)
            self.assertFalse(gate.records)
        gate=self.ready();gate.observe('controller',{'schema':'r1_neupan_controller_status_v1'},100.1)
        self.assertEqual(gate.reason(100.1),'')
        self.assertTrue(gate.reason(100.6))

if __name__=='__main__':unittest.main()
