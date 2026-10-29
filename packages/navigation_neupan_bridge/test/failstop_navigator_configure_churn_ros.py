#!/usr/bin/env python3
"""Twelve fresh full-peer fixtures; Navigator configure only, never a goal."""
import json
import unittest
from lifecycle_msgs.srv import GetState
from failstop_navigator_ros import NavigatorFailstop


class NavigatorConfigureChurn(NavigatorFailstop):
    def lifecycle(self,name,transition):
        # Keep the original peer, fixture topology and configure client timing.
        # The comparison removes only Navigator activation and all goal traffic.
        if name=='bt_navigator' and transition==3:return
        try:return super().lifecycle(name,transition)
        except Exception:
            if name=='bt_navigator' and transition==1:
                record={'configure_response_failed':True,'transition_retried':False}
                try:
                    query=self.node.create_client(GetState,'/bt_navigator/get_state')
                    self.until(query.service_is_ready,2.)
                    record['state_after_failed_response']=self.future(
                        query.call_async(GetState.Request()),2.).current_state.id
                except Exception as exc:record['query_error']=repr(exc)
                (self.report/'configure-failed-state.json').write_text(json.dumps(record,indent=2))
            raise

    def test_configure_only(self):
        service=self.node.create_client(GetState,'/bt_navigator/get_state')
        self.until(service.service_is_ready)
        state=self.future(service.call_async(GetState.Request()))
        self.assertEqual(state.current_state.id,2)
        self.assertFalse(self.contexts)
        self.assertFalse(self.sources)
        self.assertFalse(self.grants)
        self.assertTrue(self.outputs)
        self.assertFalse(any(v!=0. or w!=0. for _,v,w in self.outputs))
        (self.report/'configure-only-result.json').write_text(json.dumps(dict(
            navigator_state='INACTIVE',configured=True,activated=False,goals_sent=0,
            all_outputs_zero=True,solver_not_started=True,physical_motion=False),indent=2))


def load_tests(loader,tests,pattern):
    # Predeclared sample count. Failures remain failures; never retry a transition.
    return unittest.TestSuite(NavigatorConfigureChurn('test_configure_only') for _ in range(12))


if __name__=='__main__':unittest.main()
