import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from robot_live.shutdown import finish_bridge


class ShutdownOrder(unittest.TestCase):
    def exercise(self, fail_client=False, context=True):
        events=[]
        with tempfile.TemporaryDirectory() as root:
            (Path(root)/'live').mkdir()
            def destroy_client():
                events.append('action')
                if fail_client:
                    raise RuntimeError('injected action cleanup failure')
            node=SimpleNamespace(session='test', root=Path(root),
                a=SimpleNamespace(input_max_age=.5), motion_payload={},
                lifecycle=SimpleNamespace(result=lambda:None, current=None,
                    busy=lambda:False, cancel=lambda reason:events.append('cancel')),
                client=SimpleNamespace(destroy=destroy_client),
                destroy_node=lambda:events.append('node'))
            executor=SimpleNamespace(remove_node=lambda node:events.append('remove'),
                shutdown=lambda **kw:events.append('executor'))
            ros=SimpleNamespace(ok=lambda:context, shutdown=lambda:events.append('context'))
            report=finish_bridge(node,executor,ros)
            self.assertEqual(events[:5],['cancel','remove','executor','action','node'])
            self.assertEqual(events[5:],['context'] if context else [])
            self.assertFalse(report['action_terminal_received'])
            self.assertFalse(report['measured_stopped'])
            return report

    def test_action_destroyed_before_node_and_context(self):
        self.assertFalse(self.exercise()['cleanup_errors'])

    def test_action_error_does_not_prevent_node_or_context_cleanup(self):
        report=self.exercise(fail_client=True)
        self.assertEqual([e['step'] for e in report['cleanup_errors']],['destroy_action_client'])

    def test_lost_context_still_attempts_action_and_node_cleanup(self):
        self.assertFalse(self.exercise(context=False)['cleanup_errors'])


if __name__=='__main__':unittest.main()
