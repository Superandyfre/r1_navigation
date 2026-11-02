import threading
from unittest.mock import Mock
import pytest
from r1_neupan_integration.runtime_recovery import RecoveryTransaction

PHASES=['capture','revoke','verify_stopped','save_evidence','stop_old','start_new','verify_new_idle','enable_idle']


def test_recovery_orders_stop_before_new_identity_and_never_replays_goal():
    backend=Mock();runtime=RecoveryTransaction(backend)
    assert runtime.run()==(True,'waiting_new_goal')
    assert [call[0] for call in backend.method_calls]==PHASES
    backend.abort.assert_not_called()


@pytest.mark.parametrize('failed',PHASES)
def test_every_failed_stage_aborts_without_restart_loop(failed):
    backend=Mock();getattr(backend,failed).side_effect=RuntimeError('injected')
    runtime=RecoveryTransaction(backend)
    ok,message=runtime.run()
    assert not ok and 'injected' in message
    called=[call[0] for call in backend.method_calls]
    assert called==PHASES[:PHASES.index(failed)+1]+['abort']


def test_duplicate_and_cancel_do_not_start_second_recovery():
    backend=Mock();runtime=RecoveryTransaction(backend)
    entered=threading.Event();release=threading.Event();results=[]
    def blocked(*args):entered.set();assert release.wait(2)
    backend.capture.side_effect=blocked
    thread=threading.Thread(target=lambda:results.append(runtime.run()))
    thread.start();assert entered.wait(1)
    assert runtime.run()==(False,'recovery_already_running')
    runtime.cancelled.set();release.set();thread.join(2)
    assert not results[0][0]
    backend.start_new.assert_not_called()
    backend.abort.assert_called_once()


def test_owned_group_removes_child_after_leader_dies(tmp_path):
    import sys,time
    from r1_neupan_integration.runtime_recovery import OwnedProcessGroup
    with (tmp_path/'group.log').open('w') as log:
        command=[sys.executable,'-c',
                 'import subprocess,sys,time; subprocess.Popen([sys.executable,"-c","import time; time.sleep(30)"]); time.sleep(30)']
        group=OwnedProcessGroup(command,log)
        group.start()
        try:
            deadline=time.monotonic()+2
            while len(group.members())<3 and time.monotonic()<deadline:time.sleep(.02)
            assert len(group.members())==3
            group.process.kill();group.process.wait(timeout=1)
            assert group.members()  # orphan is still ours, not ignored
            group.stop(time.monotonic()+3,threading.Event())
            assert not group.members()
        finally:group.stop(time.monotonic()+3,threading.Event())
