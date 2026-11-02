import pickle
import tempfile
from unittest.mock import Mock

import numpy as np
import pytest
from diffcp.cone_program import SolverError
from r1_neupan_integration.solver_status import EcosOutcome, SolveBudget, RecoverableSolveError, RECOVERY_PROTOCOL
from r1_neupan_integration.worker_health import WorkerHealth
from r1_neupan_integration.n1_16_bridge import ProductionWholeCoreClient, BridgeProtocolError


def test_classification_requires_actual_iteration_limit_evidence():
    observer = EcosOutcome()
    error = SolverError('Solver ecos errored.')
    assert not observer.numerical_failure(error)
    observer.info = dict(exitFlag=-1, pres=1e-5, dres=1e-9, gap=4)
    assert observer.numerical_failure(error)
    assert not observer.numerical_failure(ValueError(str(error)))
    for flag in [-2, -7, 1, 2, 10]:
        observer.info['exitFlag'] = flag
        assert not observer.numerical_failure(error)
    observer.info.update(exitFlag=-1, gap=float('nan'))
    assert not observer.numerical_failure(error)
    observer.begin()
    assert not observer.numerical_failure(error)


def test_retry_budget_not_refreshed_by_success_or_route_revision():
    budget = SolveBudget()
    budget.bind(('boot', 'attempt'))
    assert not budget.failure()
    budget.success()
    budget.bind(('boot', 'attempt'))
    assert budget.failure()
    budget.bind(('boot', 'new-attempt'))
    assert not budget.failure(now=100)
    assert budget.expired(now=101)


def test_failure_health_does_not_fake_success_or_unlock_fatal():
    health = WorkerHealth()
    health.bind_execution('a', 1, 2, 3)
    health.ready(1)
    health.success(5)
    health.planning_failure(6, False)
    row = health.snapshot()
    assert row['planner_state'] == 'retry'
    assert row['last_successful_request_id'] == 5
    assert row['state'] == 'RUNNING'
    health.success(7)
    assert health.snapshot()['planner_failed_request_id'] == 6
    fault = health.latch('worker_crash', 1)
    health.planning_failure(8, False)
    health.bind_execution('b', 1, 3, 4)
    health.success(9)
    assert health.fault == fault
    assert health.snapshot()['state'] == 'FAULT_LATCHED'


def client_response(client, **changes):
    row = dict(type='response', worker_generation=client.worker_generation,
               request_id=1, cycle_id=2, scan_counter=3, path_version=4, goal_version=5,
               worker_sequence=0, recovery_protocol=RECOVERY_PROTOCOL,
               status='recoverable_solver_failure', outputs={}, finite=False,
               solver_failure=dict(exitFlag=-1, pres=1e-5,dres=1e-9,gap=4,iter=100))
    row.update(changes)
    client.process = Mock()
    client.process.is_alive.return_value = True
    client.parent_conn = Mock()
    client.parent_conn.poll.return_value = True
    client.parent_conn.recv_bytes.return_value = pickle.dumps(row)


def solve(client):
    return client.solve(np.zeros((3,1)),np.empty((2,0)),np.zeros((4,2)),4,3,1.,
                        goal_version=5,cycle_id=2,execution_key=('boot','a'))


def test_full_ipc_failure_response_is_recoverable_without_quarantine():
    with tempfile.TemporaryDirectory() as root:
        client = ProductionWholeCoreClient(root,'cfg','model',{},1)
        client_response(client)
        with pytest.raises(RecoverableSolveError): solve(client)
        assert client.health.fault is None
        assert not client._quarantined
        assert client.requests_completed == 1
        assert client._reset_solver_state


@pytest.mark.parametrize('changes',[
    dict(request_id=99),dict(worker_generation=0),dict(recovery_protocol='old'),
    dict(outputs={'action': {}}),dict(finite=True),
    dict(solver_failure=dict(exitFlag=-2)),dict(status='exception'),
])
def test_malformed_or_fatal_response_never_degraded(changes):
    with tempfile.TemporaryDirectory() as root:
        client=ProductionWholeCoreClient(root,'cfg','model',{},1)
        client._quarantine_worker=Mock()
        client_response(client,**changes)
        with pytest.raises(BridgeProtocolError):solve(client)
        assert client.health.fault is not None
