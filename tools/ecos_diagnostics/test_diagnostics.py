"""Offline tests: raw solver identity, error capture, spawn activation, replay."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

TOOLS = Path(__file__).resolve().parent


class DiagnosticsTests(unittest.TestCase):
    def run_python(self, code, directory):
        env = os.environ.copy()
        env['PYTHONPATH'] = str(TOOLS)
        env['RPP_ECOS_DIAGNOSTICS_DIR'] = str(directory)
        return subprocess.run([sys.executable, '-c', code], env=env, capture_output=True, text=True, check=True)

    def test_real_ecos_success_and_failure_preserve_results(self):
        with tempfile.TemporaryDirectory() as directory:
            code = '''
import sys
assert 'numpy' not in sys.modules, 'diagnostic startup imported numpy before thread policy'
import ecos,numpy as np
from scipy.sparse import csc_matrix
assert ecos.solve._rpp_diagnostics
base=ecos.solve.__wrapped__
c=np.array([1.]);G=csc_matrix([[-1.]]);h=np.array([-1.]);dims={'l':1}
for options in [dict(verbose=False),dict(verbose=False,max_iters=1)]:
 expected=base(c,G,h,dims,**options)
 actual=ecos.solve(c,G,h,dims,**options)
 assert actual['info']['exitFlag']==expected['info']['exitFlag']
 assert np.array_equal(actual['x'],expected['x'])
'''
            self.run_python(code, directory)
            records=[json.loads(x) for p in Path(directory).glob('*.jsonl') for x in p.read_text().splitlines()]
            solves=[x for x in records if x['event']=='solve']
            self.assertEqual(len(solves), 2)
            self.assertEqual(solves[0]['info']['exitFlag'], 0)
            self.assertEqual(solves[1]['info']['exitFlag'], -1)
            self.assertIn('iter', solves[1]['info'])
            payload=Path(solves[1]['input_file'])
            self.assertTrue(payload.exists())
            replay=subprocess.run([sys.executable,str(TOOLS/'replay.py'),str(payload),'--verbose'],capture_output=True,text=True,check=True)
            self.assertIn('"exitFlag": -1',replay.stdout)

    def test_logging_failure_keeps_exception_and_does_not_retry(self):
        with tempfile.TemporaryDirectory() as directory:
            code='''
import ecos_diagnostics as d
class Module:
 __file__='fake'
 def __init__(self): self.calls=0
 def solve(self,*a,**kw):
  self.calls+=1
  raise ValueError('original solver exception')
m=Module();d.wrap_ecos(m)
d._emit=lambda *a: (_ for _ in ()).throw(OSError('disk failure'))
try:m.solve()
except ValueError as e:assert str(e)=='original solver exception'
else:raise AssertionError('original exception was lost')
assert m.calls==1
'''
            self.run_python(code, directory)

    def test_cvxpylayer_keeps_diffcp_error_and_records_raw_status(self):
        with tempfile.TemporaryDirectory() as directory:
            code = """
import torch,cvxpy as cp
from cvxpylayers.torch import CvxpyLayer
from diffcp.cone_program import SolverError
x=cp.Variable(1);p=cp.Parameter(1)
layer=CvxpyLayer(cp.Problem(cp.Minimize(cp.sum_squares(x-p)),[x>=0]),parameters=[p],variables=[x])
try:
 layer(torch.tensor([1.]),solver_args={'solve_method':'ECOS','max_iters':1})
except SolverError as error:
 assert str(error)=='Solver ecos errored.',str(error)
else:raise AssertionError('expected original diffcp failure')
"""
            self.run_python(code,directory)
            records=[json.loads(x) for p in Path(directory).glob('*.jsonl') for x in p.read_text().splitlines()]
            failure=next(x for x in records if x.get('failed'))
            self.assertEqual(failure['info']['exitFlag'],-1)
            self.assertTrue(Path(failure['input_file']).exists())

    def test_spawn_inherits_hook(self):
        with tempfile.TemporaryDirectory() as directory:
            script=Path(directory)/'spawn_test.py'
            script.write_text('''import multiprocessing as mp
import os

def worker():
 import ecos
 assert ecos.solve._rpp_diagnostics
 print('spawn hook active',flush=True)

if __name__=='__main__':
 p=mp.get_context('spawn').Process(target=worker)
 p.start();p.join(10)
 assert p.exitcode==0,p.exitcode
''')
            env=os.environ.copy();env.update(PYTHONPATH=str(TOOLS),RPP_ECOS_DIAGNOSTICS_DIR=directory)
            result=subprocess.run([sys.executable,str(script)],env=env,capture_output=True,text=True,check=True)
            self.assertIn('spawn hook active',result.stdout)

if __name__=='__main__':unittest.main()
