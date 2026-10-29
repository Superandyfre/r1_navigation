#!/usr/bin/env python3
"""Checks using actual local candidate and preserved production artifacts."""
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import preflight

WS=Path('/home/rpp/neupan_failstop_ws')
REPORT=Path(os.environ.get('NEUPAN_PREFLIGHT_REPORT_DIR', str(WS/'reports/neupan_failstop_20261027/tests')))
REPORT.mkdir(parents=True,exist_ok=True)

class Preflight(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Snapshot only the already tested development build, never the mixed
        # environment under test. This is diagnostic, with all release gates off.
        cls.env=dict(os.environ)
        cls.paths=list(sys.path)
        cls.manifest=dict(schema='r1_neupan_release_v1',protocol_set='A+context1-source2-grant1-health1',
            deployment_approved=False,build_testing=True,gates={},artifacts={})
        for role in preflight.ROLES:
            p=preflight.resolve(role,cls.env,cls.paths)
            assert str(p).startswith(str(WS)),(role,p)
            cls.manifest['artifacts'][role]=dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
        (REPORT/'wp7-diagnostic-pins-20261029.json').write_text(json.dumps(cls.manifest,indent=2))
        cls.results=[]

    @classmethod
    def tearDownClass(cls):
        (REPORT/'wp7-preflight-analysis-20261029.json').write_text(json.dumps(cls.results,indent=2))

    def run_check(self,label,env=None,paths=None,manifest=None,diagnostic=True):
        result=preflight.check(manifest or self.manifest,env or self.env,paths or self.paths,diagnostic)
        self.results.append(dict(case=label,**result));return result

    def test_candidate_consistent_but_not_deployable(self):
        self.assertTrue(self.run_check('candidate-compatible')['ok'])
        r=self.run_check('candidate-deployment-denied',diagnostic=False)
        self.assertFalse(r['ok']);self.assertIn('release_not_approved',r['errors'])

    def test_actual_old_core(self):
        env=dict(self.env);env['LD_LIBRARY_PATH']='/home/rpp/rpp_ws/install/nav2_controller/lib:'+env['LD_LIBRARY_PATH']
        r=self.run_check('actual-old-core',env=env)
        self.assertFalse(r['ok']);self.assertIn('controller_core: resolved_path_mismatch',r['errors'])

    def test_actual_old_plugin(self):
        env=dict(self.env);env['LD_LIBRARY_PATH']='/home/rpp/r1_navigation/install/navigation_neupan_bridge/lib:'+env['LD_LIBRARY_PATH']
        r=self.run_check('actual-old-plugin',env=env)
        self.assertFalse(r['ok']);self.assertIn('controller_plugin: resolved_path_mismatch',r['errors'])

    def test_actual_old_guard(self):
        env=dict(self.env);env['AMENT_PREFIX_PATH']='/home/rpp/r1_navigation/install/r1_collision_guard:'+env['AMENT_PREFIX_PATH']
        r=self.run_check('actual-old-guard',env=env)
        self.assertFalse(r['ok']);self.assertIn('guard: resolved_path_mismatch',r['errors'])

    def test_actual_old_python(self):
        r=self.run_check('actual-old-python',paths=['/home/rpp/rpp_ws/src/application/neupan_migration/r1_neupan_integration']+self.paths)
        self.assertFalse(r['ok']);self.assertIn('worker_node: resolved_path_mismatch',r['errors'])

    def test_missing_role_and_changed_hash(self):
        pins=copy.deepcopy(self.manifest);pins['artifacts'].pop('guard')
        self.assertFalse(self.run_check('missing-guard',manifest=pins)['ok'])
        pins=copy.deepcopy(self.manifest);pins['artifacts']['worker_node']['sha256']='0'*64
        r=self.run_check('changed-worker',manifest=pins)
        self.assertIn('worker_node: hash_mismatch',r['errors'])

    def test_denied_command_never_executed(self):
        with tempfile.TemporaryDirectory() as directory:
            sentinel=Path(directory)/'executed'
            command=[sys.executable,str(Path(preflight.__file__)),'--manifest',str(REPORT/'wp7-diagnostic-pins-20261029.json'),
                     '--',sys.executable,'-c','from pathlib import Path; Path('+repr(str(sentinel))+').touch()']
            p=subprocess.run(command,stdout=subprocess.PIPE,text=True)
            self.assertEqual(p.returncode,2);self.assertFalse(sentinel.exists())
            command.insert(2,'--compatibility-only')
            p=subprocess.run(command,stdout=subprocess.PIPE,text=True)
            self.assertEqual(p.returncode,2);self.assertFalse(sentinel.exists())

if __name__=='__main__':unittest.main()
