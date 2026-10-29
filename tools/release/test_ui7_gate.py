#!/usr/bin/env python3
"""Run the real gate helper with isolated lock-free sentinel, no ROS launch."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]
REPORT=Path('/home/rpp/neupan_failstop_ws/reports/neupan_failstop_20261027/tests')

class UI7Gate(unittest.TestCase):
    def invoke(self,manifest):
        with tempfile.TemporaryDirectory() as directory:
            sentinel=Path(directory)/'launch-reached'
            env=dict(os.environ);env['RPP_NEUPAN_RELEASE_MANIFEST']=str(manifest)
            # Positional args avoid evaluating paths as shell source code.
            command=['/bin/bash','-c',
                'source "$1/tools/starterbutton/release_gate.sh"; neupan_release_gate "$1" || exit $?; touch "$2"',
                'test-ui7',str(ROOT),str(sentinel)]
            result=subprocess.run(command,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
            self.assertEqual(result.returncode,2,result.stdout)
            self.assertFalse(sentinel.exists())
            return result.stdout

    def test_missing_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            output=self.invoke(Path(directory)/'missing.json')
            self.assertIn('reviewed manifest/checker missing',output)

    def test_development_manifest_cannot_launch(self):
        output=self.invoke(REPORT/'wp7-diagnostic-pins-20261029.json')
        self.assertIn('release_not_approved',output)
        self.assertIn('production_requires_BUILD_TESTING_OFF',output)

    def test_bad_manifest_cannot_launch(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest=Path(directory)/'bad.json';manifest.write_text('{bad')
            self.invoke(manifest)

    def test_gate_precedes_process_creation_in_candidate(self):
        text=(ROOT/'tools/starterbutton/start_navigation_high_sampling_neupan.sh').read_text()
        gate=text.index('neupan_release_gate "${navigation_repo}" || exit $?')
        self.assertLess(gate,text.index('/usr/bin/python3 "${script_dir}/launch_with_parent_death.py"'))
        self.assertLess(gate,text.index('xwayland_args='))
        # Never ship a diagnostic-mode bypass through this entry.
        self.assertNotIn('--compatibility-only',text)

if __name__=='__main__':unittest.main()
