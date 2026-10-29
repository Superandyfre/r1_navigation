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

    def test_ui7_does_not_require_release_approval(self):
        text=(ROOT/'tools/starterbutton/start_navigation_high_sampling_neupan.sh').read_text()
        self.assertNotIn('neupan_release_gate',text)
        self.assertNotIn('source "${script_dir}/release_gate.sh"',text)
        self.assertIn('single_instance_or_exit',text)
        self.assertIn('ros2 pkg prefix',text)
        self.assertNotIn('--compatibility-only',text)

if __name__=='__main__':unittest.main()
