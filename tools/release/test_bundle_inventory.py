#!/usr/bin/env python3
"""Validate real review bundle without modifying any payload file."""
import copy
import json
from pathlib import Path
import unittest
import preflight

BUNDLE=Path('/home/rpp/neupan_failstop_ws/release_bundles/20261029-a-plus-01')
class BundleInventory(unittest.TestCase):
    def setUp(self):self.manifest=json.loads((BUNDLE/'releases/a-plus/manifest.json').read_text())
    def test_relocated_payload_compatible_but_not_approved(self):
        result=preflight.check(self.manifest,compatibility_only=True)
        self.assertTrue(result['ok'],result['errors'])
        result=preflight.check(self.manifest)
        self.assertFalse(result['ok']);self.assertIn('release_not_approved',result['errors'])
    def test_non_core_file_hash_refused(self):
        row=next(r for r in self.manifest['inventory'] if r['path'].startswith('third_party/NeuPAN/neupan/') and r['path'].endswith('.py'))
        row['sha256']='0'*64
        result=preflight.check(self.manifest,compatibility_only=True)
        self.assertFalse(result['ok']);self.assertIn('inventory_hash_mismatch:'+row['path'],result['errors'])
    def test_inventory_escape_refused(self):
        self.manifest['inventory']=[dict(path='/etc/hosts',sha256='0'*64)]
        result=preflight.check(self.manifest,compatibility_only=True)
        self.assertFalse(result['ok']);self.assertIn('inventory:inventory_path_escape',result['errors'])
if __name__=='__main__':unittest.main()
