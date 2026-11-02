import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from ui7_current import resolve


class StableEntry(unittest.TestCase):
    def make_release(self,root,name):
        bundle=root/name;entry=bundle/'tools/starterbutton/start_navigation_high_sampling_neupan.sh'
        entry.parent.mkdir(parents=True);entry.write_text('#!/bin/bash\nexit 0\n')
        manifest=bundle/'releases/a-plus/manifest.json';manifest.parent.mkdir(parents=True)
        manifest.write_text(json.dumps(dict(protocol_set='A+context1-source2-grant1-health2-owner1',
            inventory_root=str(bundle),inventory=[dict(path=str(entry.relative_to(bundle)),sha256=hashlib.sha256(entry.read_bytes()).hexdigest())],
            artifacts={},build_source_sha=name,tooling_source_sha=name)))
        return dict(schema='r1_ui7_release_pointer_v1',bundle=str(bundle),manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest())

    def test_same_cached_entry_reads_new_pointer_each_click_and_rejects_tamper(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);a=self.make_release(root,'a');b=self.make_release(root,'b')
            pointer=root/'current.json';pointer.write_text(json.dumps(a))
            self.assertEqual(resolve(pointer)[1]['build_source_sha'],'a')
            pending=root/'next.json';pending.write_text(json.dumps(b));pending.replace(pointer)
            self.assertEqual(resolve(pointer)[1]['build_source_sha'],'b')
            resolve(pointer)[0].write_text('tampered')
            with self.assertRaisesRegex(ValueError,'inventory_mismatch'):resolve(pointer)


if __name__=='__main__':unittest.main()
