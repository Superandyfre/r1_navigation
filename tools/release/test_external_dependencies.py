import json
from pathlib import Path
import tempfile
import unittest
from external_dependencies import collect,check


class ExternalDependencies(unittest.TestCase):
    def test_observed_external_file_pinned_and_change_refused(self):
        with tempfile.TemporaryDirectory() as root:
            base=Path(root);bundle=base/'bundle';bundle.mkdir();report=base/'report';report.mkdir()
            external=base/'runtime.so';external.write_bytes(b'old')
            (report/'maps-1.txt').write_text('1-2 r-xp 0 00:00 1 '+str(external)+'\n')
            snapshot=collect([report],bundle)
            self.assertEqual(len(snapshot['files']),1);self.assertTrue(check(snapshot)['ok'])
            self.assertFalse(snapshot['review_complete']);self.assertFalse(check(snapshot)['motion_authorized'])
            external.write_bytes(b'new');self.assertFalse(check(snapshot)['ok'])

    def test_symlink_retarget_and_missing_observation_refused(self):
        with tempfile.TemporaryDirectory() as root:
            base=Path(root);bundle=base/'bundle';bundle.mkdir();report=base/'report';report.mkdir()
            a=base/'a';b=base/'b';link=base/'python';a.write_bytes(b'same');b.write_bytes(b'same');link.symlink_to(a)
            (report/'summary.json').write_text(json.dumps({'python_executable':str(link)}))
            snapshot=collect([report],bundle);link.unlink();link.symlink_to(b)
            self.assertFalse(check(snapshot)['ok'])
            (report/'summary.json').write_text(json.dumps({'python_executable':str(base/'absent')}))
            snapshot=collect([report],bundle);self.assertTrue(snapshot['missing']);self.assertFalse(check(snapshot)['ok'])

    def test_bundle_payload_excluded_and_nested_runtime_included(self):
        with tempfile.TemporaryDirectory() as root:
            base=Path(root);bundle=base/'bundle';bundle.mkdir();internal=bundle/'core';internal.write_bytes(b'code')
            external=base/'blas.so';external.write_bytes(b'blas');report=base/'report';report.mkdir()
            (report/'summary.json').write_text(json.dumps({'nested':[{'filepath':str(external)}]}))
            (report/'binaries.json').write_text(json.dumps([{'path':str(internal)}]))
            snapshot=collect([report],bundle)
            self.assertEqual([r['path'] for r in snapshot['files']],[str(external)])

    def test_transient_data_classification_does_not_hide_executable_mapping(self):
        with tempfile.TemporaryDirectory() as root:
            base=Path(root);bundle=base/'bundle';bundle.mkdir();report=base/'report';report.mkdir()
            path='/dev/shm/fastrtps_nonexistent_dependency_unit'
            (report/'maps-1.txt').write_text('1-2 rw-s 0 00:00 1 '+path+'\n')
            snapshot=collect([report],bundle)
            self.assertEqual(len(snapshot['transient_data_mappings']),1)
            self.assertFalse(snapshot['missing'])
            (report/'maps-1.txt').write_text('1-2 r-xs 0 00:00 1 '+path+'\n')
            snapshot=collect([report],bundle)
            self.assertFalse(snapshot['transient_data_mappings'])
            self.assertTrue(snapshot['missing']);self.assertFalse(check(snapshot)['ok'])
            (report/'maps-1.txt').unlink()
            (report/'binaries.json').write_text(json.dumps([{'path':path}]))
            snapshot=collect([report],bundle)
            self.assertTrue(snapshot['missing']);self.assertFalse(snapshot['transient_data_mappings'])


if __name__=='__main__':unittest.main()
