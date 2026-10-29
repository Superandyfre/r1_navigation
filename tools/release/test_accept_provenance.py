import json
from pathlib import Path
import tempfile
import unittest
from accept_candidate_ros import verify_solver_provenance


class SolverProvenance(unittest.TestCase):
    def test_restart_cannot_fall_back_to_external_solver(self):
        with tempfile.TemporaryDirectory() as root:
            base=Path(root);bundle=base/'bundle';report=base/'report'
            for dirname,solver in [('worker',bundle/'third_party/NeuPAN'),('new-worker',base/'external')]:
                path=report/dirname;path.mkdir(parents=True)
                (path/'summary.json').write_text(json.dumps({'nested':{'neupan_source_root':str(solver)}}))
            errors=verify_solver_provenance(report,bundle)
            self.assertEqual(len(errors),1);self.assertIn('new-worker/summary.json',errors[0])

    def test_matching_solver_and_installed_worker_accepted(self):
        with tempfile.TemporaryDirectory() as root:
            bundle=Path(root)/'bundle';report=Path(root)/'report';report.mkdir()
            (report/'summary.json').write_text(json.dumps({'nested':[{
                'neupan_source_root':str(bundle/'third_party/NeuPAN'),
                'integration_source':str(bundle/'install/worker/core.py')}]}))
            self.assertEqual(verify_solver_provenance(report,bundle),[])

    def test_missing_or_external_worker_provenance_refused(self):
        with tempfile.TemporaryDirectory() as root:
            report=Path(root);bundle=report/'bundle'
            self.assertIn('missing_solver_provenance',verify_solver_provenance(report,bundle)[0])
            (report/'summary.json').write_text(json.dumps({
                'neupan_source_root':str(bundle/'third_party/NeuPAN'),
                'integration_source':str(report/'external/core.py')}))
            self.assertIn('wrong_worker',verify_solver_provenance(report,bundle)[0])


if __name__=='__main__':unittest.main()
