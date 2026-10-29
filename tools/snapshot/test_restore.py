import hashlib,importlib.util,io,json,sys,tarfile,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'restore'))
from restore import verify,stage

class RestoreChecks(unittest.TestCase):
    def fixture(self,root,unsafe=False):
        archive=root/'payload.tar.gz'
        with tarfile.open(archive,'w:gz') as tar:
            data=b'robot configuration';info=tarfile.TarInfo('../escape' if unsafe else 'config/robot.yaml');info.size=len(data);tar.addfile(info,io.BytesIO(data))
        m=dict(id='test',status=dict(navigation_validated=False),artifacts=[dict(path=archive.name,sha256=hashlib.sha256(archive.read_bytes()).hexdigest())])
        (root/'manifest.yaml').write_text(json.dumps(m))
        (root/'checksums.sha256').write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+p.name+'\n' for p in [archive,root/'manifest.yaml']))
        return m
    def test_verify_then_stage_without_deployment(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);m=self.fixture(root);self.assertEqual(verify(root)[1],2)
            result=stage(root,m,'payload.tar.gz',root/'stage')
            self.assertFalse(result['deployed']);self.assertEqual((root/'stage/config/robot.yaml').read_bytes(),b'robot configuration')
            with self.assertRaises(ValueError):stage(root,m,'payload.tar.gz',root/'stage')
    def test_corruption_refused(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);self.fixture(root);(root/'payload.tar.gz').write_bytes(b'corrupted')
            with self.assertRaises(ValueError):verify(root)
    def test_archive_escape_refused_before_any_extraction(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);m=self.fixture(root,True)
            with self.assertRaises(ValueError):stage(root,m,'payload.tar.gz',root/'stage')
            self.assertFalse((root/'stage').exists())
if __name__=='__main__':unittest.main()
