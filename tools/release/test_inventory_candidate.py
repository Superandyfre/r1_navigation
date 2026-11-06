import tempfile
import unittest
from pathlib import Path
from inventory_candidate import inventory,CMAKE_PACKAGES,PYTHON_PACKAGES

class CandidateInventory(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);(self.root/"source-sha.txt").write_text("a"*40)
        for name in CMAKE_PACKAGES|PYTHON_PACKAGES:
            (self.root/"install"/name).mkdir(parents=True)
            (self.root/"install"/name/"file").write_text(name)
        for name in CMAKE_PACKAGES:
            path=self.root/"build"/name;path.mkdir(parents=True)
            (path/"CMakeCache.txt").write_text("BUILD_TESTING:BOOL=OFF\n")
    def test_hashes_all_packages_without_approval(self):
        data=inventory(self.root)
        self.assertEqual(data["installed_files"],9)
        self.assertFalse(data["deployment_approved"])
    def test_rejects_test_build(self):
        (self.root/"build/nav2_controller/CMakeCache.txt").write_text("BUILD_TESTING:BOOL=ON\n")
        with self.assertRaises(ValueError):inventory(self.root)
    def test_rejects_links_and_test_adapters(self):
        (self.root/"install/link").symlink_to("/tmp")
        with self.assertRaises(ValueError):inventory(self.root)
        (self.root/"install/link").unlink()
        (self.root/"install/action_stall_controller").write_text("")
        with self.assertRaises(ValueError):inventory(self.root)
if __name__=="__main__":unittest.main()
