#!/usr/bin/env python3
"""Real mmap identity checks; file replacement affects temporary copies only."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
import runtime_maps

class RuntimeMaps(unittest.TestCase):
    def test_mapped_file_replaced(self):
        with tempfile.TemporaryDirectory() as directory:
            library=Path(directory)/'libprobe.so'
            shutil.copyfile('/usr/lib/x86_64-linux-gnu/libm.so.6',library)
            code='import ctypes,sys; lib=ctypes.CDLL(sys.argv[1]); print("ready",flush=True); sys.stdin.read()'
            process=subprocess.Popen([sys.executable,'-c',code,str(library)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
            try:
                self.assertEqual(process.stdout.readline().strip(),'ready')
                pins={role:dict(path=str(p.resolve()),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
                      for role,p in dict(controller_server=Path(sys.executable),guard=Path(sys.executable),
                                         controller_core=library,controller_plugin=library).items()}
                bindings={key:dict(pid=process.pid,start_ticks=runtime_maps.identity(process.pid)) for key in ['controller','guard']}
                good=runtime_maps.check(dict(artifacts=pins),bindings)
                self.assertTrue(good['ok'],good)
                # Same pathname and identical bytes but new inode: running
                # process still maps the now deleted old object. Must refuse.
                replacement=Path(directory)/'replacement.so';shutil.copyfile(library,replacement)
                replacement.replace(library)
                bad=runtime_maps.check(dict(artifacts=pins),bindings)
                self.assertFalse(bad['ok']);self.assertIn('controller_core: mapped_file_deleted',bad['errors'])
            finally:
                process.communicate('',timeout=5)
            dead=runtime_maps.check(dict(artifacts=pins),bindings)
            self.assertFalse(dead['ok'])

if __name__=='__main__':unittest.main()
