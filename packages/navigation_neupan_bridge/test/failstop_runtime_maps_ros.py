#!/usr/bin/env python3
"""Inspect real isolated process maps against independently pinned candidate."""
import hashlib
import json
from pathlib import Path
import sys
import unittest
import failstop_chain_ros as chain
from failstop_mixed_version_ros import MixedVersion
sys.path.insert(0,str(chain.WS/'src/r1_navigation/tools/release'))
import runtime_maps


def verify(test, old_core=False):
    install=chain.WS/'install'
    files=dict(controller_server=chain.CONTROLLER,guard=chain.GUARD,
               controller_core=install/'nav2_controller/lib/libcontroller_server_core.so',
               controller_plugin=install/'navigation_neupan_bridge/lib/libnavigation_neupan_bridge.so')
    manifest=dict(artifacts={k:dict(path=str(p.resolve()),sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for k,p in files.items()})
    bindings={}
    for role,name in [('controller','controller_server'),('guard','r1_collision_guard_node')]:
        proc=next(p for p in test.procs if Path(p.args[0]).name==name)
        bindings[role]=dict(pid=proc.pid,start_ticks=runtime_maps.identity(proc.pid))
    result=runtime_maps.check(manifest,bindings)
    if old_core:
        test.assertFalse(result['ok']);test.assertIn('controller_core: loaded_path_mismatch',result['errors'])
    else:test.assertTrue(result['ok'],result)
    wrong=json.loads(json.dumps(bindings));wrong['controller']['start_ticks']='0'
    refused=runtime_maps.check(manifest,wrong)
    test.assertFalse(refused['ok']);test.assertIn('controller_core: process_identity_changed',refused['errors'])
    (test.report/'runtime-maps-result.json').write_text(json.dumps(dict(candidate=manifest,bindings=bindings,
        result=result,recycled_pid_refusal=refused,old_core=old_core,physical_motion=False),indent=2))


class CandidateMaps(chain.FailstopChainROS):
    def test_loaded_candidate(self):verify(self)

class OldCoreMaps(MixedVersion):
    def test_loaded_old_core(self):verify(self,True)


def load_tests(loader,tests,pattern):
    return unittest.TestSuite([CandidateMaps('test_loaded_candidate'),OldCoreMaps('test_loaded_old_core')])
if __name__=='__main__':unittest.main()
