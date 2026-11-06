#!/usr/bin/env python3
"""Create a build-only inventory; a hash list does not approve deployment."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

CMAKE_PACKAGES = {"r1_neupan_runtime", "r1_lidar_preprocess", "r1_collision_guard",
                  "navigation_2d", "nav2_controller", "navigation_neupan_bridge"}
PYTHON_PACKAGES = {"neupan_ros2", "r1_neupan_integration"}

def inventory(root):
    install=root/"install"
    sha=(root/"source-sha.txt").read_text().strip()
    if len(sha)!=40 or any(c not in "0123456789abcdef" for c in sha):
        raise ValueError("invalid_source_sha")
    if not install.is_dir(): raise ValueError("missing_install")
    links=[str(p.relative_to(install)) for p in install.rglob("*") if p.is_symlink()]
    if links: raise ValueError("symlink_in_install:"+links[0])
    adapters=[str(p.relative_to(install)) for pat in ("*same_uuid*","*action_stall*","*controlled_current*")
              for p in install.rglob(pat)]
    if adapters: raise ValueError("test_adapter_in_install:"+adapters[0])
    caches=list((root/"build").glob("*/CMakeCache.txt"))
    flags={p.parent.name:"BUILD_TESTING:BOOL=OFF" in p.read_text() for p in caches}
    if set(flags)!=CMAKE_PACKAGES or not all(flags.values()): raise ValueError("cmake_build_testing_not_off")
    for package in CMAKE_PACKAGES | PYTHON_PACKAGES:
        if not (install/package).is_dir(): raise ValueError("missing_package:"+package)
    files=[dict(path=str(p.relative_to(install)),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
           for p in sorted(install.rglob("*")) if p.is_file()]
    if not files: raise ValueError("empty_install")
    return dict(source_sha=sha,install=str(install),installed_files=len(files),symlinks=[],
                test_adapters=[],build_testing_off=flags,deployment_approved=False,files=files)

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("candidate",type=Path)
    args=parser.parse_args(); root=args.candidate.resolve(strict=True)
    repo=Path(__file__).resolve().parents[2]
    sha=subprocess.check_output(["git","rev-parse","HEAD"],cwd=repo,text=True).strip()
    if subprocess.check_output(["git","status","--porcelain"],cwd=repo,text=True).strip():
        raise ValueError("dirty_source")
    result=inventory(root)
    if result["source_sha"]!=sha: raise ValueError("source_revision_changed")
    with (root/"candidate-inventory.json").open("x") as f:
        json.dump(result,f,indent=2);f.write("\n")
    print(json.dumps(dict(installed_files=len(result["files"]),source_sha=sha,deployment_approved=False)))

if __name__=="__main__":main()
