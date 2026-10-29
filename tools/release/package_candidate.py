#!/usr/bin/env python3
"""Copy a pinned candidate into a reviewable UI7 layout; approval stays false."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
import preflight


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def package(candidate, destination, repo):
    if destination.exists():raise ValueError('destination_already_exists')
    inv=json.loads((candidate/'candidate-inventory.json').read_text())
    if not inv['build_testing_off'] or not all(inv['build_testing_off'].values()):raise ValueError('testing_not_disabled')
    for item in inv['files']:
        path=candidate/'install'/item['path']
        if not path.resolve().is_relative_to(candidate/'install') or digest(path)!=item['sha256']:
            raise ValueError('candidate_inventory_mismatch:'+item['path'])
    if any(p.is_symlink() for p in (candidate/'install').rglob('*')):raise ValueError('install_contains_symlinks')
    for pattern in ['*same_uuid*','*action_stall*','*controlled_current*']:
        if list((candidate/'install').rglob(pattern)):raise ValueError('test_adapter_in_release')
    if subprocess.check_output(['git','status','--porcelain'],cwd=repo,text=True).strip():raise ValueError('dirty_tooling_tree')
    tooling_sha=subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip()
    archive=subprocess.check_output(['git','archive',tooling_sha,'tools','third_party','integrations'],cwd=repo)
    destination.mkdir(parents=True)
    # Only our own committed Git archive; additionally refuse path escapes/links.
    with tarfile.open(fileobj=io.BytesIO(archive)) as tar:
        for member in tar.getmembers():
            path=destination/member.name
            if not path.resolve().is_relative_to(destination) or not (member.isfile() or member.isdir()):
                raise ValueError('unsupported_archive_entry:'+member.name)
            tar.extract(member,destination)
    shutil.copytree(candidate/'install',destination/'install')
    manifest=dict(schema='r1_neupan_release_v1',protocol_set='A+context1-source2-grant1-health1',
        build_source_sha=inv['source_sha'],tooling_source_sha=tooling_sha,build_testing=False,
        deployment_approved=False,gates=dict(full_no_motion_acceptance=False,
        physical_stop_acceptance=False,rollback_rehearsal=False),artifacts={})
    for role,(kind,relative) in preflight.ROLES.items():
        if kind=='library':
            paths=list((destination/'install').glob('*/lib/'+relative))
            if len(paths)!=1:raise ValueError('ambiguous_artifact:'+role)
            path=paths[0]
        elif kind=='python':path=destination/'install/r1_neupan_integration/lib/python3.10/site-packages'/relative
        else:path=destination/'install'/kind/relative
        manifest['artifacts'][role]=dict(path=str(path),sha256=digest(path))
    manifest['inventory']=[dict(path=str(p.relative_to(destination)),sha256=digest(p))
                           for p in sorted(destination.rglob('*')) if p.is_file()]
    manifest['inventory_root']=str(destination)
    target=destination/'releases/a-plus/manifest.json';target.parent.mkdir(parents=True)
    target.write_text(json.dumps(manifest,indent=2)+'\n')
    return dict(bundle=str(destination),manifest=str(target),files=len(manifest['inventory']),
                build_source_sha=inv['source_sha'],tooling_source_sha=tooling_sha,deployment_approved=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate',type=Path,required=True);parser.add_argument('--destination',type=Path,required=True)
    args=parser.parse_args()
    try:result=package(args.candidate.resolve(strict=True),args.destination.absolute(),Path(__file__).resolve().parents[2])
    except (OSError,ValueError,KeyError) as exc:print(json.dumps(dict(ok=False,error=str(exc))));return 2
    print(json.dumps(dict(ok=True,**result),indent=2));return 0
if __name__=='__main__':raise SystemExit(main())
