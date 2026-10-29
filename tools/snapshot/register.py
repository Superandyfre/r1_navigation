#!/usr/bin/env python3
"""Register a verified capture in Git metadata; never marks navigation validated."""
import argparse,hashlib,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools/restore'))
from restore import verify

p=argparse.ArgumentParser(description=__doc__);p.add_argument('snapshot',type=Path);a=p.parse_args()
m,count=verify(a.snapshot)
entry=ROOT/'snapshots'/m['id'];entry.mkdir(parents=True,exist_ok=False)
source=json.loads((a.snapshot/'sources/repositories.json').read_text())
repos={}
for i,r in enumerate(source):
 repos[f'{i:03d}-{Path(r["path"]).name}']=dict(type='git',url=r['remote'] or ('snapshot-bundle:'+r['bundle']),version=r['commit'])
(ROOT/'deployment/dependencies.repos').write_text(json.dumps(dict(repositories=repos),indent=2)+'\n')
locks={}
for name in ['neupan','system']:
 data=json.loads((a.snapshot/f'environment/python-{name}.json').read_text())
 locks[name]=json.loads(data['stdout']) if data['returncode']==0 else data
(ROOT/'deployment/python.lock').write_text(json.dumps(dict(schema='r1_python_inventory_v1',note='Exact installed versions; full environment archived. Not a pip --require-hashes lock.',environments=locks),indent=2)+'\n')
packages=json.loads((a.snapshot/'environment/system-packages.json').read_text())
(ROOT/'deployment/system-packages.lock').write_text(packages.get('stdout',''))
(ROOT/'deployment/build-settings.yaml').write_text((a.snapshot/'environment/build-settings.json').read_text())
record=dict(schema='r1_navigation_snapshot_pointer_v1',id=m['id'],storage=str(a.snapshot.resolve()),manifest_sha256=hashlib.sha256((a.snapshot/'manifest.yaml').read_bytes()).hexdigest(),checksums_sha256=hashlib.sha256((a.snapshot/'checksums.sha256').read_bytes()).hexdigest(),status=m['status'],main_source_commit=m['main_source_commit'],ui7_release_build_commit=m['ui7_release_build_commit'],verified_files=count,limitations=m['limitations'])
(entry/'manifest.yaml').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
(entry/'validation.md').write_text((a.snapshot/'validation.md').read_text())
print(json.dumps(record,ensure_ascii=False,indent=2))
