#!/usr/bin/env python3
"""Verify snapshot hashes, compare live files, or stage an artifact. Never deploy."""
import argparse, hashlib, json, os, tarfile
from pathlib import Path

def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        while chunk:=f.read(1024*1024):h.update(chunk)
    return h.hexdigest()

def verify(root):
    root=root.resolve(strict=True);checks=root/'checksums.sha256';count=0
    for line in checks.read_text().splitlines():
        digest,name=line.split('  ',1);p=root/name
        if not p.resolve().is_relative_to(root) or not p.is_file() or sha(p)!=digest:
            raise ValueError('missing/changed snapshot file: '+name)
        count+=1
    manifest=json.loads((root/'manifest.yaml').read_text())
    for row in manifest['artifacts']:
        p=root/row['path']
        if sha(p)!=row['sha256']:raise ValueError('artifact mismatch: '+row['path'])
    return manifest,count

def compare(root,manifest):
    results=[]
    for row in manifest['artifacts']:
        source=Path(row['source']);missing=[];changed=[];links=[]
        with tarfile.open(root/row['path'],'r:gz') as tar:
            for member in tar:
                # Stored root is source.name. Map to the exact original path.
                parts=Path(member.name).parts
                if not parts or parts[0]!=source.name:raise ValueError('unexpected artifact root')
                p=source.parent/member.name
                if member.isfile():
                    f=tar.extractfile(member);h=hashlib.sha256()
                    while chunk:=f.read(1024*1024):h.update(chunk)
                    if not p.is_file():missing.append(str(p))
                    elif sha(p)!=h.hexdigest():changed.append(str(p))
                elif member.issym():
                    if not p.is_symlink() or os.readlink(p)!=member.linkname:links.append(str(p))
        results.append(dict(source=str(source),missing=missing,changed=changed,link_mismatches=links))
    return results

def stage(root,manifest,artifact,destination):
    rows=[x for x in manifest['artifacts'] if x['path']==artifact]
    if len(rows)!=1:raise ValueError('artifact must match manifest entry exactly')
    destination=destination.absolute()
    if destination.exists():raise ValueError('refuse existing stage directory')
    # Prevalidate whole archive before creating files; links cannot redirect
    # later extraction. Absolute environment links are recorded, not created.
    deferred=[]
    with tarfile.open(root/artifact,'r:gz') as tar:
        members=tar.getmembers();names=set()
        for m in members:
            p=Path(m.name)
            if p.is_absolute() or '..' in p.parts or m.name in names:raise ValueError('unsafe/duplicate archive path')
            names.add(m.name)
            if not(m.isfile() or m.isdir() or m.issym() or m.islnk()):raise ValueError('unsupported archive member')
        linknames={m.name for m in members if m.issym() or m.islnk()}
        for m in members:
            if any(str(parent) in linknames for parent in Path(m.name).parents):raise ValueError('archive writes through link')
        destination.mkdir(parents=True)
        for m in members:
            p=destination/m.name
            if m.issym() or m.islnk():
                deferred.append(dict(path=m.name,target=m.linkname,kind='symlink' if m.issym() else 'hardlink'));continue
            if m.isdir():p.mkdir(parents=True,exist_ok=True)
            elif m.isfile():
                p.parent.mkdir(parents=True,exist_ok=True)
                with tar.extractfile(m) as src,p.open('xb') as dst:
                    while chunk:=src.read(1024*1024):dst.write(chunk)
                p.chmod(m.mode & 0o777)
    (destination/'DEFERRED_LINKS.json').write_text(json.dumps(deferred,indent=2)+'\n')
    return dict(staged=str(destination),links_deferred=len(deferred),deployed=False)

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('snapshot',type=Path)
    ap.add_argument('--compare-live',action='store_true');ap.add_argument('--artifact');ap.add_argument('--destination',type=Path)
    a=ap.parse_args();root=a.snapshot.resolve(strict=True)
    try:
        m,n=verify(root);result=dict(snapshot=m['id'],verified_files=n,status=m['status'])
        if a.compare_live:
            result['live_comparison']=compare(root,m)
            result['live_matches']=all(not any(x[k] for k in ['missing','changed','link_mismatches']) for x in result['live_comparison'])
        if a.artifact:
            if not a.destination:ap.error('--artifact requires --destination')
            result['stage']=stage(root,m,a.artifact,a.destination)
        print(json.dumps(result,ensure_ascii=False,indent=2))
        return 0 if result.get('live_matches',True) else 2
    except (OSError,ValueError,KeyError,tarfile.TarError) as e:
        print(json.dumps(dict(ok=False,error=str(e))));return 2

if __name__=='__main__':raise SystemExit(main())
