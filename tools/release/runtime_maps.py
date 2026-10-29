#!/usr/bin/env python3
"""Same-host, read-only loaded artifact identity check. Never arms or launches."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROLE_PROCESS={'controller_server':'controller','controller_core':'controller',
              'controller_plugin':'controller','guard':'guard'}


def identity(pid):
    # comm may contain spaces/parentheses; fields after its final ')' start at3.
    fields=(Path('/proc')/str(int(pid))/'stat').read_text().rsplit(')',1)[1].split()
    if fields[0] in ('Z','X'):raise ValueError('process_not_live')
    return fields[19]


def check(manifest, bindings):
    errors=[];observed={}
    for role,process in ROLE_PROCESS.items():
        try:
            binding=bindings[process];pid=int(binding['pid'])
            before=identity(pid)
            if before!=str(binding['start_ticks']):raise ValueError('process_identity_changed')
            pin=manifest['artifacts'][role];expected=Path(pin['path']).resolve(strict=True)
            proc=Path('/proc')/str(pid)
            if role in ('controller_server','guard'):
                actual=(proc/'exe').resolve(strict=True)
                mapped_stat=(proc/'exe').stat()
            else:
                entries=[]
                for line in (proc/'maps').read_text().splitlines():
                    fields=line.split(maxsplit=5)
                    if len(fields)!=6:continue
                    name=fields[5]
                    if Path(name.removesuffix(' (deleted)')).name!=expected.name:continue
                    if name.endswith(' (deleted)'):raise ValueError('mapped_file_deleted')
                    entries.append(fields)
                if not entries:raise ValueError('required_library_not_loaded')
                if len({row[5] for row in entries})!=1:raise ValueError('multiple_loaded_artifacts')
                actual=Path(entries[0][5]).resolve(strict=True)
                mapped_stat=actual.stat()
                import os
                # Btrfs stat.st_dev may identify a subvolume, while maps uses
                # the mounted superblock device. Match maps to mountinfo.
                import re
                if os.readlink(proc/'ns/mnt')!=os.readlink('/proc/self/ns/mnt'):
                    raise ValueError('foreign_mount_namespace')
                mounts=[]
                for row in (proc/'mountinfo').read_text().splitlines():
                    parts=row.split()
                    mount=Path(re.sub(r'\\([0-7]{3})',lambda m:chr(int(m[1],8)),parts[4]))
                    if actual.is_relative_to(mount):
                        mounts.append((len(str(mount)),tuple(int(v) for v in parts[2].split(':'))))
                if not mounts:raise ValueError('mount_identity_missing')
                device=max(mounts)[1]
                if any(int(row[4])!=mapped_stat.st_ino or tuple(int(v,16) for v in row[3].split(':'))!=device for row in entries):
                    raise ValueError('mapped_file_replaced')
            if actual!=expected:errors.append(role+': loaded_path_mismatch')
            with actual.open('rb') as handle:
                import os
                first=os.fstat(handle.fileno());digest=hashlib.file_digest(handle,'sha256').hexdigest() if hasattr(hashlib,'file_digest') else hashlib.sha256(handle.read()).hexdigest()
                last=os.fstat(handle.fileno())
            if (first.st_dev,first.st_ino)!=(mapped_stat.st_dev,mapped_stat.st_ino):
                raise ValueError('file_identity_changed')
            if (first.st_size,first.st_mtime_ns,first.st_ctime_ns)!=(last.st_size,last.st_mtime_ns,last.st_ctime_ns):
                raise ValueError('file_changed_during_hash')
            if digest!=pin['sha256']:errors.append(role+': loaded_hash_mismatch')
            if identity(pid)!=before:raise ValueError('process_identity_changed')
            observed[role]=dict(pid=pid,start_ticks=before,path=str(actual),sha256=digest,inode=first.st_ino)
        except (OSError,ValueError,KeyError,TypeError,IndexError) as exc:
            errors.append(role+': '+str(exc))
    return dict(ok=not errors,errors=errors,observed=observed,
                scope='controller executable/core/plugin and guard executable only',
                motion_authorized=False)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--bindings',type=Path,required=True)
    args=p.parse_args()
    try:r=check(json.loads(args.manifest.read_text()),json.loads(args.bindings.read_text()))
    except (ValueError,OSError,TypeError) as exc:r=dict(ok=False,errors=[str(exc)])
    print(json.dumps(r,indent=2));return 0 if r['ok'] else 2
if __name__=='__main__':sys.exit(main())
