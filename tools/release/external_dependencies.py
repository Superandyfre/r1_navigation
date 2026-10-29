#!/usr/bin/env python3
"""Snapshot dependencies observed in isolated test evidence; never approves release."""
import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

SCHEMA='r1_neupan_external_dependencies_v1'


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def collect(reports,bundle):
    paths={};missing=[];evidence=[];transient=[]
    def add(value,source,mapping_permissions=None):
        if not isinstance(value,str) or not value.startswith('/'):return
        path=Path(value)
        # Fast DDS shared-memory data segments are not release code. Preserve
        # every observation, including deleted segments; never hide executable
        # mappings or a similarly named path from executable/summary records.
        if (mapping_permissions is not None and 'x' not in mapping_permissions and
                path.parent==Path('/dev/shm') and path.name.startswith('fastrtps_')):
            transient.append(dict(path=value,evidence=str(source),permissions=mapping_permissions,
                                  classification='fastdds_transient_shared_memory_data'))
            return
        # These are observations, never commands or import expressions.
        if path.is_relative_to(bundle):return
        if not path.is_file():
            missing.append(dict(path=value,evidence=str(source)));return
        paths.setdefault(str(path),set()).add(str(source))
    def walk(value,source):
        if isinstance(value,dict):
            for key,item in value.items():
                if key in ('filepath','python_executable'):add(item,source)
                walk(item,source)
        elif isinstance(value,list):
            for child in value:walk(child,source)
    for report in reports:
        for maps in sorted(report.glob('*maps*.txt')):
            evidence.append(dict(path=str(maps),sha256=digest(maps)))
            for line in maps.read_text().splitlines():
                fields=line.split(maxsplit=5)
                if len(fields)==6 and fields[5].startswith('/'):
                    add(fields[5],maps,fields[1])
        for summary in report.glob('**/summary.json'):
            evidence.append(dict(path=str(summary),sha256=digest(summary)))
            walk(json.loads(summary.read_text()),summary)
        binary=report/'binaries.json'
        if binary.is_file():
            evidence.append(dict(path=str(binary),sha256=digest(binary)))
            for row in json.loads(binary.read_text()):
                if 'path' in row:add(row['path'],binary)
    rows=[]
    for value,sources in sorted(paths.items()):
        path=Path(value);target=path.resolve(strict=True)
        before=target.stat();sha=digest(target);after=target.stat()
        if (before.st_ino,before.st_size,before.st_mtime_ns)!=(after.st_ino,after.st_size,after.st_mtime_ns):
            raise ValueError('dependency_changed_during_snapshot:'+value)
        if path.resolve(strict=True)!=target:raise ValueError('dependency_target_changed_during_snapshot:'+value)
        rows.append(dict(path=value,resolved=str(target),sha256=sha,size=after.st_size,evidence=sorted(sources)))
    return dict(schema=SCHEMA,bundle=str(bundle),review_complete=False,files=rows,
                missing=missing,evidence=evidence,transient_data_mappings=transient,
                captured_at=datetime.now(timezone.utc).isoformat(),
                hash_timing='Files hashed after saved test evidence was captured; not proof of their bytes during that test',
                scope='Observed maps, executable records and solver summary runtime paths only; not full dependency closure',
                production_started=False,motion_authorized=False)


def check(snapshot):
    errors=[]
    if snapshot.get('schema')!=SCHEMA:errors.append('invalid_dependency_schema')
    if not snapshot.get('files'):errors.append('empty_dependency_inventory')
    for row in snapshot.get('files',[]):
        try:
            path=Path(row['path'])
            if not path.is_absolute():raise ValueError('dependency_path_not_absolute')
            if str(path.resolve(strict=True))!=row['resolved']:raise ValueError('dependency_target_changed')
            if digest(path)!=row['sha256']:raise ValueError('dependency_hash_changed')
        except (OSError,KeyError,ValueError,TypeError) as exc:errors.append(str(row.get('path'))+':'+str(exc))
    if snapshot.get('missing'):errors.append('unresolved_observed_dependencies')
    return dict(ok=not errors,errors=errors,review_complete=snapshot.get('review_complete') is True,
                scope='file consistency only',motion_authorized=False)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bundle',type=Path)
    p.add_argument('--reports',type=Path,nargs='+')
    p.add_argument('--output',type=Path)
    p.add_argument('--check',type=Path)
    args=p.parse_args()
    if args.check:
        result=check(json.loads(args.check.read_text()));print(json.dumps(result,indent=2));return 0 if result['ok'] else 2
    if not args.bundle or not args.reports or not args.output:p.error('snapshot requires bundle, reports, output')
    snapshot=collect([r.resolve(strict=True) for r in args.reports],args.bundle.resolve(strict=True))
    with args.output.open('x') as stream:json.dump(snapshot,stream,indent=2);stream.write('\n')
    print(json.dumps(dict(files=len(snapshot['files']),unresolved=len(snapshot['missing']),review_complete=False)))
    return 0


if __name__=='__main__':raise SystemExit(main())
