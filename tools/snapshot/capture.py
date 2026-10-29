#!/usr/bin/env python3
"""Read-only host collection. Never starts navigation, arms, or publishes a goal."""
import argparse, datetime, hashlib, json, os, platform, re, shlex, shutil, subprocess, tarfile
from pathlib import Path
from urllib.parse import urlsplit, urlunsplit

ROOT = Path(__file__).resolve().parents[2]
SECRET_NAME = re.compile(r'(^\.env($|\.)|^credentials(\.[^.]+)?$|^id_(rsa|ed25519)(\.pub)?$|\.(pem|key)$|^(password|secret|token)\.(json|yaml|yml|txt|ini|cfg)$)', re.I)
SECRET_TEXT = re.compile(rb'(?im)^\s*(?:export\s+)?[\w-]*(?:password|passwd|access_token|api_key|secret_key)\s*[:=]\s*[\x22\x27]?[^\s\x22\x27]{3,}')
SKIP = {'.git', '__pycache__', 'node_modules', '.pytest_cache', '.venv'}

def write(p, data):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')

def sha(p):
    h = hashlib.sha256()
    with p.open('rb') as f:
        while chunk := f.read(1024 * 1024): h.update(chunk)
    return h.hexdigest()

def run(args, cwd=None, timeout=40):
    try:
        p = subprocess.run(args, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout)
        return dict(command=args, returncode=p.returncode, stdout=p.stdout.decode(errors='replace'), stderr=p.stderr.decode(errors='replace'))
    except subprocess.TimeoutExpired: return dict(command=args, returncode=None, error='timeout')
    except OSError as e: return dict(command=args, returncode=None, error=str(e))

def clean_url(value):
    if '://' not in value: return re.sub(r'^[^/@]+@', '', value)
    x = urlsplit(value)
    return urlunsplit((x.scheme, x.hostname or '', x.path, '', ''))

def repositories(base):
    result = []
    for here, dirs, files in os.walk(base):
        dirs[:] = [d for d in dirs if d not in SKIP and not d.startswith(('build', 'install', 'log'))]
        if (Path(here) / '.git').exists(): result.append(Path(here))
    return result

def safe_file(p):
    if SECRET_NAME.search(p.name): return False
    if p.suffix.lower() in {'.json','.yaml','.yml','.ini','.cfg','.sh','.bash','.config'} and p.is_file() and not p.is_symlink() and p.stat().st_size < 2 * 1024 * 1024:
        if SECRET_TEXT.search(p.read_bytes()): return False
    return True

def archive(source, target, excluded, inventory):
    source = Path(source)
    before = []
    with tarfile.open(target, 'w:gz', compresslevel=1, dereference=False) as tar:
        def filt(info):
            parts = Path(info.name).parts
            p = source.parent / info.name
            if any(x in SKIP for x in parts) or any(SECRET_NAME.search(x) for x in parts):
                excluded.append(dict(path=str(p), reason='excluded cache/git/credential filename')); return None
            if info.isfile():
                if not safe_file(p):
                    excluded.append(dict(path=str(p), reason='possible credential assignment')); return None
                before.append(dict(path=str(p), sha256=sha(p), bytes=p.stat().st_size))
            if info.issym():
                inventory.append(dict(path=str(p), link=info.linkname, resolved=str(p.resolve()), target_exists=p.exists()))
            return info
        tar.add(source, arcname=source.name, filter=filt)
    changes = [x['path'] for x in before if not Path(x['path']).exists() or sha(Path(x['path'])) != x['sha256']]
    return dict(path=target.name, source=str(source), sha256=sha(target), bytes=target.stat().st_size,
                files=len(before), changed_during_capture=changes)

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--profile', type=Path, default=ROOT/'deployment/profiles/r1-hw1.json')
    ap.add_argument('--id', required=True)
    ap.add_argument('--runtime', action='store_true', help='Read ROS graph/parameters/TF with explicit overlays; no motion')
    a = ap.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]+', a.id): ap.error('unsafe snapshot id')
    profile = json.loads(a.profile.read_text()); dest = Path(profile['snapshot_store'])/a.id
    dest.mkdir(parents=True, exist_ok=False); os.chmod(dest, 0o700)
    for d in ['sources','patches','payload','environment','runtime']: (dest/d).mkdir()
    write(dest/'profile.json', profile)
    excluded = []; links = []; gaps = []; repos = []
    mainrepo = Path(profile['main_repository']); underlay = Path(profile['underlay_sources'])
    paths = [mainrepo, Path(profile['additional_repository'])] + repositories(underlay)
    for i, path in enumerate(dict.fromkeys(paths)):
        prefix = f'{i:03d}-{path.name}'
        def git(*args): return run(['git','-C',str(path),*args])
        head = git('rev-parse','HEAD'); status = git('status','--porcelain=v1','--untracked-files=all')
        remote = git('remote','get-url','origin')
        row = dict(path=str(path), commit=head.get('stdout','').strip(), remote=clean_url(remote.get('stdout','').strip()),
                   status=status, submodules=git('submodule','status','--recursive'), bundle=f'sources/{prefix}.bundle')
        result = git('bundle','create',str(dest/row['bundle']),'--all','HEAD')
        row['bundle_result'] = result
        if result['returncode'] != 0: gaps.append('git bundle failed: '+str(path))
        else: row['bundle_sha256'] = sha(dest/row['bundle'])
        for name, args in [('worktree',['diff','--binary','HEAD']),('index',['diff','--cached','--binary'])]:
            patch = subprocess.run(['git','-C',str(path),*args],stdout=subprocess.PIPE,check=True).stdout
            if SECRET_TEXT.search(patch):
                excluded.append(dict(path=str(path),reason='possible credentials in patch')); gaps.append('redacted patch: '+str(path))
            else: (dest/'patches'/f'{prefix}-{name}.patch').write_bytes(patch)
        row['untracked'] = git('ls-files','--others','--exclude-standard')
        repos.append(row)
    write(dest/'sources/repositories.json', repos)
    sources = {'main-source':mainrepo, 'underlay-source':underlay, 'aplus-source':Path(profile['additional_repository'])}
    # Explicit source roots avoid copying mainrepo build/install/log and research output.
    payloads = []
    for label, path in sources.items():
        if label in {'main-source','aplus-source'}:
            for name in ['packages','third_party','tools','config','deployment','docs','snapshots','README.md','.gitignore']:
                p=path/name
                if p.exists(): payloads.append((label+'-'+name.replace('.','_'),p))
        else: payloads.append((label,path))
    config = json.loads(Path(profile['ui_config']).read_text()); buttons=[]
    def visit(d):
        if isinstance(d,dict):
            if d.get('name') == profile['ui_button']: buttons.append(d)
            for v in d.values(): visit(v)
    visit(config)
    if len(buttons)!=1: raise RuntimeError('UI7 not uniquely registered')
    button=buttons[0]; entry=Path(os.path.expanduser(button['bash_url'])).resolve(); bundle=entry.parents[2]
    write(dest/'runtime/ui7.json',dict(button=button,entry=str(entry),bundle=str(bundle)))
    source_manifest=bundle/'releases/a-plus/manifest.json'
    if source_manifest.exists():
        bm=json.loads(source_manifest.read_text())
        for row in bm['inventory']:
            p=bundle/row['path']
            if not p.resolve().is_relative_to(bundle) or sha(p)!=row['sha256']: raise RuntimeError('active bundle changed: '+str(p))
        write(dest/'runtime/navigation-release.json',bm)
    else: gaps.append('UI7 has no full release inventory')
    payloads += [('active-navigation',bundle),('underlay-install',Path(profile['underlay_install'])),
                 ('ros-humble',Path(profile['ros_prefix'])),('python-environment',Path(profile['python_environment'])),
                 ('ui-configuration',Path(profile['ui_config']))]
    payloads += [(k,Path(v)) for k,v in profile['assets'].items()]
    artifacts=[]
    for label,path in payloads:
        if not path.exists(): gaps.append('missing payload: '+str(path)); continue
        print('archiving '+label,flush=True)
        row=archive(path,dest/'payload'/f'{label}.tar.gz',excluded,links);row['path']='payload/'+row['path'];artifacts.append(row)
    for interp,label in [(profile['python'],'neupan'),('/usr/bin/python3','system')]:
        result=run([interp,'-c','import importlib.metadata,json,sys; print(json.dumps({"executable":sys.executable,"version":sys.version,"packages":sorted([[d.metadata["Name"],d.version] for d in importlib.metadata.distributions()])}))'])
        write(dest/'environment'/f'python-{label}.json',result)
        if result['returncode'] != 0: gaps.append('Python inventory failed: '+label)
    locks={'system-packages':run(['dpkg-query','-W','-f=${binary:Package}\t${Version}\t${Architecture}\n']),
           'compiler':run(['/usr/bin/c++','--version']), 'cuda':run(['/usr/local/cuda-12.8/bin/nvcc','--version']),
           'gpu':run(['nvidia-smi','--query-gpu=name,driver_version','--format=csv,noheader'])}
    for label,result in locks.items(): write(dest/'environment'/f'{label}.json',result)
    write(dest/'environment/host.json',dict(uname=list(platform.uname()),os_release=Path('/etc/os-release').read_text(),
        environment={k:os.environ[k] for k in ['ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION','AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','LD_LIBRARY_PATH','PYTHONPATH'] if k in os.environ}))
    caches={}
    for base in [mainrepo/'build',Path('/home/rpp/rpp_ws/build')]:
        for p in base.glob('*/CMakeCache.txt'):
            selected=[x for x in p.read_text(errors='replace').splitlines() if re.match(r'^(BUILD_TESTING|CMAKE_BUILD_TYPE|CMAKE_(C|CXX)_COMPILER|Python3_EXECUTABLE|CMAKE_HOME_DIRECTORY|.*_DIR):',x)]
            caches[str(p)]=selected
    write(dest/'environment/build-settings.json',caches)
    envcmd='; '.join('. '+shlex.quote(str(p)) for p in [Path(profile['ros_prefix'])/'setup.bash',Path(profile['underlay_install'])/'local_setup.bash',bundle/'install/local_setup.bash'])
    resolutions={pkg:run(['/bin/bash','-c',envcmd+'; ros2 pkg prefix '+shlex.quote(pkg)]) for pkg in profile['navigation_packages']}
    write(dest/'runtime/package-prefixes.json',resolutions)
    module_probe = "import importlib.util,json,sys; names=['r1_neupan_integration','r1_neupan_integration.n1_16_node','neupan','torch','cvxpy','ecos']; print(json.dumps({'executable':sys.executable,'modules':{n:(str(x.origin) if (x:=importlib.util.find_spec(n)) else None) for n in names}}))"
    probe = envcmd+'; export PYTHONPATH='+shlex.quote(str(bundle/'third_party/NeuPAN'))+':"${PYTHONPATH:-}"; '+shlex.quote(profile['python'])+' -c '+shlex.quote(module_probe)
    write(dest/'runtime/python-module-origins.json',run(['/bin/bash','-c',probe]))
    if a.runtime:
        graph=run(['/bin/bash','-c',envcmd+'; ros2 node list --no-daemon --spin-time 2'],timeout=15)
        write(dest/'runtime/nodes.json',graph)
        write(dest/'runtime/topics.json',run(['/bin/bash','-c',envcmd+'; ros2 topic list -t --no-daemon --spin-time 2'],timeout=15))
        for i,node in enumerate(graph.get('stdout','').splitlines()):
            if node.startswith('/'):
                write(dest/'runtime'/f'parameters-{i}.json',dict(node=node,result=run(['/bin/bash','-c',envcmd+'; ros2 param dump '+shlex.quote(node)],timeout=8)))
        for topic in ['/tf','/tf_static']:
            write(dest/'runtime'/(topic[1:]+'.json'),run(['/bin/bash','-c',envcmd+'; ros2 topic echo '+topic+' --once'],timeout=5))
    write(dest/'runtime/processes.json',capture_processes())
    write(dest/'environment/excluded.json',excluded);write(dest/'environment/symlinks.json',links)
    gaps += ['No normal-navigation rosbag collected; motion not authorized for this snapshot.',
             'No independent restore/build/navigation acceptance yet.',
             'System package lock has no complete offline .deb/wheel cache or OS image; bare-host offline recovery unproven.',
             'Absolute paths, external symlinks and native/CUDA system dependencies need restore-host verification.']
    if excluded: gaps.append('Some credential/cache files excluded; inspect environment/excluded.json.')
    if any(x['changed_during_capture'] for x in artifacts): gaps.append('Files changed during capture; not an atomic snapshot.')
    failed_parameters = [p.name for p in (dest/'runtime').glob('parameters-*.json') if json.loads(p.read_text())['result']['returncode'] != 0]
    if failed_parameters: gaps.append('Runtime parameter queries incomplete: '+', '.join(failed_parameters))
    unresolved = [x for x in links if not x['target_exists']]
    if unresolved: gaps.append(str(len(unresolved))+' source symlinks have missing targets; recorded, not silently restored.')
    manifest=dict(schema='r1_navigation_snapshot_v1',id=a.id,created_at=datetime.datetime.now().astimezone().isoformat(),
        main_source_commit=repos[0]['commit'],ui7_release_build_commit=json.loads(source_manifest.read_text()).get('build_source_sha') if source_manifest.exists() else None,
        status=dict(captured=True,built=False,restored=False,navigation_validated=False,offline_complete=False),
        profile='profile.json',repositories='sources/repositories.json',artifacts=artifacts,limitations=gaps)
    write(dest/'manifest.yaml',manifest) # JSON is a YAML 1.2 subset; no YAML dependency.
    (dest/'validation.md').write_text('# Baseline capture\n\nCaptured, not a validated release.\n\n'+'\n'.join('- '+x for x in gaps)+'\n')
    files=[p for p in sorted(dest.rglob('*')) if p.is_file()]
    (dest/'checksums.sha256').write_text(''.join(sha(p)+'  '+str(p.relative_to(dest))+'\n' for p in files))
    print(json.dumps(dict(snapshot=str(dest),artifacts=len(artifacts),repositories=len(repos),status=manifest['status']),indent=2))

def capture_processes():
    result=[]
    for p in Path('/proc').glob('[0-9]*'):
        try:
            args=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
            if any(x in args for x in ['controller_server','collision_guard_node','n1_16_node','location_node','robot_driver.launch.py','starterbutton_ui.py']):
                if 'capture.py' in args: continue
                result.append(dict(pid=int(p.name),executable=str((p/'exe').resolve()),maps=(p/'maps').read_text()))
        except (PermissionError,FileNotFoundError,ProcessLookupError): pass
    return result

if __name__=='__main__': main()
