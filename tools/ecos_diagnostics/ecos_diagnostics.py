"""Opt-in ECOS telemetry. Solver arguments, result and exceptions are unchanged."""
import os
import sys


def _jsonable(value):
    import math
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(v) for v in value]
    if hasattr(value, 'item'):
        return _jsonable(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return repr(value)


def _context():
    # Failure-only lookup: recover the existing worker request without modifying it.
    frame = sys._getframe(1)
    while frame:
        request = frame.f_locals.get('request')
        if isinstance(request, dict) and 'request_id' in request:
            return {k: request.get(k) for k in (
                'request_id', 'cycle_id', 'scan_counter', 'path_version',
                'goal_version', 'worker_generation')}
        frame = frame.f_back
    return {}


def _emit(root, record):
    import json
    from pathlib import Path
    path = Path(root) / f'ecos-{os.getpid()}.jsonl'
    path.parent.mkdir(parents=True, exist_ok=True)
    # At most 4 x 4 MiB per process, plus the current record.
    if path.exists() and path.stat().st_size >= 4 * 1024 * 1024:
        for i in range(2, 0, -1):
            old = path.with_suffix(f'.jsonl.{i}')
            if old.exists():
                old.replace(path.with_suffix(f'.jsonl.{i+1}'))
        path.replace(path.with_suffix('.jsonl.1'))
    with path.open('a') as stream:
        stream.write(json.dumps(_jsonable(record), allow_nan=False) + '\n')


def _capture(root, call_id, args, kwargs):
    import json
    import numpy as np
    from pathlib import Path
    names = ['c', 'G', 'h', 'dims', 'A', 'b']
    values = dict(zip(names, args))
    values.update({k: kwargs[k] for k in names if k in kwargs})
    arrays = {}
    for name in ['c', 'h', 'b']:
        value = values.get(name)
        if value is not None:
            arrays[name] = np.asarray(value)
    for name in ['G', 'A']:
        value = values.get(name)
        if value is not None:
            value = value.tocsc(copy=False)
            arrays[name + '_data'] = value.data
            arrays[name + '_indices'] = value.indices
            arrays[name + '_indptr'] = value.indptr
            arrays[name + '_shape'] = np.asarray(value.shape)
    if sum(a.nbytes for a in arrays.values()) > 32 * 1024 * 1024:
        raise ValueError('ECOS input exceeds 32 MiB capture limit')
    metadata = {'schema': 'ecos_conic_input_v1', 'call_id': call_id,
                'dims': _jsonable(values.get('dims', {})),
                'solver_kwargs': _jsonable({k: v for k, v in kwargs.items() if k not in names}),
                'context': _context()}
    arrays['metadata'] = np.asarray(json.dumps(metadata))
    # Up to eight failure inputs per worker; no pickle objects.
    path = Path(root) / f'ecos-failure-{os.getpid()}-{call_id % 8}.npz'
    tmp = path.with_suffix('.tmp')
    with tmp.open('wb') as stream:
        np.savez_compressed(stream, **arrays)
    tmp.replace(path)
    return str(path)


def wrap_ecos(module):
    import functools
    import time
    root = os.environ.get('RPP_ECOS_DIAGNOSTICS_DIR', '')
    if not root or getattr(module.solve, '_rpp_diagnostics', False):
        return
    original = module.solve
    calls = 0
    try:
        _emit(root, {'event': 'enabled', 'pid': os.getpid(), 'wall_time': time.time(),
                     'ecos_module': module.__file__, 'ecos_version': getattr(module, '__version__', None),
                     'python': sys.executable, 'profile': os.environ.get('RPP_ECOS_PROFILE'),
                     'thread_environment': {k: os.environ.get(k) for k in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS')}})
    except Exception as error:
        print(f'[ECOS diagnostics] cannot initialize log: {error}', file=sys.stderr, flush=True)

    @functools.wraps(original)
    def solve(*args, **kwargs):
        nonlocal calls
        calls += 1
        call_id = calls
        started = time.perf_counter()
        result = None
        error_text = None
        try:
            result = original(*args, **kwargs)
            return result
        except Exception as error:
            error_text = f'{type(error).__name__}: {error}'
            raise
        finally:
            # Diagnostics must never replace the solver result or original exception.
            try:
                info = result.get('info', {}) if isinstance(result, dict) else {}
                flag = info.get('exitFlag')
                failed = error_text is not None or (flag is not None and flag not in (0, 10))
                record = {'event': 'solve', 'pid': os.getpid(), 'call_id': call_id,
                          'wall_time': time.time(), 'elapsed_sec': time.perf_counter()-started,
                          'info': info, 'exception': error_text, 'failed': failed}
                if failed:
                    record['context'] = _context()
                    try:
                        record['input_file'] = _capture(root, call_id, args, kwargs)
                    except Exception as capture_error:
                        record['capture_error'] = repr(capture_error)
                    print(f'[ECOS diagnostics] exitFlag={flag} info={_jsonable(info)} '
                          f'input={record.get("input_file", "not saved")}', file=sys.stderr, flush=True)
                _emit(root, record)
            except Exception as diagnostic_error:
                print(f'[ECOS diagnostics] logging failed: {diagnostic_error}', file=sys.stderr, flush=True)
    solve._rpp_diagnostics = True
    module.solve = solve


def install():
    """Wait for ECOS import; do not import NumPy before the Torch thread policy."""
    if not os.environ.get('RPP_ECOS_DIAGNOSTICS_DIR'):
        return
    if 'ecos' in sys.modules:
        wrap_ecos(sys.modules['ecos'])
        return
    import importlib.abc
    import importlib.machinery

    class Loader(importlib.abc.Loader):
        def __init__(self, original):
            self.original = original
        def create_module(self, spec):
            return self.original.create_module(spec)
        def exec_module(self, module):
            self.original.exec_module(module)
            wrap_ecos(module)

    class Finder(importlib.abc.MetaPathFinder):
        def find_spec(self, fullname, path=None, target=None):
            if fullname != 'ecos':
                return None
            spec = importlib.machinery.PathFinder.find_spec(fullname, path)
            if spec and spec.loader:
                spec.loader = Loader(spec.loader)
            return spec
    sys.meta_path.insert(0, Finder())
