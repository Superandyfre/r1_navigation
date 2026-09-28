#!/usr/bin/env python3
"""Compare the two frozen Python environments without mutating either."""
from __future__ import annotations
import argparse, json, subprocess
from pathlib import Path

PACKAGES=['torch','numpy','scipy','cvxpy','cvxpylayers','diffcp','ecos','PyYAML','setuptools','neupan']
CODE="""import importlib.metadata as m, json, sys, torch\nout={'python':sys.version,'torch_cuda':torch.version.cuda}\nfor p in %r:\n try: out[p]=m.version(p)\n except Exception: out[p]=None\nprint(json.dumps(out))\n""" % PACKAGES
def inspect(venv:str)->dict:
    return json.loads(subprocess.check_output([str(Path(venv)/'bin/python'),'-c',CODE],text=True))
def main()->int:
    p=argparse.ArgumentParser();p.add_argument('--n0',required=True);p.add_argument('--current',required=True);p.add_argument('--output',required=True);a=p.parse_args()
    n0=inspect(a.n0); current=inspect(a.current)
    diff={k:{'n0':n0.get(k),'current':current.get(k),'same':n0.get(k)==current.get(k)} for k in sorted(set(n0)|set(current))}
    Path(a.output).write_text(json.dumps({'n0':n0,'current':current,'diff':diff},indent=2,sort_keys=True),encoding='utf-8')
    print(Path(a.output).read_text())
    return 0
if __name__=='__main__':raise SystemExit(main())
