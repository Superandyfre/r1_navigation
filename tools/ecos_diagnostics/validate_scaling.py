#!/usr/bin/env python3
"""Offline cone regression: uniform cost scaling, original-unit KKT checks.

Does not accept negative/inaccurate statuses. Does not command the robot.
Supported cones: nonnegative and second-order, as used by the NRMP sample.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import ecos
import numpy as np
from scipy.sparse import csc_matrix


def cone_violation(vector, dims):
    offset = dims['l']
    violations = [max(0., -float(np.min(vector[:offset])))] if offset else [0.]
    for count in dims.get('q', []):
        block=vector[offset:offset+count]
        violations.append(max(0., float(np.linalg.norm(block[1:])-block[0])))
        offset += count
    if dims.get('e', 0) or offset != len(vector):
        raise ValueError('unsupported_cone')
    return max(violations)


def run(path, repetitions=10):
    with np.load(path, allow_pickle=False) as data:
        meta=json.loads(str(data['metadata']))
        def matrix(name):
            if name+'_data' not in data:return None
            return csc_matrix((data[name+'_data'],data[name+'_indices'],data[name+'_indptr']),
                              shape=tuple(data[name+'_shape']))
        c,h=data['c'],data['h'];g,a=matrix('G'),matrix('A')
        b=data['b'] if 'b' in data else None
    rows=[]
    # Fixed BEFORE evaluating results. Negative statuses never pass.
    limits=dict(eq_abs=1e-5,cone_abs=1e-5,dual_relative=1e-5,gap_relative=1e-5)
    for scale in (1.,.1):
        for _ in range(repetitions):
            started=time.monotonic()
            solution=ecos.solve(c*scale,g,h,meta['dims'],a,b,**meta['solver_kwargs'])
            elapsed=time.monotonic()-started
            x=solution['x'];y=solution['y']/scale;z=solution['z']/scale
            slack=h-g@x
            dual=c+g.T@z+(a.T@y if a is not None else 0)
            primal_cost=float(c@x)
            dual_cost=-float(h@z+(b@y if b is not None else 0))
            checks=dict(eq_abs=float(np.max(abs(a@x-b))) if a is not None else 0.,
                        cone_abs=max(cone_violation(slack,meta['dims']),cone_violation(z,meta['dims'])),
                        dual_relative=float(np.max(abs(dual)))/(1+float(np.max(abs(c)))),
                        gap_relative=abs(primal_cost-dual_cost)/(1+abs(primal_cost)+abs(dual_cost)))
            valid=(solution['info']['exitFlag']==0 and np.isfinite(x).all() and
                   all(np.isfinite(v) and v<=limits[k] for k,v in checks.items()))
            rows.append(dict(scale=scale,valid=bool(valid),checks=checks,
                             cost=primal_cost,iterations=solution['info']['iter'],
                             flag=solution['info']['exitFlag'],seconds=elapsed))
    return dict(schema='r1_ecos_scaling_regression_v1',input=str(path),
                sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(),
                limits=limits,rows=rows,
                scaled_all_valid=all(r['valid'] for r in rows if r['scale']==.1),
                full_stateful_incident_context_available=False,
                physical_acceptance=False)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input',type=Path);parser.add_argument('--output',type=Path)
    args=parser.parse_args();report=run(args.input)
    text=json.dumps(report,indent=2,allow_nan=False)
    if args.output:args.output.write_text(text+'\n')
    print(text)
    raise SystemExit(0 if report['scaled_all_valid'] else 1)
