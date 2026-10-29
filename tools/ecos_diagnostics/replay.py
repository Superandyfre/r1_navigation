#!/usr/bin/env python3
"""Replay a captured cone problem offline, optionally printing ECOS iterations."""
import argparse
import json
import numpy as np
from scipy.sparse import csc_matrix
import ecos

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('input')
parser.add_argument('--verbose', action='store_true')
args = parser.parse_args()
with np.load(args.input, allow_pickle=False) as data:
    meta = json.loads(str(data['metadata']))
    def matrix(name):
        if name+'_data' not in data:
            return None
        return csc_matrix((data[name+'_data'], data[name+'_indices'], data[name+'_indptr']),
                          shape=tuple(data[name+'_shape']))
    options = meta['solver_kwargs']
    options['verbose'] = args.verbose
    result = ecos.solve(data['c'], matrix('G'), data['h'], meta['dims'],
                        matrix('A'), data['b'] if 'b' in data else None, **options)
    print(json.dumps(result['info'], indent=2, default=lambda x:x.item()))
