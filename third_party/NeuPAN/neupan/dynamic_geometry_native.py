"""Optional native implementation of the exact dynamic polygon distance."""
import ctypes as ct
import numpy as np


def _load():
    try:
        lib=ct.CDLL('libr1_neupan_dynamic_geometry.so')
        fn=lib.r1_dynamic_distance
        dp=ct.POINTER(ct.c_double)
        fn.argtypes=[ct.c_int,ct.c_int,dp,dp,dp,ct.POINTER(ct.c_uint8),dp,ct.c_double,
                     ct.POINTER(ct.c_int),dp,dp,dp,dp,dp,dp,dp,dp,dp,dp]
        fn.restype=ct.c_int
        return fn
    except (OSError,AttributeError):
        return None


_kernel=_load()


def available():
    return _kernel is not None


def prepare(geometry):
    a=lambda x:np.ascontiguousarray(x,dtype=np.float64)
    vertices=np.vstack(geometry.components)
    return (np.ascontiguousarray(np.cumsum([0]+[len(p) for p in geometry.components]),dtype=np.int32),
            a(vertices[:,0]),a(vertices[:,1]),a(np.asarray(geometry.height_ranges)[:,0]),
            a(np.asarray(geometry.height_ranges)[:,1]),
            a([-1. if r is None else r for r in geometry.motion_rates]),
            a([[c[0][0],c[1][0],c[0][1],c[1][1]] for c in geometry._polygon_cache]))


def distance(geometry,points,heights,scope,times):
    a=lambda x:np.ascontiguousarray(x,dtype=np.float64)
    dp=ct.POINTER(ct.c_double)
    ptr=lambda v:v.ctypes.data_as(dp)
    n=points.shape[1]
    x,y=a(points[0]),a(points[1])
    h,t=a(heights),a(times)
    scopes=np.ascontiguousarray(scope,dtype=np.uint8)
    starts,vx,vy,zlo,zhi,rates,bounds=geometry._native_data
    out=np.empty((4,n),dtype=np.float64)
    code=_kernel(n,len(geometry.components),ptr(x),ptr(y),ptr(h),
                 scopes.ctypes.data_as(ct.POINTER(ct.c_uint8)),ptr(t),geometry.source_age_sec,
                 starts.ctypes.data_as(ct.POINTER(ct.c_int)),ptr(vx),ptr(vy),ptr(zlo),ptr(zhi),
                 ptr(rates),ptr(bounds),*[ptr(row) for row in out])
    if code:
        raise RuntimeError('dynamic geometry native kernel failed')
    return out[0],out[1:3],out[3]
