import numpy as np
import pytest
from neupan.dynamic_geometry import DynamicGeometry
from neupan import dynamic_geometry_native


@pytest.mark.skipif(not dynamic_geometry_native.available(),reason='native kernel not installed')
def test_native_matches_reference_across_components_heights_and_times():
    rng=np.random.default_rng(821)
    for _ in range(30):
        parts=[]
        for j in range(12):
            x,y=rng.uniform(-1,1,2);w,h=rng.uniform(.05,1,2)
            angle=rng.uniform(-np.pi,np.pi)
            rotation=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
            polygon=np.array([[0,0],[w,0],[w,h],[0,h]]) @ rotation.T + [x,y]
            parts.append(dict(polygon_xy=polygon,z_min=.2,z_max=1.,uncertainty_rate_mps=.1 if j%2 else None))
        model=DynamicGeometry(parts,source_age_sec=.4)
        points=rng.uniform(-2,2,(2,700));heights=rng.uniform(0,1.5,700)
        heights[::7]=np.nan
        scopes=rng.integers(0,3,700,dtype=np.uint8);times=rng.uniform(0,5,700)
        native=model.distance_and_normal(points,heights,scopes,times)
        model._native_data=None
        reference=model.distance_and_normal(points,heights,scopes,times)
        for a,b in zip(native,reference):
            np.testing.assert_allclose(a,b,atol=1e-9)
