"""Native NeuPAN rollout validation against a newly observed arm snapshot."""
import numpy as np
from neupan.dynamic_geometry import DynamicGeometry


def validate_dynamic_candidate(components, state, controls, step_time, points,
                               point_heights=None, point_scope=None, source_age_sec=0.):
    """Check unchanged obstacle samples against the actual commanded rollout."""
    geometry=DynamicGeometry(components,source_age_sec=source_age_sec)
    state=np.asarray(state,dtype=np.float64).reshape(3)
    controls=np.asarray(controls,dtype=np.float64)
    points=np.asarray(points,dtype=np.float64)
    if (controls.ndim!=2 or controls.shape[0]!=2 or controls.shape[1]<1 or
            points.ndim!=2 or points.shape[0]!=2 or not np.isfinite(state).all() or
            not np.isfinite(controls).all() or not np.isfinite(points).all() or
            not np.isfinite(step_time) or step_time<=0):
        raise ValueError("invalid dynamic rollout")
    states=[state.copy()]
    for v,w in controls.T:
        for _ in range(3):
            dt=step_time/3.
            x,y,yaw=states[-1]
            if abs(w)<1e-8:
                next_state=[x+v*np.cos(yaw)*dt,y+v*np.sin(yaw)*dt,yaw+w*dt]
            else:
                next_state=[x+v/w*(np.sin(yaw+w*dt)-np.sin(yaw)),
                            y+v/w*(np.cos(yaw)-np.cos(yaw+w*dt)),yaw+w*dt]
            states.append(next_state)
    states=np.asarray(states).T
    if points.shape[1]==0:return True
    possible=geometry.trajectory_collision_candidates(states,points,point_heights,point_scope,
        sample_times=np.arange(states.shape[1])*step_time/3.)
    if not np.any(possible):return True
    pts=points[:,possible]
    heights=None if point_heights is None else np.asarray(point_heights)[possible]
    scopes=None if point_scope is None else np.asarray(point_scope)[possible]
    dx=pts[0][None,:]-states[0,:,None]
    dy=pts[1][None,:]-states[1,:,None]
    c=np.cos(states[2])[:,None];s=np.sin(states[2])[:,None]
    local=np.stack((c*dx+s*dy,-s*dx+c*dy),axis=1)
    flat=local.transpose(1,0,2).reshape(2,-1)
    tiled_heights=None if heights is None else np.tile(heights,states.shape[1])
    tiled_scopes=None if scopes is None else np.tile(scopes,states.shape[1])
    distances,_,_=geometry.distance_and_normal(flat,tiled_heights,tiled_scopes,
        np.repeat(np.arange(states.shape[1])*step_time/3.,pts.shape[1]))
    # Height-disjoint points have no constraint; their infinite distance
    # cannot be mistaken for a numerical failure or an obstacle.
    finite=distances[np.isfinite(distances)]
    inactive=max(1.,float(np.max(np.abs(finite)))+1.) if finite.size else 1.
    distances[np.isposinf(distances)]=inactive
    from neupan.neupan import neupan
    return neupan._candidate_feasibility(distances.reshape(states.shape[1],-1))[0]
