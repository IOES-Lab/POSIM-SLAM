"""Frozen joint square-root information for delayed acoustic observations.

This preserves linearized correlations, including nullspaces. It is not a
standalone point covariance or a streaming graph manager. A consumer must keep
the anchor convention and must not add the absorbed measurements again.
"""
import copy
import numpy as np
from .ping_joint import eliminate_patch


def marginalize_window(jacobian,residual,*,states,landmarks,planes,track_ids,
                       retain_state_stamps_ns):
    stamps=[int(round(s['stamp']*1e9)) for s in states]
    keep=list(retain_state_stamps_ns)
    if (not keep or any(type(t) is not int or t not in stamps for t in keep)
            or len(set(keep))!=len(keep) or keep!=sorted(keep) or len(keep)>=len(stamps)):
        raise ValueError('retain_proper_state_subset_required')
    if len(landmarks)!=len(track_ids) or len(set(track_ids))!=len(track_ids):
        raise ValueError('distinct_landmark_ids_required')
    dimension=15*len(states)+3*(len(landmarks)+len(planes))
    j=np.asarray(jacobian,dtype=float);r=np.asarray(residual,dtype=float)
    if j.ndim!=2 or j.shape[1]!=dimension:raise ValueError('joint_window_dimension_mismatch')
    kept=[i for i,t in enumerate(stamps) if t in keep]
    removed=[i for i,t in enumerate(stamps) if t not in keep]
    columns=[k for i in kept for k in range(15*i,15*i+15)]
    columns+=list(range(15*len(states),dimension))
    retained=len(columns)
    columns+=[k for i in removed for k in range(15*i,15*i+15)]
    reduced=eliminate_patch(j[:,columns],r,retained_columns=retained)
    plane_bases=[]
    for plane in planes:
        normal=np.asarray(plane['normal']);u=np.cross(normal,np.eye(3)[np.argmin(abs(normal))])
        u/=np.linalg.norm(u);plane_bases.append(np.column_stack([u,np.cross(normal,u)]))
    # Keep the full rectangular square root. Inverting J'J or adding diagonal
    # regularization here would fill unobservable directions with false certainty.
    return dict(schema='posim-slam-sonar-frozen-prior-1',
        jacobian=reduced['jacobian'],residual=reduced['residual'],
        state_stamps_ns=keep,states=copy.deepcopy([states[i] for i in kept]),
        landmarks=copy.deepcopy(landmarks),planes=copy.deepcopy(planes),track_ids=list(track_ids),
        plane_rotation_bases=plane_bases,
        removed_state_stamps_ns=[stamps[i] for i in removed],
        eliminated_rank=reduced['nuisance_rank'],eliminated_nullity=reduced['nuisance_nullity'],
        tangent_order='states: position velocity right-attitude gyro-bias accel-bias; additive landmarks; plane tangent rotation and signed distance',
        scope='Frozen linearized joint information; reuse requires anchor-consistent deltas and measurement ownership',
        navigation_ready=False)
