"""Frozen joint square-root information for delayed acoustic observations.

This preserves linearized correlations, including nullspaces. It is not a
standalone point covariance or a streaming graph manager. A consumer must keep
the anchor convention and must not add the absorbed measurements again.
"""
import copy
import numpy as np
from .ping_joint import eliminate_patch
from .inertial_alignment import log_so3


def compact_square_root(jacobian,residual):
    """Bound rows by columns + 1 while preserving the entire quadratic cost.

    The final zero-Jacobian row carries cost orthogonal to the column space.
    Dropping that constant would make residual costs incomparable across windows.
    QR does not invert the information matrix or fill rank-deficient directions.
    """
    j=np.asarray(jacobian,dtype=float);r=np.asarray(residual,dtype=float)
    if (j.ndim!=2 or r.shape!=(j.shape[0],) or not np.isfinite(j).all()
            or not np.isfinite(r).all()):raise ValueError('invalid_square_root')
    if j.shape[0]<=j.shape[1]:return j.copy(),r.copy()
    q,root=np.linalg.qr(j,mode='reduced');projected=q.T@r
    remainder=float(np.linalg.norm(r-q@projected))
    return np.vstack([root,np.zeros((1,j.shape[1]))]),np.r_[projected,remainder]


def prior_residual(prior,states,landmarks,track_ids,planes=()):
    """Evaluate frozen sonar/IMU history at its original tangent anchors."""
    state_map={int(round(s['stamp']*1e9)):s for s in states}
    point_map=dict(zip(track_ids,landmarks));delta=[]
    for stamp,anchor in zip(prior['state_stamps_ns'],prior['states']):
        if stamp not in state_map:raise ValueError('prior_state_missing')
        s=state_map[stamp]
        delta.extend(np.r_[s['position']-anchor['position'],s['velocity']-anchor['velocity'],
            log_so3(anchor['attitude'].T@s['attitude']),s['gyro_bias']-anchor['gyro_bias'],
            s['accel_bias']-anchor['accel_bias']])
    for ident,anchor in zip(prior['track_ids'],prior['landmarks']):
        if ident not in point_map:raise ValueError('prior_landmark_missing')
        delta.extend(point_map[ident]-anchor)
    if len(planes)<len(prior['planes']):raise ValueError('prior_plane_missing')
    for plane,anchor,basis in zip(planes,prior['planes'],prior['plane_rotation_bases']):
        a=np.asarray(anchor['normal']);b=np.asarray(plane['normal'])
        cross=np.cross(a,b);sine=float(np.linalg.norm(cross));cosine=float(a@b)
        if cosine<-.999999:raise ValueError('prior_plane_antipodal')
        tangent=cross*(np.arctan2(sine,cosine)/sine) if sine>1e-12 else cross
        delta.extend(np.r_[np.asarray(basis).T@tangent,plane['distance']-anchor['distance']])
    j=np.asarray(prior['jacobian']);r=np.asarray(prior['residual'])
    if (j.ndim!=2 or j.shape[1]!=len(delta) or r.shape!=(j.shape[0],)
            or not np.isfinite(j).all() or not np.isfinite(r).all()):
        raise ValueError('invalid_frozen_prior')
    return j@np.asarray(delta)+r


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
    compact_j,compact_r=compact_square_root(reduced['jacobian'],reduced['residual'])
    plane_bases=[]
    for plane in planes:
        normal=np.asarray(plane['normal']);u=np.cross(normal,np.eye(3)[np.argmin(abs(normal))])
        u/=np.linalg.norm(u);plane_bases.append(np.column_stack([u,np.cross(normal,u)]))
    # Keep a compact rectangular square root. Inverting J'J or adding diagonal
    # regularization here would fill unobservable directions with false certainty.
    return dict(schema='posim-slam-sonar-frozen-prior-1',
        jacobian=compact_j,residual=compact_r,
        state_stamps_ns=keep,states=copy.deepcopy([states[i] for i in kept]),
        landmarks=copy.deepcopy(landmarks),planes=copy.deepcopy(planes),track_ids=list(track_ids),
        plane_rotation_bases=plane_bases,
        removed_state_stamps_ns=[stamps[i] for i in removed],
        eliminated_rank=reduced['nuisance_rank'],eliminated_nullity=reduced['nuisance_nullity'],
        tangent_order='states: position velocity right-attitude gyro-bias accel-bias; additive landmarks; plane tangent rotation and signed distance',
        scope='Frozen linearized joint information; reuse requires anchor-consistent deltas and measurement ownership',
        navigation_ready=False)
