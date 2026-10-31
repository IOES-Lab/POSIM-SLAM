"""Joint Ping/local-plane/landmark linearization for a visual-inertial solver.

These rows must be assembled with raw visual and IMU residuals before eliminating
landmarks. A fitted plane is not an independent external depth measurement.
"""
import math
import numpy as np
from .inertial import skew
from .ping_factor import supported_plane_range_factor


def linearize_ping_patch(*,landmarks,range_sigma,surface_sigma,
                         max_support_distance,**geometry):
    """Whitened predicted-minus-measured residual and Jacobian.

    Columns: pose position/right attitude (6), plane tangent rotation/distance
    (3), additive map landmark positions (3N). The returned normal_basis maps
    the two plane rotation increments into a left SO(3) perturbation.
    surface_sigma describes physical departure from local planarity; it is
    not a replacement for landmark covariance or visual reprojection noise.
    range_sigma must describe the returned single range, not each simulated ray.
    """
    if any(not math.isfinite(x) or x<=0 for x in (range_sigma,surface_sigma)):
        raise ValueError('ping_and_surface_noise_required')
    points=np.asarray(landmarks,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3 or not 3<=len(points)<=256:
        raise ValueError('bounded_ping_patch_required')
    f=supported_plane_range_factor(support_points=points,
        max_support_distance=max_support_distance,**geometry)
    if not f['differentiable']:raise ValueError('ping_beam_switch_requires_reassociation')
    normal=np.asarray(geometry['plane_normal'],dtype=float)
    axis=np.eye(3)[np.argmin(abs(normal))]
    u=np.cross(normal,axis);u/=np.linalg.norm(u)
    basis=np.column_stack([u,np.cross(normal,u)])
    count=len(points);j=np.zeros((count+1,9+3*count));residual=np.zeros(count+1)
    residual[0]=-f['residual']/range_sigma
    j[0,:6]=f['pose_jacobians'][0]/range_sigma
    j[0,6:8]=f['plane_jacobians'][0,:3]@basis/range_sigma
    j[0,8]=f['plane_jacobians'][0,3]/range_sigma
    residual[1:]=(points@normal-geometry['plane_distance'])/surface_sigma
    j[1:,6:8]=-points@skew(normal)@basis/surface_sigma
    j[1:,8]=-1./surface_sigma
    for i in range(count):j[i+1,9+3*i:12+3*i]=normal/surface_sigma
    return dict(residual=residual,jacobian=j,normal_basis=basis,
                predicted_range=f['predicted_range'])


def eliminate_patch(jacobian,residual,*,retained_columns):
    """Rank-aware square-root elimination after assembling all shared rows.

    Retained variables precede the nuisance plane/landmark variables. Returns
    reduced J/r plus the minimum-norm nuisance back-substitution affine map.
    Rank deficiency is retained as a nullspace, never filled by an epsilon prior.
    This bounded reference implementation uses SVD; native integration may use
    rank-revealing QR with equivalent residuals and covariance.
    """
    j=np.asarray(jacobian,dtype=float);r=np.asarray(residual,dtype=float)
    if (j.ndim!=2 or not 1<=j.shape[0]<=4096 or not 1<=j.shape[1]<=1024
        or r.shape!=(j.shape[0],) or not np.isfinite(j).all() or not np.isfinite(r).all()
        or type(retained_columns) is not int or not 0<retained_columns<j.shape[1]):
        raise ValueError('invalid_joint_patch_system')
    return split_patch_information(j,r,retained_columns=retained_columns,include_conditional=False)


def split_patch_information(jacobian,residual,*,retained_columns,include_conditional=True):
    """Separate marginal and conditional square-root information exactly.

    Input columns are retained coordinates followed by retiring coordinates.
    The two output squared norms sum to the original norm for EVERY coordinate
    value. A conditional keeps its regression, observable noise factor and
    unbounded null directions. Its mode is not an independent pose observation.
    Endpoints (no retiring/retained coordinates) and empty information are valid.
    Callers may QR-compress [J r] first to bound work without losing constants.
    """
    j=np.asarray(jacobian,dtype=float);r=np.asarray(residual,dtype=float)
    if (j.ndim!=2 or not 0<=j.shape[0]<=4096 or not 0<=j.shape[1]<=1024
            or r.shape!=(len(j),) or not np.isfinite(j).all() or not np.isfinite(r).all()
            or type(retained_columns) is not int or not 0<=retained_columns<=j.shape[1]
            or type(include_conditional) is not bool):raise ValueError('invalid_joint_information_split')
    a=j[:,:retained_columns];b=j[:,retained_columns:];nuisance=b.shape[1]
    if nuisance==0 or not len(j):
        result=dict(jacobian=a.copy(),residual=r.copy(),nuisance_rank=0,nuisance_nullity=nuisance,
            backsub_offset=np.zeros(nuisance),backsub_matrix=np.zeros((nuisance,retained_columns)))
        if include_conditional:result.update(conditional_jacobian=np.zeros((0,j.shape[1])),conditional_residual=np.zeros(0),
            observable_noise_factor=np.zeros((nuisance,0)),unobservable_basis=np.eye(nuisance))
        return result
    u,s,vt=np.linalg.svd(b,full_matrices=True)
    tolerance=max(b.shape)*np.finfo(float).eps*(s[0] if len(s) else 0.)
    rank=int(np.sum(s>tolerance));null=u[:,rank:]
    inverse=(vt[:rank].T/s[:rank])@u[:,:rank].T
    result=dict(jacobian=null.T@a,residual=null.T@r,nuisance_rank=rank,
                nuisance_nullity=b.shape[1]-rank,
                backsub_offset=-inverse@r,backsub_matrix=-inverse@a)
    if include_conditional:result.update(conditional_jacobian=u[:,:rank].T@j,conditional_residual=u[:,:rank].T@r,
        observable_noise_factor=vt[:rank].T/s[:rank],unobservable_basis=vt[rank:].T)
    return result
