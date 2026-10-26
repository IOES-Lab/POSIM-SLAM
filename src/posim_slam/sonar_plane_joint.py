"""Joint pose / arbitrary-plane rows for explicitly associated sonar returns.

This experimental block does not discover physical surfaces in sonar images.
Callers own association, acoustic likelihood, finite support and joint priors.
"""
import numpy as np
from .dvl_scale import rotation, vector
from .inertial import skew
from .inertial_filter import covariance
from .ground_surface_return import plane_returns
from .surface_support import require_surface_support


def linearize_plane_patch(*, observations, states, plane_normal, plane_distance,
                          measurement_covariance, imu_to_sonar, sonar_to_imu,
                          range_min, range_max, support_points=None,
                          max_support_distance=None):
    """Whiten predicted-minus-measured rows with full range covariance.

    Each frame has stamp_ns, unique source_ids, range_m and unit rays shaped
    (returns, vertical rays, 3) in sonar coordinates. Columns are per-frame map
    translation/right IMU rotation, then additive unit-normal tangent (2) and
    signed plane distance (1). Only a unique first-return ray is differentiable.
    Optional support rejects extrapolation; it never treats an observed convex
    hull as a physical object boundary or as an independent plane measurement.
    """
    if not 1<=len(observations)<=16:raise ValueError('bounded_plane_frames_required')
    stamps=[o['stamp_ns'] for o in observations]
    if (any(type(t) is not int or t<=0 for t in stamps) or
            any(b<=a for a,b in zip(stamps,stamps[1:]))):raise ValueError('ordered_plane_stamps_required')
    if (support_points is None)!=(max_support_distance is None):raise ValueError('complete_plane_support_required')
    n=vector(plane_normal);lever=vector(imu_to_sonar);mount=rotation(sonar_to_imu)
    if not np.isclose(n@n,1.,atol=1e-8,rtol=0):raise ValueError('unit_plane_normal_required')
    axis=np.eye(3)[np.argmin(abs(n))];t=np.cross(n,axis);t/=np.linalg.norm(t)
    basis=np.column_stack((t,np.cross(n,t)))
    count=sum(len(o['range_m']) for o in observations)
    if not 3<=count<=512:raise ValueError('plane_patch_row_budget')
    noise=covariance(measurement_covariance,count)
    j=np.zeros((count,6*len(stamps)+3));residual=np.zeros(count);offset=0;hits=[];selected=[]
    for i,o in enumerate(observations):
        ranges=np.asarray(o['range_m'],dtype=float);rays=np.asarray(o['rays'],dtype=float);ids=o['source_ids']
        if (ranges.ndim!=1 or not np.isfinite(ranges).all() or np.any(ranges<range_min) or np.any(ranges>range_max)
                or rays.ndim!=3 or len(rays)!=len(ranges) or len(ids)!=len(ranges)
                or any(type(k) is not int or k<0 for k in ids) or len(set(ids))!=len(ids)):
            raise ValueError('invalid_plane_observations')
        if o['stamp_ns'] not in states:raise ValueError('plane_state_timestamp_missing')
        state=states[o['stamp_ns']];r=rotation(state['attitude']);p=vector(state['position'])
        body=rays@mount.T;world=body@r.T;origin=p+r@lever
        result=plane_returns(origin,world,n,plane_distance,range_min=range_min,range_max=range_max)
        if not result['valid'].all():raise ValueError('plane_return_unavailable')
        denominator=world@n;all_ranges=np.full(denominator.shape,np.inf)
        np.divide(plane_distance-n@origin,denominator,out=all_ranges,where=abs(denominator)>1e-8)
        all_ranges[(all_ranges<range_min)|(all_ranges>range_max)]=np.inf
        if np.any(np.sum(abs(all_ranges-result['range_m'][:,None])<=1e-9,axis=1)!=1):
            raise ValueError('plane_return_branch_ambiguous')
        if support_points is not None:
            require_surface_support(support_points,result['points'],n,max_distance=max_support_distance)
        for k,ray_index in enumerate(result['ray_indices']):
            jo=result['origin_jacobian'][k];jd=result['direction_jacobian'][k]
            residual[offset]=result['range_m'][k]-ranges[k]
            j[offset,6*i:6*i+3]=jo
            j[offset,6*i+3:6*i+6]=-jo@r@skew(lever)-jd@r@skew(body[k,ray_index])
            j[offset,-3:-1]=result['normal_jacobian'][k]@basis
            j[offset,-1]=result['distance_jacobian'][k]
            offset+=1
        hits.extend(result['points']);selected.append(result['ray_indices'])
    lower=np.linalg.cholesky(noise)
    residual=np.linalg.solve(lower,residual);j=np.linalg.solve(lower,j)
    if not np.isfinite(residual).all() or not np.isfinite(j).all():raise ValueError('nonfinite_plane_linearization')
    return dict(residual=residual,jacobian=j,
                state_stamps_ns=stamps,retained_columns=6*len(stamps),normal_basis=basis,
                intersections=np.array(hits),selected_rays=selected)


def marginal_plane_information(jacobian,pose_covariance):
    """Marginal plane covariance under an explicit independent Gaussian pose prior.

    The input Jacobian is already range-whitened, with pose columns then three
    plane columns. Preserve full inter-frame pose correlations. This integration
    is valid only when the supplied pose prior is independent of these sonar
    rows; a posterior containing the same rows would double-count observations.
    Zero pose covariance gives the conditional fixed-pose diagnostic. No pose
    prior is invented, and this function does not update an estimator state.
    """
    j=np.asarray(jacobian,dtype=float);p=np.asarray(pose_covariance,dtype=float)
    if (j.ndim!=2 or not 3<=len(j)<=512 or not 9<=j.shape[1]<=99
            or (j.shape[1]-3)%6 or p.shape!=(j.shape[1]-3,j.shape[1]-3)
            or not np.isfinite(j).all() or not np.isfinite(p).all()
            or not np.allclose(p,p.T,atol=1e-12,rtol=0)):
        raise ValueError('invalid_plane_pose_covariance')
    if np.linalg.eigvalsh(p).min() < -1e-12:raise ValueError('invalid_plane_pose_covariance')
    a=j[:,:-3];b=j[:,-3:]
    effective=np.eye(len(j))+a@p@a.T
    if not np.isfinite(effective).all():raise ValueError('nonfinite_plane_marginal')
    white=np.linalg.solve(np.linalg.cholesky(effective),b)
    _,singular,v=np.linalg.svd(white,full_matrices=False)
    full=bool(singular[-1]>1e-10*singular[0])
    c=(v.T/singular**2)@v if full else None
    if c is not None and not np.isfinite(c).all():raise ValueError('nonfinite_plane_marginal')
    return dict(full_rank=full,singular_values=singular,
                plane_covariance=c,
                information=white.T@white,
                scope='Local plane marginal under caller-supplied independent pose prior; no association or branch uncertainty')
