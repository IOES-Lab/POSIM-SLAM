"""Single-return range residual for joint pose/local-plane optimization.

The plane is an estimated variable, not known seabed height. Callers must
associate a supported visual patch and retain its correlation with the pose.
This module supplies geometry, not an independent EKF measurement of depth.
"""
import math
import numpy as np
from .dvl_scale import rotation, vector
from .inertial import skew
from .surface_support import require_surface_support


def supported_plane_range_factor(*,support_points,max_support_distance,**geometry):
    """Evaluate a factor only within its associated local visual patch.

    Support points must be plane inliers in the same map frame. Their pose
    and plane correlations still belong to the calling joint optimizer.
    """
    result=plane_range_factor(**geometry)
    require_surface_support(support_points,result['intersections'],geometry['plane_normal'],
                            max_distance=max_support_distance)
    return result


def plane_range_factor(*, position, attitude, imu_to_ping, ping_to_imu,
                       rays, plane_normal, plane_distance, measured_range,
                       range_bias, range_min, range_max):
    """Return prediction, measured-minus-predicted residual and Jacobians.

    All lengths are metres. Plane equation is n.dot(x) = d in the local
    map; attitude maps IMU to map, rays are unit vectors in the Ping frame.
    Pose Jacobian columns are additive map position and right IMU rotation.
    Plane columns are left normal rotation and additive signed distance.
    Jacobians differentiate the prediction (negate for the residual).

    The imported Orca sensor reports the minimum of its simulated rays.
    At a beam switch the minimum is not differentiable; every tied branch is
    returned so the optimizer can relinearize rather than average the rays.
    The caller must also verify the intersections lie inside observed support.
    """
    p=vector(position);r=rotation(attitude);lever=vector(imu_to_ping)
    mount=rotation(ping_to_imu);n=vector(plane_normal)
    directions=np.asarray(rays,dtype=float)
    if (directions.ndim!=2 or directions.shape[1]!=3 or not 1<=len(directions)<=256
        or not np.isfinite(directions).all()
        or not np.allclose(np.linalg.norm(directions,axis=1),1.,rtol=0,atol=1e-8)
        or not math.isclose(float(n@n),1.,rel_tol=0,abs_tol=1e-8)):
        raise ValueError('invalid_ping_geometry')
    if (not all(math.isfinite(v) for v in
                (plane_distance,measured_range,range_bias,range_min,range_max))
        or not 0<range_min<range_max or not range_min<=measured_range<=range_max):
        raise ValueError('invalid_ping_range')
    origin=p+r@lever
    body_rays=directions@mount.T
    world_rays=body_rays@r.T
    numerator=plane_distance-n@origin
    denominators=world_rays@n
    # Require a forward, non-grazing intersection for the whole footprint.
    # This is invariant to reversing the signed plane representation.
    if abs(numerator)<1e-9 or np.any(np.sign(numerator)*denominators<.1):
        raise ValueError('unsupported_ping_plane')
    distances=numerator/denominators
    nearest=float(np.min(distances))
    prediction=nearest+range_bias
    if nearest<range_min or nearest>range_max or not range_min<=prediction<=range_max:
        raise ValueError('predicted_ping_out_of_range')
    active=np.flatnonzero(np.abs(distances-nearest)<=1e-9)
    pose=[];plane=[]
    for index in active:
        b=denominators[index];t=distances[index];v=world_rays[index]
        pose.append(np.r_[-n/b,(n@r@skew(lever)+t*n@r@skew(body_rays[index]))/b])
        plane.append(np.r_[(origin+t*v)@skew(n)/b,1./b])
    return dict(predicted_range=float(prediction),residual=float(measured_range-prediction),
                active_rays=active,pose_jacobians=np.asarray(pose),
                plane_jacobians=np.asarray(plane),differentiable=len(active)==1,
                intersections=origin+distances[:,None]*world_rays)
