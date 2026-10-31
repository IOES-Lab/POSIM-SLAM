"""Conditional range factors for finite DVL cones and a supported local plane.

Gazebo's bottom range is the minimum slant return inside a cone. It is not a
centre-ray XYZ measurement. The unknown plane and its correlation with poses
belong to a calling joint estimator. This module does not assign a plane to a
sonar echo, infer range noise from resolution, or enable a live SLAM mode.
"""
from __future__ import annotations

import math
import numpy as np

from .dvl_scale import rotation, vector
from .inertial import skew
from .surface_support import _convex_hull


def cone_plane_range_factor(*, position, attitude, imu_to_dvl, dvl_to_imu,
                            beam_axis, aperture_rad, plane_normal,
                            plane_distance, measured_range, range_min,
                            range_max):
    """Predict the earliest continuous-cone return from n.dot(X) = d.

    Axes are in the published DVL reference frame; dvl_to_imu maps that frame
    to the IMU. Attitude maps IMU to map, with its lever arm included.
    aperture_rad is the FULL cone angle. Both signs of a plane are supported.

    Prediction Jacobian columns: additive map position, right IMU rotation;
    left plane-normal rotation, additive signed plane distance. Residual is
    measured minus predicted, so its Jacobians have the opposite sign.
    The footprint must be bounded and the first return within sensor limits.
    """
    p=vector(position);r=rotation(attitude);lever=vector(imu_to_dvl)
    mount=rotation(dvl_to_imu);a=vector(beam_axis);n=vector(plane_normal)
    if (not math.isclose(float(a@a),1.,rel_tol=0,abs_tol=1e-8)
            or not math.isclose(float(n@n),1.,rel_tol=0,abs_tol=1e-8)
            or not all(math.isfinite(v) for v in
                (aperture_rad,plane_distance,measured_range,range_min,range_max))
            or not 0<=aperture_rad<math.pi or not 0<range_min<range_max
            or not range_min<=measured_range<=range_max):
        raise ValueError('invalid_dvl_cone_geometry')
    original_plane=np.r_[n,plane_distance]
    origin=p+r@lever;body_axis=mount@a;axis=r@body_axis
    height=float(plane_distance-n@origin)
    if abs(height)<1e-9:raise ValueError('dvl_origin_on_plane')
    sign=1. if height>0 else -1.
    n=sign*n;height=sign*height
    half=aperture_rad/2;sa=math.sin(half);ca=math.cos(half)
    cosine=float(np.clip(n@axis,-1,1))
    # This exact condition makes the entire plane footprint an ellipse.
    # A cone containing a grazing ray has no bounded support footprint.
    if cosine<=sa+1e-10:raise ValueError('unbounded_dvl_plane_footprint')
    if cosine>=ca:
        maximum=1.;derivative=0.;hit_direction=n
    else:
        sine=math.sqrt(max(0.,1-cosine*cosine))
        maximum=cosine*ca+sine*sa
        derivative=ca-cosine/sine*sa
        hit_direction=ca*axis+sa*(n-cosine*axis)/sine
    predicted=height/maximum
    if not range_min<=predicted<=range_max:
        raise ValueError('dvl_first_return_out_of_range')
    gradient_axis=-height*derivative*n/(maximum*maximum)
    pose=np.r_[-n/maximum,
        n@r@skew(lever)/maximum-gradient_axis@r@skew(body_axis)]
    normal_gradient=-origin/maximum-height*derivative*axis/(maximum*maximum)
    plane=np.r_[-normal_gradient@skew(n),sign/maximum]

    helper=np.eye(3)[np.argmin(np.abs(n))]
    u=np.cross(n,helper);u/=np.linalg.norm(u);basis=np.column_stack((u,np.cross(n,u)))
    b=basis.T@axis;denominator=cosine*cosine-sa*sa
    q=ca*ca*np.eye(2)-np.outer(b,b)
    center2=height*cosine*b/denominator
    radius2=height*height*ca*ca*sa*sa/denominator
    shape2=radius2*np.linalg.inv(q)
    center=origin+height*n+basis@center2
    return dict(predicted_range=float(predicted),residual=float(measured_range-predicted),
        pose_jacobian=pose,plane_jacobian=plane,plane=original_plane,
        state_reference=np.r_[p,r.ravel(),lever,mount.ravel()],
        conditional_hit=origin+predicted*hit_direction,
        footprint_center=center,footprint_basis=basis,footprint_shape2=shape2,
        supported=False,physical_identity_verified=False,map_qualified=False,
        scope='Continuous finite-cone minimum on an associated plane; no observed XYZ, noise or navigation qualification')


def require_cone_surface_support(factor, support_points, *, max_plane_deviation,
                                 max_support_distance):
    """Require the ENTIRE ellipse within a local observed planar hull.

    A sufficient nearest-sample disk bound also rejects large unseen holes.
    This is conservative and assumes local planarity; sparse samples cannot
    prove dense surface occupancy or rule out an unobserved nearer occluder.
    It does not discard or alter a failed factor's original measurement.
    """
    points=np.asarray(support_points,dtype=float)
    if (points.ndim!=2 or points.shape[1]!=3 or not 3<=len(points)<=4096
            or not np.isfinite(points).all()
            or not all(math.isfinite(v) and v>0 for v in (max_plane_deviation,max_support_distance))):
        raise ValueError('invalid_dvl_surface_support')
    n=factor['plane'][:3];distance=factor['plane'][3]
    if np.any(abs(points@n-distance)>max_plane_deviation):
        raise ValueError('dvl_support_not_planar')
    scale=max_support_distance
    samples=(points-factor['footprint_center'])@factor['footprint_basis']/scale
    shape=factor['footprint_shape2']/(scale*scale)
    hull=_convex_hull(samples)
    if len(hull)<3:raise ValueError('degenerate_dvl_surface_support')
    for first,second in zip(hull,hull[1:]+hull[:1]):
        edge=np.asarray(second)-first;outward=np.array([edge[1],-edge[0]])
        radius=math.sqrt(max(0.,float(outward@shape@outward)))
        if radius>float(outward@first)+1e-10*np.linalg.norm(outward):
            raise ValueError('dvl_cone_outside_surface_support')
    # Triangle inequality covers every ellipse point, not just sampled rays.
    radius=math.sqrt(max(0.,float(np.linalg.eigvalsh(shape)[-1])))
    if np.min(np.linalg.norm(samples,axis=1))+radius>1.+1e-10:
        raise ValueError('dvl_surface_support_gap')
    return dict(factor,supported=True)


def whiten_plane_ranges(factors, covariance, *, uncertainty_source):
    """Keep full caller-supplied range covariance, including shared errors.

    Every factor must use the SAME pose, mounting and signed map plane. Source variances may be
    unknown: callers must supply an explicit positive-definite model and label
    its provenance. Resolution and a protobuf default zero are not a model.
    Pose/plane information here remains conditional, not joint covariance.
    """
    if (not 1<=len(factors)<=64 or not isinstance(uncertainty_source,str)
            or not uncertainty_source.strip()):
        raise ValueError('explicit_dvl_range_uncertainty_required')
    if any(not np.array_equal(f['plane'],factors[0]['plane']) for f in factors):
        raise ValueError('shared_dvl_plane_required')
    if any(not np.array_equal(f['state_reference'],factors[0]['state_reference']) for f in factors):
        raise ValueError('shared_dvl_state_required')
    cov=np.asarray(covariance,dtype=float)
    if (cov.shape!=(len(factors),len(factors)) or not np.isfinite(cov).all()
            or not np.allclose(cov,cov.T,rtol=1e-12,atol=0)):
        raise ValueError('invalid_dvl_range_covariance')
    try:lower=np.linalg.cholesky(cov)
    except np.linalg.LinAlgError as error:raise ValueError('invalid_dvl_range_covariance') from error
    residual=np.linalg.solve(lower,[f['residual'] for f in factors])
    pose=np.linalg.solve(lower,np.array([f['pose_jacobian'] for f in factors]))
    plane=np.linalg.solve(lower,np.array([f['plane_jacobian'] for f in factors]))
    return dict(residual=residual,pose_jacobian=pose,plane_jacobian=plane,
        uncertainty_source=uncertainty_source,supported=all(f['supported'] for f in factors),
        pose_information_eigenvalues=np.linalg.eigvalsh(pose.T@pose),
        map_qualified=False,scope='Conditional range linearization; no prior map, plane identity or joint state covariance')
