# SPDX-License-Identifier: Apache-2.0
"""Conditional geometry of one opaque edge and its shadow on a supplied plane.

The measured near boundary MUST be the occluding edge's range, not an arbitrary
dark pixel. The far boundary MUST be the shadow's return to the same planar
ground. Low reflectivity, multiple occluders, fan truncation and a wrong local
plane can invalidate these assumptions. This module cannot identify them from
two scalar ranges. No native update or map admission is provided.
"""
import math
import numpy as np


def plane_ray(*, azimuth_rad, range_m, normal, distance_m, vertical_fov_rad):
    """All sphere/plane intersections on a calibrated elevation arc, analytic."""
    n = np.asarray(normal,dtype=float)
    if (n.shape != (3,) or not np.isfinite(n).all()
            or not np.isclose(np.linalg.norm(n),1.,atol=1e-9,rtol=0)
            or not np.isfinite([azimuth_rad,range_m,distance_m,vertical_fov_rad]).all()
            or not -math.pi/2 < azimuth_rad < math.pi/2
            or range_m<=0 or distance_m<=0 or not 0<vertical_fov_rad<math.pi):
        raise ValueError('invalid_shadow_plane_geometry')
    a = n[0]*math.cos(azimuth_rad)+n[1]*math.sin(azimuth_rad)
    b = n[2]
    magnitude = math.hypot(a,b)
    c = distance_m/range_m
    if magnitude == 0 or c>magnitude+1e-12:
        return []
    phase=math.atan2(b,a)
    offset=math.acos(np.clip(c/magnitude,-1.,1.))
    angles=[]
    for value in [phase-offset,phase+offset]:
        value=(value+math.pi)%(2*math.pi)-math.pi
        if abs(value)<=vertical_fov_rad/2+1e-12 and all(abs(value-e)>1e-10 for e in angles):
            angles.append(value)
    return sorted(angles)


def shadow_edge(*, edge_range_m, shadow_end_range_m, azimuth_rad,
                plane_normal, plane_distance_m, vertical_fov_rad,
                measurement_covariance=None):
    """Return one conditional edge point, or preserve unavailable geometry.

    Coordinates are in the sonar frame. Plane n dot X = H has n pointing from
    the sensor toward the ground and H>0. Collinearity of sensor, occluding edge
    and shadow endpoint gives h=H*(1-r_edge/r_end).

    Optional full 7x7 covariance order is r_edge, r_end, azimuth, H, then the
    three left-rotation coordinates of n. Cross-correlation is preserved.
    Results remain conditional on actual edge/shadow association, local plane
    validity and fixed sensor pose; missing uncertainty is never replaced by 0.
    """
    if (not np.isfinite([edge_range_m,shadow_end_range_m]).all()
            or not 0<edge_range_m<shadow_end_range_m):
        raise ValueError('ordered_positive_shadow_ranges_required')
    n=np.asarray(plane_normal,dtype=float)
    solutions=plane_ray(azimuth_rad=azimuth_rad,range_m=shadow_end_range_m,
         normal=n,distance_m=plane_distance_m,vertical_fov_rad=vertical_fov_rad)
    base=dict(available=False,map_qualified=False,physical_identity_verified=False,
              height_m=float(plane_distance_m*(1-edge_range_m/shadow_end_range_m)),
              covariance_available=False,conditional=True)
    cov=None
    if measurement_covariance is not None:
        cov=np.asarray(measurement_covariance,dtype=float)
        if (cov.shape!=(7,7) or not np.isfinite(cov).all()
                or not np.allclose(cov,cov.T,atol=1e-14,rtol=1e-12)
                or np.linalg.eigvalsh(cov).min() < -1e-12*max(np.finfo(float).tiny,np.linalg.norm(cov))):
            raise ValueError('explicit_shadow_covariance_must_be_positive_semidefinite')
    if len(solutions)!=1:
        return dict(base,reason='no_ground_intersection' if not solutions else 'ambiguous_ground_intersection',
                    elevation_solutions_rad=solutions)
    e=solutions[0]
    u=np.array([math.cos(e)*math.cos(azimuth_rad),math.cos(e)*math.sin(azimuth_rad),math.sin(e)])
    ue=np.array([-math.sin(e)*math.cos(azimuth_rad),-math.sin(e)*math.sin(azimuth_rad),math.cos(e)])
    ua=np.array([-math.cos(e)*math.sin(azimuth_rad),math.cos(e)*math.cos(azimuth_rad),0.])
    derivative=float(n@ue)
    if abs(derivative)<1e-9:
        return dict(base,reason='tangent_ground_intersection',elevation_solutions_rad=solutions)
    j=np.zeros((3,7))
    j[:,0]=u
    j[:,1]=edge_range_m*ue*(-plane_distance_m/(shadow_end_range_m**2*derivative))
    j[:,2]=edge_range_m*(ua-ue*(n@ua)/derivative)
    j[:,3]=edge_range_m*ue/(shadow_end_range_m*derivative)
    j[:,4:]=-edge_range_m*np.outer(ue,np.cross(n,u))/derivative
    jh=np.array([-plane_distance_m/shadow_end_range_m,
                  plane_distance_m*edge_range_m/shadow_end_range_m**2,
                  0.,1-edge_range_m/shadow_end_range_m,0.,0.,0.])
    result=dict(base,available=True,reason='conditional_opaque_edge',
                point=edge_range_m*u,shadow_endpoint=shadow_end_range_m*u,
                elevation_rad=float(e),point_jacobian=j,height_jacobian=jh)
    if cov is not None:
        result.update(covariance_available=True,point_covariance=j@cov@j.T,
                      height_variance_m2=float(jh@cov@jh))
    return result
