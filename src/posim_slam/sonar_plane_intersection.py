"""Conditional intersections of measured FLS range/azimuth and an unknown plane.

The image does not measure elevation. A caller-supplied associated plane can
leave zero, one, two or infinitely many elevations inside the vertical fan.
Keep every branch; a unique conditional solution is not an observed landmark.
"""
import math
import numpy as np

from .dvl_scale import rotation, vector


def range_azimuth_plane(*, ranges, azimuths, origin, sonar_to_map,
                        plane_normal, plane_distance, vertical_fov_rad):
    """Return both analytic branches of n·(o + r R u(az,el)) = d.

    count=-1 denotes a continuum of elevations. Unavailable branches contain
    NaN. The derivative of the normalized plane equation with respect to
    elevation is retained to expose tangent/ill-conditioned intersections.
    This conditions on exact poses/plane; it supplies no noise, support,
    physical correspondence, occlusion check or joint covariance.
    """
    ranges=np.asarray(ranges,dtype=float);azimuths=np.asarray(azimuths,dtype=float)
    o=vector(origin);r=rotation(sonar_to_map);n=vector(plane_normal)
    if (ranges.ndim!=1 or not 1<=len(ranges)<=4096 or azimuths.shape!=ranges.shape
            or not np.isfinite(ranges).all() or not np.isfinite(azimuths).all()
            or np.any(ranges<=0) or np.any(abs(azimuths)>=math.pi/2)
            or not math.isclose(float(n@n),1.,rel_tol=0,abs_tol=1e-8)
            or not math.isfinite(plane_distance) or not math.isfinite(vertical_fov_rad)
            or not 0<vertical_fov_rad<math.pi):
        raise ValueError('invalid_sonar_plane_intersection')
    normal=r.T@n;height=float(plane_distance-n@o)
    a=normal[0]*np.cos(azimuths)+normal[1]*np.sin(azimuths)
    b=np.full(len(ranges),normal[2]);c=height/ranges;radius=np.hypot(a,b)
    count=np.zeros(len(ranges),dtype=np.int64)
    elevations=np.full((len(ranges),2),np.nan)
    derivative=np.full_like(elevations,np.nan)
    points=np.full((len(ranges),2,3),np.nan)
    # Roundoff tolerance only, not a measurement/model uncertainty allowance.
    tolerance=1e-12;half=vertical_fov_rad/2
    continuum=(radius<tolerance)&(abs(c)<tolerance)
    count[continuum]=-1
    feasible=(radius>=tolerance)&(abs(c)<=radius+tolerance)
    ratio=np.divide(c,radius,out=np.zeros_like(c),where=radius>=tolerance)
    phase=np.arctan2(b,a);delta=np.arccos(np.clip(ratio,-1,1))
    candidates=np.column_stack((phase-delta,phase+delta))
    candidates=(candidates+math.pi)%(2*math.pi)-math.pi
    candidates.sort(axis=1)
    for i in np.flatnonzero(feasible):
        accepted=[]
        for elevation in candidates[i]:
            if abs(elevation)>half+tolerance:continue
            elevation=float(np.clip(elevation,-half,half))
            if accepted and abs(elevation-accepted[-1])<=tolerance:continue
            accepted.append(elevation)
        count[i]=len(accepted)
        for branch,elevation in enumerate(accepted):
            elevations[i,branch]=elevation
            ray=np.array([math.cos(azimuths[i])*math.cos(elevation),
                          math.sin(azimuths[i])*math.cos(elevation),math.sin(elevation)])
            points[i,branch]=o+ranges[i]*(r@ray)
            derivative[i,branch]=-a[i]*math.sin(elevation)+b[i]*math.cos(elevation)
    return dict(count=count,elevations=elevations,points=points,
        plane_elevation_derivative=derivative,map_qualified=False,
        physical_identity_verified=False,
        scope='All conditional range/azimuth-plane branches; elevation not measured')
