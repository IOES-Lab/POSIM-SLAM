"""Estimate monocular scale from a locally planar seabed and the imported Ping.

Inputs are camera-frame visual landmarks and a time-aligned range measurement.
No simulator position, depth map or ground-truth transform is used.
"""
from collections import deque
import math
import numpy as np

# Optical axes: x right, y down in image, z forward. Derived from Orca5's
# camera link + sensor poses and Ping link pose (metres, not visual units).
PING_ORIGIN = np.array([0., .05, .095])
ANGLES = np.linspace(-.14835,.14835,5)
PING_DIRECTIONS = np.array([[-math.cos(v)*math.sin(h), math.sin(v), math.cos(v)*math.cos(h)]
                            for v in ANGLES for h in ANGLES])
PING_BIAS_M = .1  # Gaussian mean in the pinned imported SDF.


def candidate_scale(points_camera, range_m, *, ping_origin=PING_ORIGIN, ping_directions=PING_DIRECTIONS, ping_bias_m=PING_BIAS_M, range_min=.2, range_max=50.):
    """Return a scale candidate and diagnostics, or reject an unsupported patch.

The scan's nearest return intersects the locally fitted plane. Its sensor
translation remains in metres; multiplying that offset by visual scale would
introduce a systematic error. This approximation is rejected on rough patches.
"""
    origin=np.asarray(ping_origin,dtype=float);rays=np.asarray(ping_directions,dtype=float)
    if (origin.shape!=(3,) or rays.ndim!=2 or rays.shape[1]!=3 or len(rays)!=25
        or not np.isfinite(origin).all() or not np.isfinite(rays).all()
        or not np.allclose(np.linalg.norm(rays,axis=1),1,atol=1e-6)
        or np.any(rays[:,2]<=.7)
        or any(not math.isfinite(v) for v in (ping_bias_m,range_min,range_max))
        or not 0<range_min<range_max or abs(ping_bias_m)>=range_min):
        raise ValueError('invalid_ping_calibration')
    points = np.asarray(points_camera,dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError('invalid_landmarks')
    if not math.isfinite(range_m) or not range_min <= range_m <= range_max:
        raise ValueError('invalid_ping')
    corrected = range_m-ping_bias_m
    # A wider camera patch encloses the Ping footprint and its small baseline.
    front = points[:,2] > 0
    points = points[front]
    points = points[np.linalg.norm(points[:,:2],axis=1) < .35*points[:,2]]
    if len(points)<20:
        raise ValueError('insufficient_surface')
    # Deterministic RANSAC protects the plane from unrelated visible objects.
    rng = np.random.default_rng(0)
    threshold = .015*np.median(points[:,2])
    best = np.zeros(len(points),dtype=bool)
    for _ in range(64):
        a,b,c = points[rng.choice(len(points),3,replace=False)]
        normal = np.cross(b-a,c-a)
        length = np.linalg.norm(normal)
        if length<1e-12:continue
        normal /= length
        inliers = np.abs((points-a)@normal)<threshold
        if inliers.sum()>best.sum():best=inliers
    if best.sum()<20 or best.mean()<.8:
        raise ValueError('nonplanar_surface')
    patch = points[best]
    center = patch.mean(axis=0)
    _, singular, axes = np.linalg.svd(patch-center,full_matrices=False)
    if singular[1] < .05*singular[0] or singular[1]<1e-9:
        raise ValueError('degenerate_surface')
    normal = axes[-1]
    if normal[2]<0:normal=-normal
    distance = float(normal@center)
    if normal[2]<.7 or distance<=0:
        raise ValueError('grazing_surface')
    residual = float(np.sqrt(np.mean(((patch-center)@normal)**2))/distance)
    if residual>.01:
        raise ValueError('rough_surface')
    scale = float((normal@origin+corrected*np.max(rays@normal))/distance)
    # Check that the predicted acoustic intersections are inside observed patch
    # support, rather than extrapolating far beyond the visual measurements.
    ranges = (scale*distance-normal@origin)/(rays@normal)
    intersections = (origin+ranges[:,None]*rays)/scale
    if np.any(intersections[:,:2].min(axis=0)<patch[:,:2].min(axis=0)) or np.any(intersections[:,:2].max(axis=0)>patch[:,:2].max(axis=0)):
        raise ValueError('surface_does_not_cover_ping')
    if not math.isfinite(scale) or not 1e-4 < scale < 1e4:
        raise ValueError('invalid_scale')
    return {'scale':scale,'inliers':int(best.sum()),'relative_residual':residual}


class ScaleWindow:
    """Require repeated independent, consistent candidates before metric output."""
    def __init__(self,**calibration):
        self.calibration=calibration
        self.values=deque(maxlen=15)
        self.last_stamp=None
    def reset(self):
        self.values.clear();self.last_stamp=None
    def add(self, stamp, points_camera, range_m, ping_stamp):
        if not math.isfinite(stamp) or not math.isfinite(ping_stamp):
            raise ValueError('invalid_timestamp')
        if self.last_stamp is not None and (stamp<self.last_stamp or stamp-self.last_stamp>1):
            self.reset()
        if self.last_stamp is not None and stamp<=self.last_stamp:
            raise ValueError('duplicate_timestamp')
        self.last_stamp=stamp
        if abs(stamp-ping_stamp)>.1+1e-6:
            self.values.clear()
            raise ValueError('ping_not_synchronized')
        try:
            candidate=candidate_scale(points_camera,range_m,**self.calibration)
        except ValueError:
            self.values.clear()
            raise
        self.values.append(candidate['scale'])
        values=np.asarray(self.values)
        median=float(np.median(values))
        spread=float(np.max(np.abs(values-median))/median)
        return dict(candidate,scale=median,observations=len(values),relative_spread=spread,
                    ready=len(values)>=8 and spread<=.05)
