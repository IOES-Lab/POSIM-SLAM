"""Range/azimuth geometry for an associated imaging-sonar point return.

Elevation is unmeasured. Association, echo extent, uncertainty, outlier rejection
and the correlated joint optimizer belong to the caller. A collapsed intensity
bin is not automatically a point landmark or a seabed-plane observation.
"""
import math
import numpy as np
from .dvl_scale import rotation, vector
from .inertial import skew


def within_fov(angle, fov):
    """Accept float32 unit-direction endpoint roundoff without clamping data."""
    return (math.isfinite(angle) and math.isfinite(fov) and fov>0
            and abs(angle)<=fov/2+4*float(np.finfo(np.float32).eps))


def point_return_factor(*, position, attitude, imu_to_sonar, sonar_to_imu,
                        landmark, measured_range, measured_azimuth,
                        range_min, range_max, horizontal_fov, vertical_fov):
    """Measured-minus-predicted range (m) and azimuth (rad), with Jacobians.

    Sonar axes are forward X, left Y, up Z. FOV values are full angles in
    radians. The return must be associated with a point in the joint map.
    Attitude maps IMU to map; imu_to_sonar is the lever arm in IMU axes.
    Pose columns are additive map position and right IMU rotation. Landmark
    columns are additive map coordinates. Jacobians differentiate prediction,
    so the residual derivatives have the opposite sign.

    A return constrains two dimensions, leaving elevation unresolved. Do not
    convert its landmark to a fan-plane point or supply an artificial Z prior.
    """
    p=vector(position);r=rotation(attitude);lever=vector(imu_to_sonar)
    mount=rotation(sonar_to_imu);point=vector(landmark)
    scalars=(measured_range,measured_azimuth,range_min,range_max,horizontal_fov,vertical_fov)
    if (not all(math.isfinite(v) for v in scalars) or
            not 0<range_min<range_max or measured_range<=0 or
            not 0<horizontal_fov<=math.pi or not 0<vertical_fov<math.pi or
            not within_fov(measured_azimuth,horizontal_fov)):
        raise ValueError('invalid_sonar_measurement')
    # The interval constrains the latent physical point, not its noisy sampled
    # range. The source input contract checks valid published coordinates.
    relative=r.T@(point-p)
    local=mount.T@(relative-lever)
    x,y,z=local;horizontal2=x*x+y*y
    distance=float(np.linalg.norm(local))
    if x<=0 or horizontal2<1e-12 or not range_min<=distance<=range_max:
        raise ValueError('unsupported_sonar_point')
    azimuth=math.atan2(y,x);elevation=math.atan2(z,math.sqrt(horizontal2))
    if not within_fov(azimuth,horizontal_fov) or not within_fov(elevation,vertical_fov):
        raise ValueError('sonar_point_outside_fov')
    projection=np.array([local/distance,[-y/horizontal2,x/horizontal2,0.]])
    landmark_jacobian=projection@mount.T@r.T
    pose_jacobian=np.column_stack((-landmark_jacobian,projection@mount.T@skew(relative)))
    angle_error=math.atan2(math.sin(measured_azimuth-azimuth),math.cos(measured_azimuth-azimuth))
    return dict(prediction=np.array([distance,azimuth]),
                residual=np.array([measured_range-distance,angle_error]),
                pose_jacobian=pose_jacobian,landmark_jacobian=landmark_jacobian)
