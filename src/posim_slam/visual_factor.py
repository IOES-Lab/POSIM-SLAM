"""Rectified pinhole observations for shared pose/landmark optimization."""
import numpy as np
from .dvl_scale import rotation,vector
from .inertial import skew
from .inertial_filter import covariance


def linearize_visual_track(*,observations,states,landmark,measurement_covariance,
                           imu_to_camera,camera_to_imu,intrinsics,image_size,near,far):
    """Whiten predicted-minus-measured pixels; poses map IMU to local map.

    Optical camera axes are right X, down Y, forward Z. Images must already be
    rectified to the supplied pinhole intrinsics. Observations contain exact
    stamp_ns and pixel. Columns are per-observation pose (position/right attitude)
    followed by the shared map point. No VIO pose estimate is a measurement here.
    """
    count=len(observations)
    if not 1<=count<=16:raise ValueError('bounded_visual_track_required')
    stamps=[o['stamp_ns'] for o in observations]
    if any(type(t) is not int or t<0 for t in stamps) or any(b<=a for a,b in zip(stamps,stamps[1:])):
        raise ValueError('ordered_visual_stamps_required')
    k=np.asarray(intrinsics,dtype=float);size=np.asarray(image_size,dtype=float)
    if (k.shape!=(4,) or size.shape!=(2,) or not np.isfinite(k).all() or not np.isfinite(size).all()
            or np.any(k[:2]<=0) or np.any(size<=0) or not np.isfinite([near,far]).all() or not 0<near<far):
        raise ValueError('invalid_pinhole_calibration')
    mount=rotation(camera_to_imu);lever=vector(imu_to_camera);point=vector(landmark)
    whitening=np.linalg.cholesky(covariance(measurement_covariance,2*count))
    residual=np.empty(2*count);jacobian=np.zeros((2*count,6*count+3))
    for i,observation in enumerate(observations):
        if observation['stamp_ns'] not in states:raise ValueError('visual_state_timestamp_missing')
        pixel=np.asarray(observation['pixel'],dtype=float)
        if pixel.shape!=(2,) or not np.isfinite(pixel).all() or np.any(pixel<0) or np.any(pixel>=size):
            raise ValueError('invalid_visual_pixel')
        s=states[observation['stamp_ns']];r=rotation(s['attitude']);relative=r.T@(point-vector(s['position']))
        local=mount.T@(relative-lever);x,y,z=local
        if not near<z<far:raise ValueError('visual_point_outside_clip')
        predicted=k[:2]*local[:2]/z+k[2:]
        if np.any(predicted<0) or np.any(predicted>=size):raise ValueError('visual_point_outside_image')
        projection=np.array([[k[0]/z,0,-k[0]*x/z**2],[0,k[1]/z,-k[1]*y/z**2]])
        point_j=projection@mount.T@r.T;rows=slice(2*i,2*i+2)
        residual[rows]=predicted-pixel
        jacobian[rows,6*i:6*i+6]=np.column_stack([-point_j,projection@mount.T@skew(relative)])
        jacobian[rows,-3:]=point_j
    return dict(residual=np.linalg.solve(whitening,residual),jacobian=np.linalg.solve(whitening,jacobian))
