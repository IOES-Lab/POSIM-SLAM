"""Joint pose/point rows for a single associated acoustic observation chain.

This is a bounded solver building block, not a trajectory estimator. Point
initialization, physical association and measurement covariance are explicit
caller responsibilities. Assemble shared visual/Ping/IMU rows before elimination.
"""
import numpy as np
from .sonar_factor import point_return_factor
from .inertial_filter import covariance


def linearize_track(*, observations, states, landmark, measurement_covariance,
                    imu_to_sonar, sonar_to_imu, range_min, range_max,
                    horizontal_fov, vertical_fov):
    """Whiten predicted-minus-measured residuals with full temporal covariance.

    Each observation has stamp_ns, range_m and azimuth_rad. States are keyed by
    the exact integer stamp and contain position and attitude. Columns are each
    observed pose (map translation, right IMU rotation), then the shared additive
    3D landmark. The covariance order is [range_0, azimuth_0, range_1, ...].

    No pose prior or elevation constraint is added. Use the existing rank-aware
    ping_joint.eliminate_patch only after assembling all rows sharing this point;
    eliminating independently would lose its correlation with visual/Ping rows.
    """
    count=len(observations)
    if not 1<=count<=16:raise ValueError('bounded_sonar_track_required')
    stamps=[o['stamp_ns'] for o in observations]
    if (any(type(t) is not int or t<0 for t in stamps) or
            any(b<=a for a,b in zip(stamps,stamps[1:]))):
        raise ValueError('ordered_sonar_stamps_required')
    noise=covariance(measurement_covariance,2*count)
    whitening=np.linalg.cholesky(noise)
    residual=np.empty(2*count);jacobian=np.zeros((2*count,6*count+3))
    for i,(stamp,observation) in enumerate(zip(stamps,observations)):
        if stamp not in states:raise ValueError('sonar_state_timestamp_missing')
        state=states[stamp]
        factor=point_return_factor(position=state['position'],attitude=state['attitude'],
            imu_to_sonar=imu_to_sonar,sonar_to_imu=sonar_to_imu,landmark=landmark,
            measured_range=observation['range_m'],measured_azimuth=observation['azimuth_rad'],
            range_min=range_min,range_max=range_max,horizontal_fov=horizontal_fov,vertical_fov=vertical_fov)
        rows=slice(2*i,2*i+2)
        residual[rows]=-factor['residual']
        jacobian[rows,6*i:6*i+6]=factor['pose_jacobian']
        jacobian[rows,-3:]=factor['landmark_jacobian']
    return dict(residual=np.linalg.solve(whitening,residual),
                jacobian=np.linalg.solve(whitening,jacobian),state_stamps_ns=stamps,
                retained_columns=6*count)
