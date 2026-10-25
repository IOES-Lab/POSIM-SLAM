"""Raw bottom-lock DVL velocity, with synchronous raw gyro and covariance.

Velocity/covariance are in the DVL frame; gyro/covariance are in the IMU frame.
The caller must synchronize gyro to the DVL timestamp before submission, with
appropriate interpolation uncertainty. This packet never accepts a pose.
"""
import struct
import numpy as np


def encode_dvl(stamp_ns, velocity, covariance, *, gyro, gyro_stamp_ns,
               gyro_covariance, bottom_locked, locked_beams):
    if (type(stamp_ns) is not int or not 0 < stamp_ns < 2**63
            or type(gyro_stamp_ns) is not int or gyro_stamp_ns != stamp_ns
            or bottom_locked is not True or type(locked_beams) is not int
            or locked_beams not in (3, 4)):
        raise ValueError('dvl_clock_or_bottom_lock')
    vectors = []
    for value, limit in ((velocity, 50), (gyro, 35)):
        array = np.asarray(value)
        if array.dtype.kind not in 'iuf' or array.shape != (3,):
            raise ValueError('invalid_dvl_vector')
        array = array.astype(float)
        if not np.isfinite(array).all() or np.linalg.norm(array) > limit:
            raise ValueError('invalid_dvl_vector')
        vectors.extend(array.tolist())
    matrices = []
    for value, positive in ((covariance, True), (gyro_covariance, False)):
        array = np.asarray(value)
        if array.dtype.kind not in 'iuf' or array.shape != (3, 3):
            raise ValueError('invalid_dvl_covariance')
        array = array.astype(float)
        if (not np.isfinite(array).all() or
                np.linalg.norm(array-array.T) > 1e-10*np.linalg.norm(array)):
            raise ValueError('invalid_dvl_covariance')
        if positive:
            try:
                np.linalg.cholesky(array)
            except np.linalg.LinAlgError as error:
                raise ValueError('invalid_dvl_covariance') from error
        elif np.linalg.eigvalsh(array).min() < 0:
            raise ValueError('invalid_dvl_covariance')
        matrices.extend(array.ravel().tolist())
    return struct.pack('<qqII24d', stamp_ns, gyro_stamp_ns, 1, locked_beams,
                       *vectors, *matrices)
