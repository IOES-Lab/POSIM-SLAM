"""Decode a ProjectedSonarImage without ROS dependencies or invented returns."""
import numpy as np


def projected_frame(message):
    image=message.image
    beams=image.beam_count;rows=len(message.ranges)
    if (type(beams) is not int or not 16<=beams<=4096 or not 16<=rows<=4096
            or beams*rows>2_000_000):
        raise ValueError('acoustic_shape_limit')
    if image.dtype!=image.DTYPE_FLOAT32:raise ValueError('unsupported_acoustic_dtype')
    if len(image.data)!=beams*rows*4:raise ValueError('acoustic_byte_count')
    if len(message.beam_directions)!=beams:raise ValueError('acoustic_direction_count')
    sec,nsec=message.header.stamp.sec,message.header.stamp.nanosec
    if type(sec) is not int or type(nsec) is not int or sec<0 or not 0<=nsec<1_000_000_000:
        raise ValueError('acoustic_timestamp')
    power=np.frombuffer(bytes(image.data),dtype='>f4' if image.is_bigendian else '<f4').reshape(rows,beams).astype(np.float32)
    ranges=np.asarray(message.ranges,dtype=float)
    directions=np.asarray([[v.x,v.y,v.z] for v in message.beam_directions],dtype=float)
    if np.isnan(power).any() or np.isposinf(power).any():raise ValueError('invalid_acoustic_intensity')
    if (not np.isfinite(ranges).all() or ranges[0]<0 or not (np.diff(ranges)>0).all()
            or not np.isfinite(directions).all()
            or not np.allclose(np.linalg.norm(directions,axis=1),1,atol=1e-5)):
        raise ValueError('invalid_acoustic_geometry')
    # Negative infinity represents no acoustic return and is retained for replay.
    return dict(intensity_db=power,ranges=ranges,beam_directions=directions,
                stamp=np.asarray([sec,nsec],dtype=np.int64))


def validate_frame(frame):
    power = np.asarray(frame['intensity_db'])
    ranges = np.asarray(frame['ranges'])
    directions = np.asarray(frame['beam_directions'])
    stamp = np.asarray(frame['stamp'])
    if (power.ndim != 2 or power.size > 2_000_000 or not 16 <= min(power.shape) or max(power.shape) > 4096
            or ranges.shape != (power.shape[0],) or directions.shape != (power.shape[1], 3)
            or stamp.shape != (2,) or not all(np.isfinite(v).all() for v in (power, ranges, directions, stamp))
            or not (np.diff(ranges) > 0).all() or ranges[0] < 0
            or stamp[0] < 0 or not np.equal(stamp,np.floor(stamp)).all()
            or not 0 <= stamp[1] < 1e9):
        raise ValueError('invalid_acoustic_frame')
    angles = np.arctan2(directions[:, 1], directions[:, 0])
    if (not (np.diff(angles) > 0).all() or np.max(np.abs(directions[:, 2])) > 1e-5
            or not np.allclose(np.linalg.norm(directions, axis=1), 1, atol=1e-5)):
        raise ValueError('invalid_acoustic_fan')
