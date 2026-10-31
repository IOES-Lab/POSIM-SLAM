"""Decode a ProjectedSonarImage without ROS dependencies or invented returns."""
import math
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
            or stamp.shape != (2,) or np.isnan(power).any() or np.isposinf(power).any()
            or not all(np.isfinite(v).all() for v in (ranges, directions, stamp))
            or not (np.diff(ranges) > 0).all() or ranges[0] < 0
            or stamp[0] < 0 or not np.equal(stamp,np.floor(stamp)).all()
            or not 0 <= stamp[1] < 1e9):
        raise ValueError('invalid_acoustic_frame')
    angles = np.arctan2(directions[:, 1], directions[:, 0])
    if (not (np.diff(angles) > 0).all() or np.max(np.abs(directions[:, 2])) > 1e-5
            or not np.allclose(np.linalg.norm(directions, axis=1), 1, atol=1e-5)):
        raise ValueError('invalid_acoustic_fan')


def validate_calibrated_frame(frame, *, beams, horizontal_fov, range_max,
                              bandwidth_hz, sound_speed_m_s):
    """Check the source's uniform physical grid, including its covering cell.

    This is the POSIM broadband sampling contract, not a real-sonar noise model.
    A final cell centre may exceed the render clip. Its actual coordinate must
    match c/(2B); neither stretching nor clamping the published axis is allowed.
    """
    validate_frame(frame)
    values=(horizontal_fov,range_max,bandwidth_hz,sound_speed_m_s)
    if (type(beams) is not int or not 16<=beams<=4096
            or not all(math.isfinite(v) and v>0 for v in values)
            or horizontal_fov>=math.pi):
        raise ValueError('invalid_acoustic_calibration')
    intervals=2*range_max*bandwidth_hz/sound_speed_m_s
    if not math.isfinite(intervals) or not 0<intervals<=4095:
        raise ValueError('acoustic_calibration_shape_limit')
    count=math.ceil(intervals)+1
    if not 16<=count<=4096 or np.asarray(frame['intensity_db']).shape!=(count,beams) or count*beams>2_000_000:
        raise ValueError('acoustic_calibrated_shape')
    spacing=sound_speed_m_s/(2*bandwidth_hz)
    expected=np.arange(count,dtype=float)*spacing
    # The source rounds its reciprocal bandwidth, products and published range
    # values to float32. Four unit roundoffs cover these arithmetic steps; this
    # allowance scales with coordinates and is not a physical range tolerance.
    rounding=4*np.finfo(np.float32).eps*np.maximum(expected,spacing)
    if np.any(np.abs(np.asarray(frame['ranges'])-expected)>rounding):
        raise ValueError('acoustic_calibrated_range_grid')
    angles=np.arctan2(frame['beam_directions'][:,1],frame['beam_directions'][:,0])
    if not np.allclose(angles,np.linspace(-horizontal_fov/2,horizontal_fov/2,beams),atol=1e-5,rtol=0):
        raise ValueError('acoustic_calibrated_bearings')


def log_image_values(power, *, dtype=float):
    """Bound censored log pixels only in an image working copy; return support.

    Negative infinity is zero acoustic power. Image filtering uses the lowest
    finite measured level in its place, then masks out no-return pixels. This
    representation is never a raw observation, noise estimate or likelihood.
    Finite-only input is unchanged, including its dtype conversion.
    """
    values=np.asarray(power,dtype=dtype)
    if values.ndim!=2 or np.isnan(values).any() or np.isposinf(values).any():
        raise ValueError('invalid_acoustic_intensity')
    finite=np.isfinite(values)
    if finite.all():return values,finite
    if not finite.any():return np.zeros_like(values),finite
    return np.where(finite,values,np.min(values[finite])),finite


def log_quantile(values, q, *, axis=None):
    """Linear log quantile with the zero-power limit retained as -infinity.

    NumPy interpolation can form inf-inf for equal censored endpoints. Resolve
    that extended-real limit explicitly, without inventing a finite noise floor.
    Finite-only inputs use the original NumPy calculation unchanged.
    """
    values=np.asarray(values,dtype=float)
    if np.isnan(values).any() or np.isposinf(values).any():
        raise ValueError('invalid_acoustic_intensity')
    if np.isfinite(values).all():return np.quantile(values,q,axis=axis)
    if not np.isfinite(q) or not 0<=q<=1:raise ValueError('invalid_acoustic_quantile')
    if axis is None:values=values.reshape(-1);axis=0
    size=values.shape[axis]
    if not size:raise ValueError('empty_acoustic_quantile')
    index=(size-1)*q;lo=int(math.floor(index));hi=int(math.ceil(index))
    ordered=np.partition(values,(lo,hi),axis=axis)
    low=np.take(ordered,lo,axis=axis);high=np.take(ordered,hi,axis=axis)
    finite=np.isfinite(low)&np.isfinite(high)
    result=np.full(np.shape(low),-np.inf,dtype=float)
    np.subtract(high,low,out=result,where=finite)
    np.multiply(result,index-lo,out=result,where=finite)
    np.add(result,low,out=result,where=finite)
    return result
