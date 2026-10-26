"""Original-time auxiliary measurements for live and recorded native fusion."""
import bisect
import math
import numpy as np


def bracketed_gyro(samples, stamp_ns, *, model_sigma):
    """Interpolate raw gyro and covariance, never orientation or pose.

    Samples are (stamp_ns, angular_velocity, covariance_flat). Unknown ROS
    covariance contributes zero reported variance plus the explicit model term.
    The returned variance does not model correlation with IMU preintegration.
    """
    if not math.isfinite(model_sigma) or not 0 < model_sigma <= .5:
        raise ValueError('explicit_dvl_gyro_uncertainty_required')
    times = [sample[0] for sample in samples]
    if not times or any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError('dvl_gyro_clock')
    upper = bisect.bisect_left(times, stamp_ns)
    if upper == len(times) or (upper == 0 and times[0] != stamp_ns):
        raise ValueError('dvl_gyro_not_bracketed')

    def unpack(index):
        _, gyro, covariance = samples[index]
        gyro = np.asarray(gyro, dtype=float)
        covariance = np.asarray(covariance, dtype=float)
        if gyro.shape != (3,) or covariance.shape != (9,) or not np.isfinite(np.r_[gyro, covariance]).all():
            raise ValueError('invalid_dvl_gyro_sample')
        unknown = covariance[0] == -1 or not np.any(covariance)
        covariance = np.zeros((3, 3)) if unknown else covariance.reshape(3, 3)
        if (not np.allclose(covariance, covariance.T, atol=1e-12, rtol=0)
                or np.linalg.eigvalsh(covariance).min() < 0):
            raise ValueError('invalid_dvl_gyro_covariance')
        return gyro, covariance, int(unknown)

    gyro, covariance, unknown = unpack(upper)
    interpolated = times[upper] != stamp_ns
    if interpolated:
        gap = times[upper] - times[upper-1]
        if gap > 20_000_000:
            raise ValueError('dvl_gyro_bracket_gap')
        alpha = (stamp_ns-times[upper-1])/gap
        lower, lower_covariance, lower_unknown = unpack(upper-1)
        gyro = (1-alpha)*lower + alpha*gyro
        covariance = (1-alpha)**2*lower_covariance + alpha**2*covariance
        unknown += lower_unknown
    return dict(gyro=gyro.tolist(), covariance=(covariance+np.eye(3)*model_sigma**2).tolist(),
                interpolated=interpolated, unknown_covariance_inputs=unknown)


def hydrostatic_height(stamp_ns, pressure_pa, variance_pa2, *, reference_pa, density_kg_m3, gravity_m_s2):
    if (type(stamp_ns) is not int or stamp_ns <= 0
            or not all(math.isfinite(v) and v > 0 for v in
                       (pressure_pa, variance_pa2, reference_pa, density_kg_m3, gravity_m_s2))):
        raise ValueError('positive_pressure_variance_and_calibration_required')
    if pressure_pa-reference_pa <= 3*math.sqrt(variance_pa2):
        return dict(stamp_ns=stamp_ns, valid=False, reason='surface_clipping_or_insufficient_submergence')
    scale = density_kg_m3*gravity_m_s2
    packet = dict(stamp_ns=stamp_ns, height_m=-(pressure_pa-reference_pa)/scale,
                  variance_m2=variance_pa2/scale**2)
    from .pressure_packet import encode_pressure
    encode_pressure(**packet)
    return dict(stamp_ns=stamp_ns, valid=True, packet=packet)


def validate_auxiliary_ack(state, expected):
    """Validate exact-event ownership before committing any estimator output.

    ``expected`` maps selected sensor names to booleans indicating a valid
    measurement sent for this event. Reception alone does not prove fusion.
    """
    included=[]
    for kind, sent in expected.items():
        if kind not in ('pressure','dvl') or type(sent) is not bool:
            raise ValueError('invalid_auxiliary_ack_expectation')
        report=state.get(kind) or {}
        if not isinstance(report,dict) or report.get('owner_received',False) is not sent:
            raise RuntimeError('sonar_auxiliary_ack_mismatch')
        fused=report.get('optimization_included',False)
        if type(fused) is not bool or fused and not sent:
            raise RuntimeError('sonar_auxiliary_optimization_mismatch')
        if fused:included.append(kind)
    return tuple(included)


def fused_recent(state_stamp_ns, fused_stamp_ns, fused_wall, now, *, max_age_s=2.):
    """A recently optimized observation on both measurement and wall clocks."""
    if not math.isfinite(max_age_s) or max_age_s<=0:
        raise ValueError('invalid_auxiliary_freshness_request')
    return bool(fused_wall is not None and fused_stamp_ns>0 and state_stamp_ns>0
                and math.isfinite(now) and math.isfinite(fused_wall)
                and 0<=now-fused_wall<max_age_s
                and 0<=state_stamp_ns-fused_stamp_ns<max_age_s*1e9)


def dvl_status(value, *, nonce, frame):
    """Validate a session-owned DVL status; invalid lock is a timed absence.

    Returns (stamp_ns, payload), or None for another session or an untimed
    unavailable status. No velocity is invented for lost bottom lock.
    """
    if value.get('session_nonce') != nonce:
        return None
    if (value.get('frame_id') != frame or value.get('source') != '/vehicle/sensors/dvl'
            or value.get('version') != '0.1.0' or type(value.get('valid')) is not bool):
        raise ValueError('dvl_identity')
    if 'stamp' not in value:
        if value['valid']:raise ValueError('dvl_stamp_missing')
        return None
    sec, ns = value['stamp']['sec'], value['stamp']['nanosec']
    if type(sec) is not int or type(ns) is not int or sec < 0 or not 0 <= ns < 10**9:
        raise ValueError('dvl_stamp')
    payload = dict(valid=value['valid'])
    if value['valid']:
        if value.get('reason') != 'bottom_locked' or value.get('locked_beams') not in (3, 4):
            raise ValueError('dvl_bottom_lock')
        from .inertial_filter import covariance
        from .dvl_scale import vector
        payload.update(velocity=vector(value['velocity_m_s']).tolist(),
                       covariance=covariance(np.asarray(value['covariance']).reshape(3,3),3).tolist(),
                       locked_beams=value['locked_beams'])
    elif value.get('velocity_m_s') is not None or value.get('covariance') is not None:
        raise ValueError('invalid_dvl_return')
    return sec*10**9+ns, payload


def measured_dvl_packet(stamp_ns, payload, samples, *, gyro_sigma):
    """Build the shared native DVL packet at its original measurement time."""
    if payload.get('valid') is not True:raise ValueError('valid_dvl_required')
    gyro=bracketed_gyro(samples,stamp_ns,model_sigma=gyro_sigma)
    packet=dict(stamp_ns=stamp_ns,velocity=payload['velocity'],covariance=payload['covariance'],
                gyro=gyro['gyro'],gyro_stamp_ns=stamp_ns,gyro_covariance=gyro['covariance'],
                bottom_locked=True,locked_beams=payload['locked_beams'])
    from .dvl_packet import encode_dvl
    encode_dvl(**packet)
    return packet
