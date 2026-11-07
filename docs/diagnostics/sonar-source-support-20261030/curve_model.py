"""Measured echo curves and ambiguous fan/plane shape hypotheses.

An eligible image edge is neither a physical first arrival nor an identified
reflector. Range/bearing curves retain unknown elevation and both mirrored
plane modes. No Cartesian observations, pose prior or calibrated covariance
are manufactured. These functions do not install SLAM factors or admit a map.
"""
from dataclasses import dataclass, field
import math

import numpy as np

from posim_slam.sonar_edges import EchoEdgeConfig, echo_boundaries


@dataclass(frozen=True)
class EchoCurveConfig:
    edge: EchoEdgeConfig = field(default_factory=EchoEdgeConfig)
    minimum_beams: int = 8
    maximum_edges: int = 65536


def echo_curves(frame, *, sensor_id, vertical_fov_rad, config=EchoCurveConfig()):
    """Group first eligible edges; retain every detected edge and short chain.

    Consecutive beams link only when their radial jump is within the existing
    edge rise window. Missing beams and larger jumps split chains. The radial
    band is that detector window, not a confidence interval or hardware noise.
    It must not be used to claim geometric first-return identity. Original
    stamp/sensor/beam/bin keys and all later eligible edges remain available.
    """
    if (type(sensor_id) is not int or sensor_id < 0
            or not math.isfinite(vertical_fov_rad) or not 0 < vertical_fov_rad < math.pi
            or not isinstance(config, EchoCurveConfig)
            or not isinstance(config.edge, EchoEdgeConfig)
            or type(config.minimum_beams) is not int or not 4 <= config.minimum_beams <= 4096
            or type(config.maximum_edges) is not int or not 1 <= config.maximum_edges <= 2_000_000):
        raise ValueError('invalid_echo_curve_config')
    detected = echo_boundaries(frame, config=config.edge)
    mask = detected['edge_mask']
    if int(mask.sum()) > config.maximum_edges:
        raise ValueError('echo_curve_edge_capacity')
    bins, beams = np.nonzero(mask)
    ranges = np.asarray(frame['ranges'], dtype=float)
    directions = np.asarray(frame['beam_directions'], dtype=float)
    angles = np.arctan2(directions[:, 1], directions[:, 0])
    first = np.full(mask.shape[1], -1, dtype=np.int64)
    for beam in range(mask.shape[1]):
        rows = np.flatnonzero(mask[:, beam])
        if len(rows):
            first[beam] = rows[0]
    chains, current = [], []
    for beam, index in enumerate(first):
        if (index < 0 or (current and abs(ranges[index]-ranges[first[current[-1]]])
                                       > config.edge.rise_span_m)):
            if current:
                chains.append(current)
                current = []
        if index >= 0:
            current.append(beam)
    if current:
        chains.append(current)
    stamp = int(frame['stamp'][0])*1_000_000_000+int(frame['stamp'][1])
    curves = []
    for chain in chains:
        ids = np.asarray(chain, dtype=np.int64)
        radial = first[ids]
        measured = ranges[radial]
        band = np.column_stack([measured-config.edge.rise_span_m/2,
                                measured+config.edge.rise_span_m/2])
        curves.append(dict(beam_ids=ids, range_bin_ids=radial.copy(),
            ranges_m=measured.copy(), azimuth_rad=angles[ids].copy(),
            detector_radial_band_m=band,
            source_keys=tuple((stamp, sensor_id, int(b), int(k)) for b, k in zip(ids, radial)),
            enough_beams=len(ids) >= config.minimum_beams))
    return dict(stamp_ns=stamp, sensor_id=sensor_id, vertical_fov_rad=vertical_fov_rad,
        edge_bin_ids=np.column_stack([bins, beams]), first_range_bin_ids=first,
        curves=curves, noise_db=detected['noise_db'],
        observed_elevation=False, physical_identity_verified=False, map_qualified=False,
        scope='eligible echo curves; not physical first arrivals or Cartesian observations')


def fan_plane_curve(inverse_plane, azimuth_rad, *, vertical_fov_rad, allow_missing=False):
    """Predict the first intersection of an infinite plane within each fan.

    The source-frame plane is q dot x = 1, q in inverse metres. The selected
    elevation is a MODEL extremum, never an observed angle. Reflection in the
    central fan plane leaves all ranges unchanged. The returned Jacobian is
    with respect to q; uncertain poses and acoustic beam mixing are excluded.
    For prediction in a second view, allow_missing=True retains nonintersecting
    beams with visible=False, infinite range and unavailable Jacobian/angle.
    """
    q = np.asarray(inverse_plane, dtype=float)
    angles = np.asarray(azimuth_rad, dtype=float)
    if (q.shape != (3,) or not np.isfinite(q).all() or np.linalg.norm(q) == 0
            or angles.ndim != 1 or not 1 <= len(angles) <= 4096
            or not np.isfinite(angles).all() or np.any(abs(angles) >= math.pi/2)
            or not math.isfinite(vertical_fov_rad) or not 0 < vertical_fov_rad < math.pi
            or type(allow_missing) is not bool):
        raise ValueError('invalid_fan_plane_curve')
    a = q[0]*np.cos(angles)+q[1]*np.sin(angles)
    half = vertical_fov_rad/2
    interior = np.clip(np.arctan2(q[2], a), -half, half)
    options = np.column_stack([np.full(len(angles), -half), interior,
                               np.full(len(angles), half)])
    values = a[:, None]*np.cos(options)+q[2]*np.sin(options)
    chosen = options[np.arange(len(angles)), np.argmax(values, axis=1)]
    rays = np.column_stack([np.cos(angles)*np.cos(chosen),
                            np.sin(angles)*np.cos(chosen), np.sin(chosen)])
    denominator = rays@q
    visible = denominator > 1e-10
    if not allow_missing and not visible.all():
        raise ValueError('fan_plane_curve_has_no_forward_return')
    safe = np.where(visible, denominator, 1.)
    return dict(ranges_m=np.where(visible, 1/safe, np.inf),
                jacobian=np.where(visible[:, None], -rays/safe[:, None]**2, np.nan),
                predicted_elevation_rad=np.where(visible, chosen, np.nan), visible=visible)


def fit_fan_plane_curve(ranges_m, azimuth_rad, *, vertical_fov_rad):
    """All-row shape fit with retained elevation-reflection ambiguity.

    Numerical damping only chooses a reference. It creates no prior and no
    covariance. Rank/free directions describe this conditional local shape,
    not joint navigation or a calibrated physical measurement. Correspondence,
    visibility, range-response bias and cross-beam mixing require qualification.
    No rows are removed and no mode is promoted to a unique 3D surface.
    """
    ranges = np.asarray(ranges_m, dtype=float)
    angles = np.asarray(azimuth_rad, dtype=float)
    if (ranges.ndim != 1 or not 4 <= len(ranges) <= 4096 or angles.shape != ranges.shape
            or not np.isfinite(ranges).all() or np.any(ranges <= 0)
            or not np.isfinite(angles).all() or not np.all(np.diff(angles) > 0)
            or np.any(abs(angles) >= math.pi/2)
            or not math.isfinite(vertical_fov_rad) or not 0 < vertical_fov_rad < math.pi):
        raise ValueError('invalid_echo_curve_fit')
    models = []
    for elevation in (0., vertical_fov_rad/2, -vertical_fov_rad/2):
        design = np.column_stack([np.cos(angles)*math.cos(elevation),
            np.sin(angles)*math.cos(elevation), np.full(len(angles), math.sin(elevation))])
        q = np.linalg.lstsq(design, 1/ranges, rcond=None)[0]
        try:
            current = fan_plane_curve(q, angles, vertical_fov_rad=vertical_fov_rad)
        except ValueError:
            continue
        damping, converged = 1e-3, False
        for iteration in range(50):
            j = current['jacobian']; residual = current['ranges_m']-ranges
            gradient = j.T@residual
            if np.linalg.norm(gradient, np.inf) < 1e-9:
                converged = True
                break
            h = j.T@j
            step = np.linalg.solve(h+damping*max(float(np.trace(h))/3, 1.)*np.eye(3), -gradient)
            try:
                trial = fan_plane_curve(q+step, angles, vertical_fov_rad=vertical_fov_rad)
            except ValueError:
                trial = None
            old = float(residual@residual)
            cost = float(np.sum((trial['ranges_m']-ranges)**2)) if trial is not None else math.inf
            if cost < old:
                q += step; current = trial; damping = max(damping/3, 1e-12)
                if np.linalg.norm(step) < 1e-9 or old-cost < 1e-12*max(1., old):
                    converged = True
                    break
            else:
                damping = min(damping*10, 1e15)
        if converged:
            models.append((float(np.sum((current['ranges_m']-ranges)**2)), q.copy()))
    if not models:
        raise ValueError('echo_curve_seed_not_converged')
    cost, q = min(models, key=lambda item: item[0])
    q[2] = abs(q[2])
    modes = [q.copy()]
    if q[2] > 64*np.finfo(float).eps*np.linalg.norm(q):
        modes.append(q*np.array([1., 1., -1.]))
    prediction = fan_plane_curve(q, angles, vertical_fov_rad=vertical_fov_rad)
    _, singular, vt = np.linalg.svd(prediction['jacobian'], full_matrices=False)
    rank = int(np.sum(singular > np.finfo(float).eps*max(len(ranges), 3)*singular[0]))
    return dict(plane_modes_inverse_m=np.asarray(modes), predicted_ranges_m=prediction['ranges_m'],
        residual_m=prediction['ranges_m']-ranges, rms_m=math.sqrt(cost/len(ranges)),
        singular_values=singular, rank=rank, unbounded_basis=vt[rank:].T,
        source_rows=len(ranges), added_prior=False, covariance_available=False,
        physical_identity_verified=False, map_qualified=False,
        scope='conditional fan/plane curve fit with mirror ambiguity; no point or navigation observation')
